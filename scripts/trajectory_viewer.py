#!/usr/bin/env python3
"""Draw/edit human paths, publish to Gazebo, and compare actual GT in a live UI."""

import argparse
import csv
from dataclasses import asdict
from datetime import datetime, timezone
import json
import hashlib
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

import rclpy
from rclpy.executors import ExternalShutdownException
from PyQt5 import QtCore, QtWidgets
from matplotlib import rcParams
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT
from matplotlib.figure import Figure

rcParams["keymap.save"] = []

if __package__:
    from .ground_truth_recorder import GroundTruthRecorder, observation, parse_mappings
    from .trajectory_io import TrajectorySample, load_trajectory, write_json
    from .trajectory_live_data import LiveComparison, LiveTracks, live_summary
    from .trajectory_editor import TrajectoryEditor, ViewControls, choose_csv_destination
    from .trajectory_view_frame import ViewFrame
    from .trajectory_path import trajectory_csv_bytes
    from .generate_trajectory import TrajectoryPoint
else:
    from ground_truth_recorder import GroundTruthRecorder, observation, parse_mappings
    from trajectory_io import TrajectorySample, load_trajectory, write_json
    from trajectory_live_data import LiveComparison, LiveTracks, live_summary
    from trajectory_editor import TrajectoryEditor, ViewControls, choose_csv_destination
    from trajectory_view_frame import ViewFrame
    from trajectory_path import trajectory_csv_bytes
    from generate_trajectory import TrajectoryPoint


class LiveRecorder(GroundTruthRecorder):
    def __init__(self, output_dir, mappings, model, keepalive_topic):
        self.model = model
        super().__init__(output_dir, mappings, timeout=3600, keepalive_topic=keepalive_topic)
        self.metadata["live_gt_interpolation"] = "linear between observed endpoints only; raw GT CSV is unchanged"
        write_json(self.output_dir/"ground_truth_metadata.json", self.metadata)

    def receive(self, message, person):
        count = self.counts[person]
        super().receive(message, person)
        if self.counts[person] != count:
            stamp, row = observation(message, person)
            self.model.add(TrajectorySample(person, stamp, row[2], row[3], row[4]))

    def check_progress(self):
        # Live display can resume after a gap: only fill it when the next real
        # observation arrives. Never repeat cached pose as a new observation.
        now = self.get_clock().now().nanoseconds
        if self.last_clock is not None and now < self.last_clock:
            self.error = "Simulation reset: click Start for a new session"
            self.done = True
        self.last_clock = now


class ViewToolbar(NavigationToolbar2QT):
    toolitems = tuple(item for item in NavigationToolbar2QT.toolitems if item[3] != "save_figure")


class TrajectoryWindow(QtWidgets.QMainWindow):
    def __init__(self, csv_path, output_root, mappings, *, rate=50, start_delay=1,
                 keepalive_topic="/j100_0000/sensors/camera_0/color/image"):
        super().__init__()
        self.output_root = Path(output_root)
        self.mappings = mappings
        self.rate = rate
        self.start_delay = start_delay
        self.keepalive_topic = keepalive_topic
        self.recorder = None
        self.publisher = None
        self.publisher_log = None
        self.pending_start = False
        self.run_started = False
        self.notice = ""
        self.last_draw = 0.0
        self.last_version = None
        self.path_specs = {}
        self.view_frame = ViewFrame()
        self.setWindowTitle("Human Trajectory — Draw / Publish / Ground Truth")
        self.resize(1380, 940)
        container = QtWidgets.QWidget()
        self.setCentralWidget(container)
        layout = QtWidgets.QVBoxLayout(container)
        controls = QtWidgets.QHBoxLayout()
        self.open_button = QtWidgets.QPushButton("Open CSV")
        self.edit_button = QtWidgets.QPushButton("Draw / edit path")
        self.save_button = QtWidgets.QPushButton("Save target CSV")
        self.start_button = QtWidgets.QPushButton("Start trajectory")
        self.stop_button = QtWidgets.QPushButton("Stop publishing")
        self.method_box = QtWidgets.QComboBox()
        self.method_box.addItem("Target: linear", "linear")
        self.method_box.addItem("Target: hold samples", "previous")
        for control in (self.open_button, self.edit_button, self.save_button,
                        self.start_button, self.stop_button, self.method_box):
            controls.addWidget(control)
        controls.addStretch()
        layout.addLayout(controls)
        self.view_controls = ViewControls(self.view_frame.yaw_deg)
        layout.addWidget(self.view_controls)
        self.csv_label = QtWidgets.QLabel()
        self.csv_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(self.csv_label)
        self.figure = Figure(figsize=(13, 8), constrained_layout=True)
        self.canvas = FigureCanvasQTAgg(self.figure)
        grid = self.figure.add_gridspec(3, 2, width_ratios=(1.15, 1))
        self.xy = self.figure.add_subplot(grid[:, 0])
        self.x_time = self.figure.add_subplot(grid[0, 1])
        self.y_time = self.figure.add_subplot(grid[1, 1], sharex=self.x_time)
        self.error_axis = self.figure.add_subplot(grid[2, 1], sharex=self.x_time)
        self.toolbar = ViewToolbar(self.canvas, self)
        layout.addWidget(self.toolbar)
        layout.addWidget(self.canvas, 1)
        self.metrics_label = QtWidgets.QLabel()
        self.metrics_label.setWordWrap(True)
        layout.addWidget(self.metrics_label)
        self.status_label = QtWidgets.QLabel()
        self.status_label.setWordWrap(True)
        layout.addWidget(self.status_label)
        self.directory_label = QtWidgets.QLabel()
        self.directory_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(self.directory_label)
        self.open_button.clicked.connect(self.choose_csv)
        self.edit_button.clicked.connect(self.edit_path)
        self.save_button.clicked.connect(self.save_target)
        self.start_button.clicked.connect(self.start_run)
        self.stop_button.clicked.connect(self.stop_publishing)
        self.method_box.currentIndexChanged.connect(lambda: self.draw(force=True))
        self.view_controls.changed.connect(self.change_view)
        if csv_path:
            self.load_csv(csv_path)
        else:
            self.set_target([], b"", "No target yet. Click Draw / edit path or Open CSV.")
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(20)

    def load_csv(self, path):
        path = Path(path).resolve()
        samples = load_trajectory(path)
        self.set_target(samples, path.read_bytes(), f"Target CSV: {path}", path=path)

    def set_target(self, samples, data, label, *, path=None, path_specs=None):
        # Validate identities before closing a current experiment. Frozen bytes
        # are the exact file sent to the existing Publisher for this experiment.
        LiveTracks(samples, self.mappings, self.rate)
        self.finish_session()
        self.csv_path = path
        self.csv_bytes = data
        self.csv_hash = hashlib.sha256(data).hexdigest()
        self.samples = samples
        self.path_specs = path_specs or {}
        self.create_session()
        self.csv_label.setText(label)
        self.update_controls()

    def edit_path(self):
        if self.pending_start or (self.publisher and self.publisher.poll() is None):
            return
        try:
            first = next(iter(self.model.targets), None)
            editor = TrajectoryEditor(self.mappings, self.model.targets,
                                      draft=self.path_specs.get(first), view_yaw=self.view_frame.yaw_deg, parent=self)
            accepted = editor.exec_() == QtWidgets.QDialog.Accepted
            self.view_controls.yaw.setValue(editor.view_frame.yaw_deg)
            if not accepted:
                return
            self.use_editor_path(editor.result_samples, editor.result_spec, send=editor.send_requested)
        except (ValueError, OSError, RuntimeError) as error:
            QtWidgets.QMessageBox.warning(self, "Path error", str(error))

    def use_editor_path(self, points, spec, *, send=False):
        person = spec["person_id"]
        # Preserve other actors' paths if the CSV already contains several IDs.
        combined = [TrajectoryPoint(identity, p.timestamp_ns/1e9, p.x, p.y, p.yaw)
                    for identity, samples in self.model.targets.items() if identity != person
                    for p in samples]
        combined.extend(points)
        combined.sort(key=lambda point: (point.timestamp, point.person_id))
        samples = [TrajectorySample(p.person_id, round(p.timestamp*1e9), p.x, p.y, p.yaw) for p in combined]
        specs = dict(self.path_specs)
        specs[person] = spec
        data = trajectory_csv_bytes(combined)
        self.set_target(samples, data, f"Drawn target: {person} | {len(points)} samples | {points[-1].timestamp:.2f} s",
                        path_specs=specs)
        self.notice = "Drawn path ready. Click Start trajectory to send it to Gazebo."
        if send:
            self.start_run()

    def save_target(self):
        if not self.samples:
            return
        try:
            default = self.csv_path or Path.cwd()/"trajectories"/"drawn_path.csv"
            destination = choose_csv_destination(self, default)
            if destination:
                destination.parent.mkdir(parents=True, exist_ok=True)
                destination.write_bytes(self.csv_bytes)
                self.notice = f"Target CSV saved: {destination}"
        except OSError as error:
            QtWidgets.QMessageBox.warning(self, "Save error", str(error))

    def update_controls(self):
        running = bool(self.publisher and self.publisher.poll() is None)
        busy = running or self.pending_start
        self.open_button.setEnabled(not busy)
        self.edit_button.setEnabled(not busy)
        self.save_button.setEnabled(bool(self.samples) and not busy)
        self.start_button.setEnabled(bool(self.samples) and not busy)
        self.stop_button.setEnabled(busy)

    def change_view(self, yaw):
        # Only redraw artists. ROS data, replay origin and experiment stay intact.
        self.view_frame = ViewFrame(yaw)
        mode = str(self.toolbar.mode)
        if "pan" in mode:
            self.toolbar.pan()
        elif "zoom" in mode:
            self.toolbar.zoom()
        self.toolbar.update()
        self.configure_axes()
        self.last_version = None
        self.draw(force=True)

    def create_session(self):
        self.model = LiveTracks(self.samples, self.mappings, self.rate)
        name = datetime.now(timezone.utc).strftime("run_%Y%m%dT%H%M%SZ_")+uuid.uuid4().hex[:6]
        directory = self.output_root/name
        self.recorder = LiveRecorder(directory, self.mappings, self.model, self.keepalive_topic)
        if self.path_specs:
            (directory/"drawn_path.csv").write_bytes(self.csv_bytes)
            write_json(directory/"drawn_path.json", {"frame_id":"world", "paths":self.path_specs})
        self.run_started = False
        self.pending_start = False
        self.notice = "Draw / edit a path or open CSV. Publishing waits for /clock and actual GT."
        self.directory_label.setText(f"CSV recording: {directory}")
        self.last_version = None
        self.configure_axes()
        self.draw(force=True)

    def configure_axes(self):
        for axis in (self.xy, self.x_time, self.y_time, self.error_axis):
            axis.clear()
            axis.grid(True, color="0.86", linewidth=0.7)
            axis.tick_params(direction="in", top=True, right=True, labelsize=9)
        self.xy.set_title("Target / Ground Truth — world XY in Gazebo view")
        self.view_frame.decorate(self.xy)
        self.xy.set_aspect("equal", adjustable="box")
        self.xy.margins(x=0.12, y=0.18)
        self.x_time.set(ylabel="X (m)", title="X position over time")
        self.y_time.set(ylabel="Y (m)", title="Y position over time")
        self.error_axis.set(xlabel="Time from replay origin (s)", ylabel="Distance error (m)", title="Distance between Target and Ground Truth")
        self.x_time.tick_params(labelbottom=False)
        self.y_time.tick_params(labelbottom=False)
        self.lines = {}
        curve_handles, marker_handles = [], []
        self.xy_summary = self.xy.text(0.02, 0.98, "Waiting for aligned GT", transform=self.xy.transAxes,
            ha="left", va="top", fontsize=9, fontweight="bold",
            bbox={"facecolor":"white", "edgecolor":"0.8", "alpha":0.9, "pad":4})
        multiple = len(self.model.targets) > 1
        styles = ("-", "--", ":", "-.")
        for index, (person, samples) in enumerate(self.model.targets.items()):
            prefix = person+" " if multiple else ""
            style = styles[index % len(styles)]
            target, = self.xy.plot(*self.view_frame.series((p.x, p.y) for p in samples),
                color="#d62728", linestyle=style, linewidth=2.8, label=prefix+"Target CSV")
            filled, = self.xy.plot([], [], color="black", linestyle=style, linewidth=1.4,
                label=prefix+"Ground Truth (linear)")
            observed, = self.xy.plot([], [], ".", color="black", alpha=0.5, markersize=3,
                label=prefix+"Measured GT")
            current, = self.xy.plot([], [], "o", color="black", markersize=5)
            target_now, = self.xy.plot([], [], "o", markerfacecolor="none",
                markeredgecolor="#d62728", markersize=6)
            distance_link, = self.xy.plot([], [], ":", color="0.4", linewidth=1)
            start, = self.xy.plot(*self.view_frame.series([(samples[0].x, samples[0].y)]), "*", markersize=13,
                markerfacecolor="#d62728", markeredgecolor="black", label=prefix+"Start point")
            target_end, = self.xy.plot(*self.view_frame.series([(samples[-1].x, samples[-1].y)]), "^", markersize=10,
                markerfacecolor="none", markeredgecolor="#d62728", markeredgewidth=1.8,
                label=prefix+"Target end")
            gt_end, = self.xy.plot([], [], "^", markersize=7, markerfacecolor="none",
                markeredgecolor="black", markeredgewidth=1.8, label=prefix+"GT end")
            relative = [p.timestamp_ns/1e9 for p in samples]
            target_x, = self.x_time.plot(relative, [p.x for p in samples], color="#d62728",
                linewidth=1.8, linestyle=style, label=prefix+"Target")
            target_y, = self.y_time.plot(relative, [p.y for p in samples], color="#d62728",
                linewidth=1.8, linestyle=style, label=prefix+"Target")
            gt_x, = self.x_time.plot([], [], color="black", linestyle="--", linewidth=1.2,
                label=prefix+"Ground Truth")
            gt_y, = self.y_time.plot([], [], color="black", linestyle="--", linewidth=1.2,
                label=prefix+"Ground Truth")
            error, = self.error_axis.plot([], [], color=f"C{index % 10}", linewidth=1.5,
                label=prefix+"Distance error")
            error_now, = self.error_axis.plot([], [], "o", color=f"C{index % 10}", markersize=4)
            self.lines[person] = {"gt":filled, "measured":observed, "current":current,
                "target":target, "start":start, "target_end":target_end,
                "target_now":target_now, "distance_link":distance_link,
                "gt_end":gt_end, "target_x":target_x, "target_y":target_y,
                "gt_x":gt_x, "gt_y":gt_y, "error":error, "error_now":error_now}
            curve_handles.extend((target, filled, observed))
            marker_handles.extend((start, target_end, gt_end))
        if curve_handles:
            curve_legend = self.xy.legend(handles=curve_handles, loc="upper right", fontsize=8, framealpha=0.95)
            self.xy.add_artist(curve_legend)
            self.xy.legend(handles=marker_handles, loc="lower left", fontsize=8, framealpha=0.95)
            for axis in (self.x_time, self.y_time, self.error_axis):
                axis.legend(loc="upper right", fontsize=8)
        else:
            self.xy.set_xlim(-2.2, 2.2)
            self.xy.set_ylim(-2.2, 2.2)

    def choose_csv(self):
        directory = self.csv_path.parent if self.csv_path else Path.cwd()/"trajectories"
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Choose target trajectory", str(directory), "CSV (*.csv)")
        if path:
            try:
                self.load_csv(path)
            except (ValueError, OSError) as error:
                QtWidgets.QMessageBox.warning(self, "CSV error", str(error))

    def start_run(self):
        if not self.samples or self.pending_start or (self.publisher and self.publisher.poll() is None):
            return
        try:
            if self.run_started or self.recorder.done:
                self.finish_session()
                self.create_session()
            self.pending_start = True
            self.notice = "Waiting for clock, GT and actor controller before publishing."
            self.update_controls()
        except (ValueError, OSError, RuntimeError) as error:
            self.notice = str(error)

    def begin_publishing(self):
        snapshot = self.recorder.output_dir/"input.csv"
        snapshot.write_bytes(self.csv_bytes)
        self.publisher_log = (self.recorder.output_dir/"publisher.log").open("w")
        script = Path(__file__).with_name("trajectory_publisher.py")
        self.publisher = subprocess.Popen([
            sys.executable, str(script), "--csv", str(snapshot),
            "--experiment-dir", str(self.recorder.output_dir),
            "--start-delay", str(self.start_delay), "--wait-for-subscribers",
            "--ros-args", "-p", "use_sim_time:=true"],
            stdout=self.publisher_log, stderr=subprocess.STDOUT, start_new_session=True)
        self.pending_start = False
        self.run_started = True
        self.notice = "Publishing trajectory; GT gaps fill only after both endpoints arrive."

    def stop_publishing(self):
        self.pending_start = False
        if self.publisher and self.publisher.poll() is None:
            os.killpg(self.publisher.pid, signal.SIGINT)
            try:
                self.publisher.wait(timeout=2)
            except subprocess.TimeoutExpired:
                os.killpg(self.publisher.pid, signal.SIGKILL)
                self.publisher.wait()
        if self.publisher_log:
            self.publisher_log.close()
            self.publisher_log = None
        self.notice = "Publishing stopped; actual GT observation continues."
        self.update_controls()

    def tick(self):
        try:
            deadline = time.monotonic()+0.005
            for _ in range(100):
                if time.monotonic() >= deadline:
                    break
                rclpy.spin_once(self.recorder, timeout_sec=0)
            if self.recorder.error:
                self.stop_publishing()
                self.notice = self.recorder.error
            if self.pending_start:
                ready = self.recorder.get_clock().now().nanoseconds > 0
                ready &= all(self.recorder.counts[person] > 0 for person in self.model.targets)
                if ready:
                    self.begin_publishing()
            playback = self.recorder.output_dir/"playback.json"
            if self.model.start_ns is None and playback.exists():
                metadata = json.loads(playback.read_text())
                if metadata["target_sha256"] != self.csv_hash or metadata["experiment_id"] != self.recorder.metadata["experiment_id"]:
                    raise ValueError("playback metadata does not match this CSV/session")
                self.model.set_origin(metadata["start_ros_time_ns"])
            if self.publisher and self.publisher.poll() is not None and self.publisher_log:
                code = self.publisher.returncode
                self.publisher_log.close()
                self.publisher_log = None
                self.notice = "Trajectory complete. Click Start for a fresh recording." if code == 0 else "Publisher stopped with an error; see publisher.log."
                self.save_comparison()
            self.update_controls()
            if time.monotonic()-self.last_draw >= 0.15:
                self.draw()
        except (ValueError, OSError, RuntimeError, KeyError) as error:
            self.stop_publishing()
            self.notice = str(error)
            self.status_label.setText(self.notice)
        except ExternalShutdownException:
            self.close()

    def draw(self, force=False):
        method = self.method_box.currentData()
        version = (self.model.version, method, self.view_frame.yaw_deg)
        if force or version != self.last_version:
            summaries = []
            plot_summaries = []
            for person in self.model.targets:
                rows, complete = self.model.compare(person, method)
                raw = self.model.measured_in_view(person)
                artists = self.lines[person]
                artists["gt"].set_data(*self.view_frame.series((row.gt_x, row.gt_y) for row in rows))
                stride = max(1, len(raw)//5000)
                artists["measured"].set_data(*self.view_frame.series((p.x, p.y) for p in raw[::stride]))
                latest = self.model.history[person][-1:]
                artists["current"].set_data(*self.view_frame.series((p.x, p.y) for p in latest))
                artists["target_now"].set_data(*self.view_frame.series((r.target_x, r.target_y) for r in rows[-1:]))
                link = [(rows[-1].target_x, rows[-1].target_y), (rows[-1].gt_x, rows[-1].gt_y)] if rows else []
                artists["distance_link"].set_data(*self.view_frame.series(link))
                endpoint = rows[-1:] if complete else []
                artists["gt_end"].set_data(*self.view_frame.series((p.gt_x, p.gt_y) for p in endpoint))
                times = [row.timestamp for row in rows]
                artists["gt_x"].set_data(times, [row.gt_x for row in rows])
                artists["gt_y"].set_data(times, [row.gt_y for row in rows])
                drawstyle = "steps-post" if method == "previous" else "default"
                artists["target_x"].set_drawstyle(drawstyle)
                artists["target_y"].set_drawstyle(drawstyle)
                artists["error"].set_data(times, [row.position_error for row in rows])
                artists["error_now"].set_data([row.timestamp for row in rows[-1:]],
                    [row.position_error for row in rows[-1:]])
                metrics = live_summary(person, rows, complete)
                if metrics:
                    target = self.model.targets[person]
                    length = math.fsum(math.hypot(b.x-a.x, b.y-a.y) for a,b in zip(target, target[1:]))
                    gt_length = math.fsum(math.hypot(b.gt_x-a.gt_x, b.gt_y-a.gt_y) for a,b in zip(rows, rows[1:]))
                    plot_summaries.append(f"{person}  Mean={metrics['mean_position_error_m']:.3f} m  RMSE={metrics['rmse_m']:.3f} m\nTarget length={length:.3f} m  GT length={gt_length:.3f} m")
                    summaries.append(f"{person}: t={rows[-1].timestamp:.2f} s, distance={rows[-1].position_error:.4f} m | mean {metrics['mean_position_error_m']:.4f} m | RMSE {metrics['rmse_m']:.4f} m | max {metrics['maximum_position_error_m']:.4f} m | yaw {metrics['mean_absolute_yaw_error_rad']:.4f} rad | interpolated {metrics['interpolated_gt_samples']}/{metrics['samples']}")
            waiting = "Waiting for aligned Ground Truth" if self.samples else "Draw / edit path or open a CSV"
            self.xy_summary.set_text("\n".join(plot_summaries) or waiting)
            self.metrics_label.setText("\n".join(summaries) or waiting)
            if not self.toolbar.mode:
                for axis in (self.xy, self.x_time, self.y_time, self.error_axis):
                    axis.relim()
                    axis.autoscale_view()
                peak = max((max(artists["error"].get_ydata(), default=0)
                            for artists in self.lines.values()), default=0)
                self.error_axis.set_ylim(0, max(0.001, peak*1.15))
            self.canvas.draw_idle()
            self.last_version = version
        now = self.recorder.get_clock().now().nanoseconds
        counts = ", ".join(f"{person}: {count}" for person, count in self.recorder.counts.items())
        stale = any(now-stamp > 2_000_000_000 for stamp in self.recorder.previous.values())
        suffix = " No fresh GT; waiting for the next real observation before filling the gap." if stale else ""
        self.status_label.setText(f"{self.notice}{suffix}\nRaw observations — {counts}")
        self.last_draw = time.monotonic()

    def save_comparison(self):
        all_rows, summaries = [], []
        method = self.method_box.currentData()
        for person in self.model.targets:
            rows, complete = self.model.compare(person, method)
            all_rows.extend(rows)
            summary = live_summary(person, rows, complete)
            if summary:
                summaries.append(summary)
        if not all_rows:
            return
        directory = self.recorder.output_dir
        with (directory/"live_comparison.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(LiveComparison.__dataclass_fields__))
            writer.writeheader()
            writer.writerows(asdict(row) for row in all_rows)
        with (directory/"live_metrics.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(summaries[0]))
            writer.writeheader()
            writer.writerows(summaries)
        write_json(directory/"live_metadata.json", {
            "frame_id":"world", "start_ros_time_ns":self.model.start_ns,
            "target_interpolation":method, "gt_interpolation":"linear between observed endpoints",
            "requested_grid_rate_hz":self.rate, "graph_point_limit":5000,
            "metric_basis":"display comparison grid; interpolated GT is explicitly marked",
            "initial_placement":"pre-playback waiting poses excluded from interpolation; comparison begins at first in-run GT observation",
            "raw_ground_truth_csv":"ground_truth.csv", "target_sha256":self.csv_hash,
            "xy_display_camera_yaw_deg":self.view_frame.yaw_deg,
            "display_transform":"view only; CSV, ROS messages and error calculations remain in world XY",
        })

    def finish_session(self):
        if self.recorder is None:
            return
        self.stop_publishing()
        self.save_comparison()
        self.recorder.finish()
        self.recorder.destroy_node()
        self.recorder = None

    def closeEvent(self, event):
        if hasattr(self, "timer"):
            self.timer.stop()
        self.finish_session()
        event.accept()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, help="optional target CSV; omit to draw a new path")
    parser.add_argument("--output-root", type=Path, default=Path("results/live"))
    parser.add_argument("--mapping", action="append", help="person_id=actor_name; default: human_1=walking_person")
    parser.add_argument("--interpolation-rate", type=float, default=50)
    parser.add_argument("--start-delay", type=float, default=1)
    parser.add_argument("--keepalive-image-topic", default="/j100_0000/sensors/camera_0/color/image")
    args = parser.parse_args()
    try:
        mappings = parse_mappings(args.mapping or ["human_1=walking_person"])
        samples = load_trajectory(args.csv) if args.csv else []
        LiveTracks(samples, mappings, args.interpolation_rate)
        if not math.isfinite(args.start_delay) or args.start_delay < 0:
            raise ValueError("start delay must be finite and nonnegative")
    except (ValueError, OSError) as error:
        parser.error(str(error))
    app = QtWidgets.QApplication([sys.argv[0]])
    rclpy.init(args=[])
    window = None
    try:
        window = TrajectoryWindow(args.csv, args.output_root, mappings,
            rate=args.interpolation_rate, start_delay=args.start_delay,
            keepalive_topic=args.keepalive_image_topic)
        window.show()
        return app.exec_()
    finally:
        if window:
            window.finish_session()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
