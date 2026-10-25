#!/usr/bin/env python3
"""Record Gazebo renderer-produced Actor WorldPose observations, never targets."""

import argparse
import csv
from datetime import datetime, timezone
import math
from pathlib import Path
import re
import sys
import time
import uuid

import rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args

if __package__:
    from .trajectory_io import FIELDS, NANOSECONDS, write_json
else:
    from trajectory_io import FIELDS, NANOSECONDS, write_json


def parse_mappings(values):
    mappings = {}
    for value in values:
        parts = value.split("=", 1)
        if len(parts) != 2 or not all(re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", part) for part in parts):
            raise ValueError("mapping must be person_id=actor_name, e.g. human_1=walking_person")
        person, actor = parts
        if person in mappings or actor in mappings.values():
            raise ValueError("duplicate person or actor mapping")
        mappings[person] = actor
    return mappings


def observation(message, person):
    if message.header.frame_id != "world":
        raise ValueError("GT frame must be world")
    stamp = message.header.stamp.sec * NANOSECONDS + message.header.stamp.nanosec
    if stamp <= 0 or message.header.stamp.nanosec >= NANOSECONDS:
        raise ValueError("GT must have a positive simulation timestamp")
    p, q = message.pose.position, message.pose.orientation
    if not all(math.isfinite(value) for value in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
        raise ValueError("nonfinite GT observation")
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if norm < 1e-9:
        raise ValueError("invalid GT quaternion")
    x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
    yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    return stamp, (person, stamp / NANOSECONDS, p.x, p.y, yaw)


class GroundTruthRecorder(Node):
    def __init__(self, output_dir, mappings, duration=None, timeout=30.0,
                 keepalive_topic="/j100_0000/sensors/camera_0/color/image"):
        super().__init__("ground_truth_recorder", parameter_overrides=[Parameter("use_sim_time", value=True)])
        if not self.get_parameter("use_sim_time").value:
            raise ValueError("ground truth recording requires use_sim_time=true")
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=False)
        self.duration = duration
        self.timeout = timeout
        self.started_wall = time.monotonic()
        self.first_stamp = None
        self.last_clock = None
        self.previous = {}
        self.counts = {person: 0 for person in mappings}
        self.stream = None
        self.writer = None
        self.done = False
        self.error = None
        self.metadata = {
            "schema_version": 1, "experiment_id": str(uuid.uuid4()),
            "frame_id": "world", "time_basis": "simulation_absolute",
            "world": "jackal_actor_world", "mappings": mappings,
            "gt_source": "gazebo_rendered_actor_world_pose", "rate_hz": 20,
            "timestamp_semantics": "rendering Scene.Time() at SceneUpdate, in the /clock simulation epoch",
            "keepalive_image_topic": keepalive_topic,
            "created_utc": datetime.now(timezone.utc).isoformat(), "status": "waiting",
        }
        write_json(self.output_dir / "ground_truth_metadata.json", self.metadata)
        self.subscriptions_by_person = []
        if keepalive_topic:
            self.image_keepalive = self.create_subscription(
                Image, keepalive_topic, lambda message: None, qos_profile_sensor_data)
        for person, actor in mappings.items():
            topic = f"/humans/{person}/ground_truth"
            self.subscriptions_by_person.append(self.create_subscription(
                PoseStamped, topic, lambda msg, identity=person: self.receive(msg, identity), 100))
            self.get_logger().info(f"Observing {actor} as {person}: {topic}")
        self.poll_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.1, self.check_progress, clock=self.poll_clock)
        self.get_logger().info(f"Experiment directory: {self.output_dir}; waiting for /clock and rendered WorldPose")

    def receive(self, message, person):
        if self.done or self.get_clock().now().nanoseconds <= 0:
            return
        try:
            stamp, row = observation(message, person)
            if stamp < self.previous.get(person, -1):
                raise ValueError("GT time moved backwards; start a new experiment after resetting Gazebo")
            if stamp == self.previous.get(person):
                return
            if self.stream is None:
                self.stream = (self.output_dir / "ground_truth.csv").open("x", newline="", encoding="utf-8", buffering=1)
                self.writer = csv.writer(self.stream)
                self.writer.writerow(FIELDS)
                self.first_stamp = stamp
                self.metadata["status"] = "recording"
                write_json(self.output_dir / "ground_truth_metadata.json", self.metadata)
                self.get_logger().info("First actual WorldPose received; CSV recording started")
            self.writer.writerow(row)
            self.previous[person] = stamp
            self.counts[person] += 1
        except (ValueError, OSError) as error:
            self.error = str(error)
            self.done = True

    def check_progress(self):
        now = self.get_clock().now().nanoseconds
        if self.last_clock is not None and now < self.last_clock:
            self.error = "Simulation clock reset; this experiment cannot mix time epochs"
            self.done = True
        self.last_clock = now
        if self.first_stamp is None and time.monotonic() - self.started_wall > self.timeout:
            self.error = "No ground truth received. Check /clock, controller build, GT topic, and an active server rendering sensor. No GT CSV was fabricated."
            self.done = True
        if self.first_stamp is not None and self.duration is not None:
            self.done |= now - self.first_stamp >= self.duration * NANOSECONDS
        if self.first_stamp is not None:
            for person in self.counts:
                if now - self.previous.get(person, self.first_stamp) > 2 * NANOSECONDS:
                    self.error = f"No fresh rendered observations for {person} for over 2 simulation seconds; check active camera/LiDAR rendering"
                    self.done = True

    def finish(self):
        if self.stream:
            self.stream.close()
        missing = [person for person, count in self.counts.items() if count == 0]
        if missing and not self.error:
            self.error = "No observations for: " + ", ".join(missing)
        self.metadata.update(status="failed" if self.error else "complete", counts=self.counts, error=self.error)
        write_json(self.output_dir / "ground_truth_metadata.json", self.metadata)


def main(argv=None):
    argv = sys.argv if argv is None else [sys.argv[0], *argv]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=None, help="new experiment directory; must not already exist")
    parser.add_argument("--mapping", action="append", help="person_id=actor_name; repeat for additional actors")
    parser.add_argument("--duration", type=float, help="record this many simulation seconds after the first GT sample; default: until Ctrl+C")
    parser.add_argument("--timeout", type=float, default=30, help="wall seconds to wait for first GT (default: 30)")
    parser.add_argument("--keepalive-image-topic", default="/j100_0000/sensors/camera_0/color/image",
                        help="subscribe to existing ROS image topic to keep lazy server rendering active; empty string disables")
    args = parser.parse_args(remove_ros_args(args=argv)[1:])
    try:
        mappings = parse_mappings(args.mapping or ["human_1=walking_person"])
        if not math.isfinite(args.timeout) or args.timeout <= 0:
            raise ValueError("timeout must be finite and positive")
        if args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0):
            raise ValueError("duration must be finite and positive")
        output = args.output_dir or Path("results") / (datetime.now(timezone.utc).strftime("experiment_%Y%m%dT%H%M%SZ_") + uuid.uuid4().hex[:6])
        if output.exists():
            raise ValueError("output directory already exists; choose a new experiment")
    except ValueError as error:
        parser.error(str(error))
    rclpy.init(args=argv)
    recorder = None
    result = 0
    try:
        recorder = GroundTruthRecorder(output, mappings, args.duration, args.timeout, args.keepalive_image_topic)
        while rclpy.ok() and not recorder.done:
            rclpy.spin_once(recorder, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (OSError, ValueError, RuntimeError) as error:
        print(f"Recorder error: {error}", file=sys.stderr)
        result = 1
    finally:
        if recorder:
            recorder.finish()
            print(f"GT observations: {recorder.counts}; directory: {output}")
            if recorder.error:
                print(recorder.error, file=sys.stderr)
                result = 1
            recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result


if __name__ == "__main__":
    sys.exit(main())
