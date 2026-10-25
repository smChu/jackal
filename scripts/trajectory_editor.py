"""Mouse and numeric waypoint editor for the existing PyQt5 trajectory viewer."""

import math
from pathlib import Path

from PyQt5 import QtCore, QtWidgets
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from matplotlib.patches import Polygon

if __package__:
    from .trajectory_path import MAX_WAYPOINTS, build_path_trajectory, clean_points, path_length, trajectory_csv_bytes
    from .trajectory_view_frame import ViewFrame
else:
    from trajectory_path import MAX_WAYPOINTS, build_path_trajectory, clean_points, path_length, trajectory_csv_bytes
    from trajectory_view_frame import ViewFrame


class ViewControls(QtWidgets.QWidget):
    """Set the drawing/comparison view without rotating command coordinates."""
    changed = QtCore.pyqtSignal(float)

    def __init__(self, yaw=0, parent=None):
        super().__init__(parent)
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(QtWidgets.QLabel("XY view:"))
        self.presets = QtWidgets.QComboBox()
        for label, value in (("Gazebo default: +X up", 0), ("World XY: +Y up", 90),
                             ("Opposite: -X up", 180), ("Side: -Y up", -90), ("Custom yaw", None)):
            self.presets.addItem(label, value)
        self.yaw = QtWidgets.QDoubleSpinBox()
        self.yaw.setRange(-180, 180)
        self.yaw.setDecimals(1)
        self.yaw.setSuffix(" deg")
        self.yaw.setValue(yaw)
        self.yaw.setToolTip("Match the Gazebo camera yaw. 0: +X up / +Y left; 90: +Y up / +X right. No automatic camera tracking.")
        layout.addWidget(self.presets)
        layout.addWidget(self.yaw)
        layout.addStretch()
        self.presets.currentIndexChanged.connect(self.use_preset)
        self.yaw.valueChanged.connect(self.yaw_changed)
        self.yaw_changed(yaw)

    def use_preset(self):
        value = self.presets.currentData()
        if value is not None:
            self.yaw.setValue(value)

    def yaw_changed(self, value):
        match = next((i for i in range(self.presets.count()-1)
                      if math.isclose((value-self.presets.itemData(i)) % 360, 0, abs_tol=1e-9)), self.presets.count()-1)
        self.presets.blockSignals(True)
        self.presets.setCurrentIndex(match)
        self.presets.blockSignals(False)
        self.changed.emit(value)


def choose_csv_destination(parent, default_path):
    dialog = QtWidgets.QFileDialog(parent, "Save trajectory CSV", str(default_path))
    dialog.setAcceptMode(QtWidgets.QFileDialog.AcceptSave)
    dialog.setNameFilter("Trajectory CSV (*.csv)")
    dialog.setDefaultSuffix("csv")
    return Path(dialog.selectedFiles()[0]) if dialog.exec_() else None


class TrajectoryEditor(QtWidgets.QDialog):
    """Edit one actor's path. The caller preserves other actors' target samples."""

    def __init__(self, mappings, targets=None, *, draft=None, view_yaw=0, parent=None):
        super().__init__(parent)
        self.mappings = mappings
        self.targets = targets or {}
        self.draft = draft
        self.local_drafts = {}
        self.current_person = None
        self.points = []
        self.undo_stack, self.redo_stack = [], []
        self.selected = None
        self.dragging = False
        self.gesture_before = None
        self.result_samples = None
        self.result_spec = None
        self.send_requested = False
        self.view_frame = ViewFrame(view_yaw)
        self.setWindowTitle("Human trajectory — draw and edit world-XY path")
        self.resize(1160, 830)
        root = QtWidgets.QVBoxLayout(self)
        hint = QtWidgets.QLabel(
            "Draw: left-click or drag. Move: drag a waypoint. Right-click: delete the nearest point. "
            "The first point is the start; coordinates are Gazebo world metres.")
        hint.setWordWrap(True)
        root.addWidget(hint)
        self.view_controls = ViewControls(view_yaw)
        root.addWidget(self.view_controls)
        body = QtWidgets.QHBoxLayout()
        root.addLayout(body, 1)
        self.figure = Figure(figsize=(7, 6), constrained_layout=True)
        self.canvas = FigureCanvasQTAgg(self.figure)
        self.axis = self.figure.add_subplot(111)
        self.axis.set_title("Editable waypoints — Gazebo world coordinates")
        self.axis.set_aspect("equal", adjustable="box")
        self.axis.grid(True, color="0.85")
        self.world_bounds = [(-2, -2), (2, -2), (2, 2), (-2, 2)]
        self.bounds_patch = Polygon([self.view_frame.project(*p) for p in self.world_bounds],
                                    fill=False, edgecolor="0.45", linestyle="--")
        self.axis.add_patch(self.bounds_patch)
        self.compass_artists = self.view_frame.decorate(self.axis)
        self.route_line, = self.axis.plot([], [], "o-", color="#d62728", markersize=4, linewidth=2)
        self.start_marker, = self.axis.plot([], [], "*", color="#d62728", markeredgecolor="black", markersize=14)
        self.end_marker, = self.axis.plot([], [], "^", markerfacecolor="none", markeredgecolor="#d62728", markersize=9)
        self.selection_marker, = self.axis.plot([], [], "o", markerfacecolor="none", markeredgecolor="C0", markersize=12)
        body.addWidget(self.canvas, 1)
        panel = QtWidgets.QWidget()
        panel.setMaximumWidth(355)
        panel.setMinimumWidth(300)
        side = QtWidgets.QVBoxLayout(panel)
        body.addWidget(panel)
        form = QtWidgets.QFormLayout()
        self.person_box = QtWidgets.QComboBox()
        for person, actor in mappings.items():
            self.person_box.addItem(f"{person} -> {actor}", person)
        self.speed_box = QtWidgets.QDoubleSpinBox()
        self.speed_box.setRange(0.01, 10)
        self.speed_box.setDecimals(3)
        self.speed_box.setSingleStep(0.1)
        self.speed_box.setValue(0.5)
        self.speed_box.setSuffix(" m/s")
        self.rate_box = QtWidgets.QSpinBox()
        self.rate_box.setRange(1, 100)
        self.rate_box.setValue(20)
        self.rate_box.setSuffix(" Hz")
        self.mode_box = QtWidgets.QComboBox()
        self.mode_box.addItems(["Draw / append", "Move waypoints"])
        form.addRow("Person / actor", self.person_box)
        form.addRow("Walking speed", self.speed_box)
        form.addRow("Sample rate", self.rate_box)
        form.addRow("Mouse mode", self.mode_box)
        side.addLayout(form)
        self.limit_box = QtWidgets.QCheckBox("Limit x and y to [-2, 2] m")
        self.limit_box.setChecked(True)
        side.addWidget(self.limit_box)
        self.stats_label = QtWidgets.QLabel()
        self.stats_label.setWordWrap(True)
        side.addWidget(self.stats_label)
        history = QtWidgets.QHBoxLayout()
        self.undo_button = QtWidgets.QPushButton("Undo")
        self.redo_button = QtWidgets.QPushButton("Redo")
        self.clear_button = QtWidgets.QPushButton("Clear")
        for button in (self.undo_button, self.redo_button, self.clear_button):
            history.addWidget(button)
        side.addLayout(history)
        self.undo_button.setShortcut("Ctrl+Z")
        self.redo_button.setShortcut("Ctrl+Shift+Z")
        side.addWidget(QtWidgets.QLabel("Select a row; double-click X or Y to edit:"))
        self.table = QtWidgets.QTableWidget(0, 2)
        self.table.setHorizontalHeaderLabels(["World X (m)", "World Y (m)"])
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        side.addWidget(self.table, 1)
        point_buttons = QtWidgets.QHBoxLayout()
        self.delete_button = QtWidgets.QPushButton("Delete selected")
        self.fit_button = QtWidgets.QPushButton("Fit view")
        point_buttons.addWidget(self.delete_button)
        point_buttons.addWidget(self.fit_button)
        side.addLayout(point_buttons)
        self.message = QtWidgets.QLabel("No path is sent until you choose Send to Gazebo.")
        self.message.setWordWrap(True)
        root.addWidget(self.message)
        self.coordinate_label = QtWidgets.QLabel("Cursor: World X / Y (m). Saved coordinates always use the world frame.")
        root.addWidget(self.coordinate_label)
        actions = QtWidgets.QHBoxLayout()
        self.save_button = QtWidgets.QPushButton("Save CSV")
        self.use_button = QtWidgets.QPushButton("Use path / preview")
        self.send_button = QtWidgets.QPushButton("Send to Gazebo")
        self.cancel_button = QtWidgets.QPushButton("Cancel")
        for button in (self.save_button, self.use_button, self.send_button, self.cancel_button):
            button.setAutoDefault(False)
            actions.addWidget(button)
        root.addLayout(actions)
        self.person_box.currentIndexChanged.connect(self.load_person)
        self.speed_box.valueChanged.connect(self.update_stats)
        self.rate_box.valueChanged.connect(self.update_stats)
        self.limit_box.toggled.connect(self.check_bounds)
        self.table.cellChanged.connect(self.edit_cell)
        self.table.itemSelectionChanged.connect(self.select_row)
        self.undo_button.clicked.connect(self.undo)
        self.redo_button.clicked.connect(self.redo)
        self.clear_button.clicked.connect(self.clear)
        self.delete_button.clicked.connect(self.delete_selected)
        self.fit_button.clicked.connect(self.fit_view)
        self.save_button.clicked.connect(self.save_csv)
        self.use_button.clicked.connect(lambda: self.apply_path(False))
        self.send_button.clicked.connect(lambda: self.apply_path(True))
        self.cancel_button.clicked.connect(self.reject)
        self.view_controls.changed.connect(self.change_view)
        self.canvas.mpl_connect("button_press_event", self.on_press)
        self.canvas.mpl_connect("motion_notify_event", self.on_motion)
        self.canvas.mpl_connect("button_release_event", self.on_release)
        self.canvas.setFocusPolicy(QtCore.Qt.StrongFocus)
        # Programmatically selecting another person is also supported by CLI mappings.
        initial = draft.get("person_id") if draft else next(iter(self.targets), next(iter(mappings)))
        index = self.person_box.findData(initial)
        self.person_box.blockSignals(True)
        self.person_box.setCurrentIndex(max(0, index))
        self.person_box.blockSignals(False)
        self.load_person()

    def load_person(self):
        if self.current_person is not None:
            self.local_drafts[self.current_person] = {
                "waypoints":list(self.points), "speed_m_s":self.speed_box.value(),
                "sample_rate_hz":self.rate_box.value(),
            }
        person = self.person_box.currentData()
        draft = self.local_drafts.get(person)
        if draft is None and self.draft and self.draft["person_id"] == person:
            draft = self.draft
        if draft:
            self.points = clean_points(draft["waypoints"])
            self.speed_box.setValue(draft["speed_m_s"])
            self.rate_box.setValue(draft["sample_rate_hz"])
        else:
            self.points = clean_points((sample.x, sample.y) for sample in self.targets.get(person, []))
        self.undo_stack.clear()
        self.redo_stack.clear()
        self.selected = None
        self.current_person = person
        self.limit_box.blockSignals(True)
        self.limit_box.setChecked(all(self.inside(point) for point in self.points))
        self.limit_box.blockSignals(False)
        self.fit_view()
        self.refresh()

    @staticmethod
    def inside(point):
        return all(-2 <= value <= 2 for value in point)

    def check_bounds(self, checked):
        if checked and not all(self.inside(point) for point in self.points):
            self.limit_box.blockSignals(True)
            self.limit_box.setChecked(False)
            self.limit_box.blockSignals(False)
            self.message.setText("Some waypoints are outside [-2, 2] m. Move them before enabling the limit.")

    def fit_view(self):
        xs, ys = self.view_frame.series(self.world_bounds+self.points)
        span = max(max(xs)-min(xs), max(ys)-min(ys))*1.12
        cx, cy = (max(xs)+min(xs))/2, (max(ys)+min(ys))/2
        self.axis.set_xlim(cx-span/2, cx+span/2)
        self.axis.set_ylim(cy-span/2, cy+span/2)
        self.canvas.draw_idle()

    def change_view(self, yaw):
        self.view_frame = ViewFrame(yaw)
        for artist in self.compass_artists:
            artist.remove()
        self.compass_artists = self.view_frame.decorate(self.axis)
        self.bounds_patch.set_xy([self.view_frame.project(*p) for p in self.world_bounds])
        self.fit_view()
        self.refresh(table=False)

    def remember(self, before):
        if before != self.points:
            self.undo_stack.append(list(before))
            del self.undo_stack[:-100]
            self.redo_stack.clear()

    def refresh(self, table=True):
        self.route_line.set_data(*self.view_frame.series(self.points))
        self.start_marker.set_data(*self.view_frame.series(self.points[:1]))
        self.end_marker.set_data(*self.view_frame.series(self.points[-1:]))
        chosen = self.points[self.selected:self.selected+1] if self.selected is not None else []
        self.selection_marker.set_data(*self.view_frame.series(chosen))
        if table:
            self.table.blockSignals(True)
            self.table.setRowCount(len(self.points))
            for row, point in enumerate(self.points):
                for column, value in enumerate(point):
                    self.table.setItem(row, column, QtWidgets.QTableWidgetItem(f"{value:.6f}"))
            if self.selected is not None and self.selected < len(self.points):
                self.table.selectRow(self.selected)
            self.table.blockSignals(False)
        self.update_stats()
        self.canvas.draw_idle()

    def update_stats(self):
        length = path_length(self.points)
        self.stats_label.setText(
            f"Waypoints: {len(self.points)}/{MAX_WAYPOINTS}\n"
            f"Length: {length:.3f} m   Duration: {length/self.speed_box.value():.2f} s\n"
            "Yaw follows the path. Speed is constant; corners remain sharp.")
        valid = len(clean_points(self.points)) >= 2
        for button in (self.save_button, self.use_button, self.send_button):
            button.setEnabled(valid)
        self.undo_button.setEnabled(bool(self.undo_stack))
        self.redo_button.setEnabled(bool(self.redo_stack))
        self.delete_button.setEnabled(self.selected is not None)

    def nearest(self, event):
        if not self.points:
            return None
        positions = self.axis.transData.transform([self.view_frame.project(*p) for p in self.points])
        distances = [(p[0]-event.x)**2+(p[1]-event.y)**2 for p in positions]
        index = min(range(len(distances)), key=distances.__getitem__)
        return index if distances[index] <= 14**2 else None

    def event_point(self, event):
        if event.inaxes != self.axis or event.xdata is None or event.ydata is None:
            return None
        point = self.view_frame.unproject(float(event.xdata), float(event.ydata))
        if self.limit_box.isChecked() and not self.inside(point):
            self.message.setText("The drawing limit is [-2, 2] m on both axes.")
            return None
        return point

    def append_point(self, point, minimum=0.03):
        if len(self.points) >= MAX_WAYPOINTS:
            self.message.setText(f"Waypoint limit reached ({MAX_WAYPOINTS}). Delete points or undo.")
            return
        if not self.points or math.hypot(point[0]-self.points[-1][0], point[1]-self.points[-1][1]) >= minimum:
            self.points.append(point)
            self.selected = len(self.points)-1

    def on_press(self, event):
        point = self.event_point(event)
        if point is None or event.button not in (1, 3):
            return
        self.canvas.setFocus()
        if event.button == 3:
            self.selected = self.nearest(event)
            self.delete_selected()
            return
        self.gesture_before = list(self.points)
        self.dragging = True
        if self.mode_box.currentIndex() == 0:
            self.append_point(point, 1e-9)
        else:
            self.selected = self.nearest(event)
        self.refresh()

    def on_motion(self, event):
        if event.inaxes == self.axis and event.xdata is not None and event.ydata is not None:
            self.coordinate_label.setText("Cursor: "+self.view_frame.coordinate_text(event.xdata, event.ydata))
        if not self.dragging:
            return
        point = self.event_point(event)
        if point is None:
            return
        if self.mode_box.currentIndex() == 0:
            self.append_point(point)
        elif self.selected is not None:
            self.points[self.selected] = point
        self.refresh(table=False)

    def on_release(self, event):
        if not self.dragging:
            return
        point = self.event_point(event)
        if point is not None:
            if self.mode_box.currentIndex() == 0:
                self.append_point(point, 1e-9)
            elif self.selected is not None:
                self.points[self.selected] = point
        self.remember(self.gesture_before)
        self.gesture_before = None
        self.dragging = False
        self.refresh()

    def select_row(self):
        rows = self.table.selectionModel().selectedRows()
        self.selected = rows[0].row() if rows else None
        self.refresh(table=False)

    def edit_cell(self, row, column):
        try:
            value = float(self.table.item(row, column).text())
            point = list(self.points[row])
            point[column] = value
            clean_points([point])
            if self.limit_box.isChecked() and not self.inside(point):
                raise ValueError("Coordinates must stay within [-2, 2] m")
            before = list(self.points)
            self.points[row] = tuple(point)
            self.remember(before)
        except ValueError as error:
            self.message.setText(str(error))
        self.refresh()

    def delete_selected(self):
        if self.selected is not None and self.selected < len(self.points):
            before = list(self.points)
            del self.points[self.selected]
            self.selected = None
            self.remember(before)
        self.refresh()

    def clear(self):
        before = list(self.points)
        self.points = []
        self.selected = None
        self.remember(before)
        self.refresh()

    def undo(self):
        if self.undo_stack:
            self.redo_stack.append(list(self.points))
            self.points = self.undo_stack.pop()
            self.selected = None
            self.refresh()

    def redo(self):
        if self.redo_stack:
            self.undo_stack.append(list(self.points))
            self.points = self.redo_stack.pop()
            self.selected = None
            self.refresh()

    def build(self):
        return build_path_trajectory(self.points, self.person_box.currentData(),
                                     self.speed_box.value(), self.rate_box.value())

    def save_csv(self):
        try:
            data = trajectory_csv_bytes(self.build())
            destination = choose_csv_destination(self, Path.cwd()/"trajectories"/"drawn_path.csv")
            if destination:
                destination.parent.mkdir(parents=True, exist_ok=True)
                destination.write_bytes(data)
                self.message.setText(f"Saved: {destination}")
        except (ValueError, OSError) as error:
            self.message.setText(str(error))

    def apply_path(self, send):
        try:
            self.result_samples = self.build()
            self.result_spec = {
                "frame_id":"world", "time_basis":"relative_seconds",
                "person_id":self.person_box.currentData(), "actor_name":self.mappings[self.person_box.currentData()],
                "speed_m_s":self.speed_box.value(), "sample_rate_hz":self.rate_box.value(),
                "waypoints":clean_points(self.points), "bounded_4x4":self.limit_box.isChecked(),
                "interpolation":"piecewise linear at constant speed; yaw follows each segment",
                "display_view_yaw_deg":self.view_frame.yaw_deg,
            }
            self.send_requested = send
            self.accept()
        except ValueError as error:
            self.message.setText(str(error))
