from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QLabel, QSlider, QHBoxLayout,
    QScrollArea, QTabWidget, QCheckBox, QGridLayout
)
from PyQt5.QtCore import Qt
import mujoco
import re
from collections import defaultdict

from PyQt5.QtWidgets import QPushButton, QLineEdit
from PyQt5.QtCore import QTimer, QTime
import time


class ActuatorControlGUI(QWidget):
    def __init__(self, model, data):
        super().__init__()
        self.recording = []  # List of (timestamp, {actuator_idx: value})
        self.is_recording = False
        self.is_looping = False
        self.playback_timer = QTimer()
        self.loop_wait_timer = QTimer()
        self.playback_start_time = None
        self.loop_delay_seconds = 1.0

        self.model = model
        self.data = data
        self.setWindowTitle("Octopus Actuator Controller")
        self.resize(1000, 800)

        self.layout = QVBoxLayout()
        self.tabs = QTabWidget()
        self.layout.addWidget(self.tabs)
        self.setLayout(self.layout)

        self.robot_actuators = self._group_actuators()
        self.sliders = {}  # Individual sliders
        self.group_sliders = {}  # Group control sliders
        self.full_control_groups = {}  # For full control checkboxes
        self._create_tabs()

    def _group_actuators(self):
        grouped = {f"R{i}": [] for i in range(1, 9)}
        for i in range(self.model.nu):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            for key in grouped:
                if f"_{key}" in name:
                    grouped[key].append((i, name))
                    break
        return grouped

    def _create_tabs(self):
        for robot_key, actuators in self.robot_actuators.items():
            robot_tab = QWidget()
            robot_layout = QVBoxLayout()
            sub_tabs = QTabWidget()

            # --- Manual Control Tab ---
            manual_widget = QWidget()
            manual_layout = QVBoxLayout()
            for idx, name in actuators:
                row = QHBoxLayout()
                label = QLabel(name)
                slider = QSlider(Qt.Horizontal)
                slider.setMinimum(0)
                slider.setMaximum(1000)
                slider.setValue(int(self.data.ctrl[idx] * 1000))
                slider.setSingleStep(1)

                def make_callback(act_idx):
                    return lambda val: self._slider_changed(act_idx, val)

                slider.valueChanged.connect(make_callback(idx))
                row.addWidget(label)
                row.addWidget(slider)

                manual_layout.addLayout(row)
                self.sliders[idx] = slider
            manual_widget.setLayout(manual_layout)
            scroll_manual = QScrollArea()
            scroll_manual.setWidget(manual_widget)
            scroll_manual.setWidgetResizable(True)
            sub_tabs.addTab(scroll_manual, "Manual")

            # --- Group Control Tab ---
            group_widget = QWidget()
            group_layout = QVBoxLayout()
            group_map = defaultdict(list)

            for idx, name in actuators:
                match = re.search(r'_([0-9]+)_'+robot_key+'$', name)
                if match:
                    group_id = match.group(1)
                    group_map[group_id].append(idx)

            for group_id, indices in group_map.items():
                row = QHBoxLayout()
                label = QLabel(f"Group {group_id}")
                slider = QSlider(Qt.Horizontal)
                slider.setMinimum(0)
                slider.setMaximum(1000)
                slider.setValue(0)
                slider.setSingleStep(1)

                def make_group_callback(group_indices):
                    return lambda val: self._group_slider_changed(group_indices, val)

                slider.valueChanged.connect(make_group_callback(indices))
                row.addWidget(label)
                row.addWidget(slider)

                group_layout.addLayout(row)
                self.group_sliders[(robot_key, group_id)] = slider

            group_widget.setLayout(group_layout)
            scroll_group = QScrollArea()
            scroll_group.setWidget(group_widget)
            scroll_group.setWidgetResizable(True)
            sub_tabs.addTab(scroll_group, "Group Control")

            # Combine into Robot tab
            robot_layout.addWidget(sub_tabs)
            robot_tab.setLayout(robot_layout)
            self.tabs.addTab(robot_tab, f"Robot {robot_key}")

        # === Full Control Tab (ALL robots & groups) ===
        full_tab = QWidget()
        full_layout = QVBoxLayout()

        grid = QGridLayout()
        row = 0

        # Create checkboxes for each Robot R# and Group #
        for robot_key, actuators in self.robot_actuators.items():
            group_map = defaultdict(list)
            for idx, name in actuators:
                match = re.search(r'_([0-9]+)_'+robot_key+'$', name)
                if match:
                    group_id = match.group(1)
                    group_map[group_id].append(idx)

            for group_id, indices in group_map.items():
                checkbox = QCheckBox(f"{robot_key} - Group {group_id}")
                self.full_control_groups[checkbox] = indices
                grid.addWidget(checkbox, row // 2, row % 2)
                row += 1

        full_layout.addLayout(grid)

        # Master control slider
        slider_row = QHBoxLayout()
        slider_label = QLabel("Set Activation for Selected Groups")
        self.full_control_slider = QSlider(Qt.Horizontal)
        self.full_control_slider.setMinimum(0)
        self.full_control_slider.setMaximum(1000)
        self.full_control_slider.setValue(0)
        self.full_control_slider.setSingleStep(1)
        self.full_control_slider.valueChanged.connect(self._apply_full_control)

        slider_row.addWidget(slider_label)
        slider_row.addWidget(self.full_control_slider)
        full_layout.addLayout(slider_row)

        # === Recording and Playback Controls ===
        button_row = QHBoxLayout()

        self.record_button = QPushButton("Record")
        self.stop_button = QPushButton("Stop Recording")
        self.play_button = QPushButton("Play Recorded")
        self.loop_button = QPushButton("Loop: OFF")
        self.loop_wait_input = QLineEdit("1.0")
        self.loop_wait_input.setFixedWidth(50)

        self.record_button.clicked.connect(self.start_recording)
        self.stop_button.clicked.connect(self.stop_recording)
        self.play_button.clicked.connect(self.play_recording)
        self.loop_button.clicked.connect(self.toggle_loop)

        button_row.addWidget(self.record_button)
        button_row.addWidget(self.stop_button)
        button_row.addWidget(self.play_button)
        button_row.addWidget(self.loop_button)
        button_row.addWidget(QLabel("Loop Wait (s):"))
        button_row.addWidget(self.loop_wait_input)

        full_layout.addLayout(button_row)


        full_tab.setLayout(full_layout)
        scroll_full = QScrollArea()
        scroll_full.setWidget(full_tab)
        scroll_full.setWidgetResizable(True)

        self.tabs.addTab(scroll_full, "Full Control")

    def _slider_changed(self, idx, value):
        self.data.ctrl[idx] = value / 1000.0

    def _group_slider_changed(self, indices, value):
        val = value / 1000.0
        for idx in indices:
            self.data.ctrl[idx] = val
            if idx in self.sliders:
                self.sliders[idx].blockSignals(True)
                self.sliders[idx].setValue(value)
                self.sliders[idx].blockSignals(False)

    def _apply_full_control(self, value):
        val = value / 1000.0
        for checkbox, indices in self.full_control_groups.items():
            if checkbox.isChecked():
                for idx in indices:
                    self.data.ctrl[idx] = val
                    if idx in self.sliders:
                        self.sliders[idx].blockSignals(True)
                        self.sliders[idx].setValue(value)
                        self.sliders[idx].blockSignals(False)

    def update_sliders(self):
        for idx, slider in self.sliders.items():
            slider.blockSignals(True)
            slider.setValue(int(self.data.ctrl[idx] * 1000))
            slider.blockSignals(False)


    def start_recording(self):
        self.recording = []
        self.is_recording = True
        self.record_start_time = time.time()
        self._record_snapshot()
        self.record_timer = QTimer()
        self.record_timer.timeout.connect(self._record_snapshot)
        self.record_timer.start(50)  # Record every 50ms

    def _record_snapshot(self):
        if not self.is_recording:
            return
        timestamp = time.time() - self.record_start_time
        snapshot = {idx: self.data.ctrl[idx] for idx in range(self.model.nu)}
        self.recording.append((timestamp, snapshot))

    def stop_recording(self):
        if self.is_recording:
            self.record_timer.stop()
            self.is_recording = False
            print(f"Recorded {len(self.recording)} snapshots")

    def play_recording(self):
        if not self.recording:
            print("No recording to play.")
            return

        self.playback_start_time = time.time()
        self.playback_index = 0

        # Disconnect existing connections to prevent stacking them
        try:
            self.playback_timer.timeout.disconnect()
        except TypeError:
            pass  # No existing connection, safe to proceed

        self.playback_timer.timeout.connect(self._playback_step)
        self.playback_timer.start(10)


    def _playback_step(self):
        if self.playback_index >= len(self.recording):
            self.playback_timer.stop()
            if self.is_looping:
                try:
                    self.loop_delay_seconds = float(self.loop_wait_input.text())
                except ValueError:
                    self.loop_delay_seconds = 1.0
                self.loop_wait_timer.singleShot(int(self.loop_delay_seconds * 1000), self.play_recording)
            return

        now = time.time() - self.playback_start_time
        while (self.playback_index < len(self.recording) and 
            self.recording[self.playback_index][0] <= now):
            _, snapshot = self.recording[self.playback_index]
            for idx, val in snapshot.items():
                self.data.ctrl[idx] = val
                if idx in self.sliders:
                    slider_value = int(val * 1000)
                    self.sliders[idx].blockSignals(True)
                    self.sliders[idx].setValue(slider_value)
                    self.sliders[idx].blockSignals(False)
            self.playback_index += 1

    def toggle_loop(self):
        self.is_looping = not self.is_looping
        self.loop_button.setText(f"Loop: {'ON' if self.is_looping else 'OFF'}")
