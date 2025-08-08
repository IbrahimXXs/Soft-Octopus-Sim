from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QLabel, QSlider, QHBoxLayout,
    QScrollArea, QTabWidget, QCheckBox, QGridLayout
)
from PyQt5.QtCore import Qt
import mujoco
import re
from collections import defaultdict


class ActuatorControlGUI(QWidget):
    def __init__(self, model, data):
        super().__init__()
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
