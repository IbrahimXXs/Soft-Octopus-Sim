import mujoco
import time
import os
import json
import datetime
from openpyxl import Workbook
from PyQt5.QtWidgets import QApplication, QWidget, QTabWidget, QVBoxLayout, QTableWidget, QTableWidgetItem
from PyQt5.QtWidgets import QTextEdit, QPushButton
import sys

class OctopusMonitor:
    def __init__(self, model, data):
        self.model = model
        self.data = data
        self._init_robot_actuators()

        self.app = QApplication.instance() or QApplication(sys.argv)
        self.gui = self.ActivationGUI(self)
        self.gui.show()

        self.step_count = 0
        self.enabled = True

    def _init_robot_actuators(self):
        self.robot_actuators = {f"Robot {i+1}": [] for i in range(8)}
        for i in range(self.model.nu):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            for robot_name in self.robot_actuators:
                if f"_R{robot_name[-1]}" in name:
                    self.robot_actuators[robot_name].append((i, name))
                    break

    def update(self):
        if not self.enabled or not self.gui.isVisible():
            return
        if self.step_count % 10 == 0:
            self.gui.update_activations()
            self.app.processEvents()
        self.step_count += 1

    def close(self):
        self.gui.close()

    class ActivationGUI(QWidget):
        def __init__(self, monitor):
            super().__init__()
            self.monitor = monitor
            self.setWindowTitle("Octopus Robot State Viewer")
            self.resize(1000, 700)

            self.activation_history = []
            self.state_history = []
            self.tables = {}
            self.state_boxes = {}
            self.start_time = time.time()

            layout = QVBoxLayout()
            self.main_tabs = QTabWidget()

            for robot, actuators in self.monitor.robot_actuators.items():
                robot_tab = QWidget()
                robot_layout = QVBoxLayout()
                sub_tabs = QTabWidget()

                table = QTableWidget()
                table.setRowCount(len(actuators))
                table.setColumnCount(2)
                table.setHorizontalHeaderLabels(["Actuator Name", "Activation"])
                for row, (idx, name) in enumerate(actuators):
                    table.setItem(row, 0, QTableWidgetItem(name))
                    table.setItem(row, 1, QTableWidgetItem("0.0000"))

                self.tables[robot] = table
                sub_tabs.addTab(table, "Activation")

                state_box = QTextEdit()
                state_box.setReadOnly(True)
                self.state_boxes[robot] = state_box
                sub_tabs.addTab(state_box, "State Info")

                robot_layout.addWidget(sub_tabs)
                robot_tab.setLayout(robot_layout)
                self.main_tabs.addTab(robot_tab, robot)

            self.global_state_text = QTextEdit()
            self.global_state_text.setReadOnly(True)
            self.main_tabs.addTab(self.global_state_text, "Head & Nest")

            layout.addWidget(self.main_tabs)

            self.save_button = QPushButton("Save Data")
            self.save_button.clicked.connect(self.save_data)
            layout.addWidget(self.save_button)

            self.setLayout(layout)

        def update_activations(self):
            model = self.monitor.model
            data = self.monitor.data
            timestamp = round(time.time() - self.start_time, 3)
            activation_snapshot = {"timestamp": timestamp}

            for robot, actuators in self.monitor.robot_actuators.items():
                table = self.tables[robot]
                for row, (idx, name) in enumerate(actuators):
                    activation = data.ctrl[idx]
                    table.item(row, 1).setText(f"{activation:.4f}")
                    activation_snapshot[name] = activation
            self.activation_history.append(activation_snapshot)

            self.global_state_text.clear()
            state_snapshot = {"timestamp": timestamp}
            for body_name in ['head', 'nest']:
                self.global_state_text.append(self._get_body_state_text(body_name))
                pos, quat = self._get_body_state(body_name)
                state_snapshot[body_name] = {"pos": pos.tolist(), "quat": quat.tolist()}

            for robot_index in range(1, 9):
                robot_key = f"Robot {robot_index}"
                box = self.state_boxes[robot_key]
                box.clear()
                for seg_num in range(1, 27):
                    body_name = f"segment_{seg_num}_R{robot_index}"
                    try:
                        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
                        box.append(self._get_body_state_text(body_name))
                        pos, quat = self._get_body_state(body_name)
                        state_snapshot[body_name] = {"pos": pos.tolist(), "quat": quat.tolist()}
                    except mujoco.MujocoException:
                        continue

            self.state_history.append(state_snapshot)

        def _get_body_state_text(self, body_name):
            model = self.monitor.model
            data = self.monitor.data
            try:
                body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
                pos = data.xpos[body_id]
                quat = data.xquat[body_id]
                return f"{body_name}:\n  Pos = {pos.round(4)}\n  Ori (quat) = {quat.round(4)}\n"
            except Exception:
                return f"{body_name}: not found\n"

        def _get_body_state(self, body_name):
            model = self.monitor.model
            data = self.monitor.data
            body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
            pos = data.xpos[body_id].copy()
            quat = data.xquat[body_id].copy()
            return pos, quat

        def save_data(self):
            now = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            os.makedirs("output", exist_ok=True)

            SAVE_AS_JSON = True
            SAVE_AS_EXCEL = True

            if SAVE_AS_JSON:
                with open(f"output/activation_data_{now}.json", "w") as f:
                    json.dump(self.activation_history, f, indent=2)
                with open(f"output/state_data_{now}.json", "w") as f:
                    json.dump(self.state_history, f, indent=2)

            if SAVE_AS_EXCEL:
                wb = Workbook()
                ws = wb.active
                ws.title = "Activations"
                if self.activation_history:
                    headers = ["timestamp"] + [k for k in self.activation_history[0] if k != "timestamp"]
                    ws.append(headers)
                    for entry in self.activation_history:
                        row = [entry["timestamp"]] + [entry.get(k, 0) for k in headers[1:]]
                        ws.append(row)

                ws2 = wb.create_sheet("States")
                if self.state_history:
                    headers = ["timestamp"]
                    first_entry = self.state_history[0]
                    for body, v in first_entry.items():
                        if body == "timestamp":
                            continue
                        headers += [f"{body}_px", f"{body}_py", f"{body}_pz",
                                    f"{body}_qx", f"{body}_qy", f"{body}_qz", f"{body}_qw"]
                    ws2.append(headers)

                    for entry in self.state_history:
                        row = [entry["timestamp"]]
                        for body in first_entry:
                            if body == "timestamp":
                                continue
                            val = entry.get(body, {"pos": [0,0,0], "quat": [1,0,0,0]})
                            row += val["pos"] + val["quat"]
                        ws2.append(row)

                wb.save(f"output/robot_data_{now}.xlsx")

            print(f"Saved data to output folder.")
