import mujoco
import mujoco_viewer
import time
import sys
from PyQt5.QtWidgets import QApplication

# === Toggle options ===
USE_MONITOR = False         # For data logging
USE_SLIDER_GUI = False      # For actuator control GUI

# === Optional imports ===
if USE_MONITOR:
    from State_Info.State_Info import OctopusMonitor

if USE_SLIDER_GUI:
    from Control.ActuatorGUI import ActuatorControlGUI

# === MuJoCo model setup ===
model = mujoco.MjModel.from_xml_path('Final_OctopusV2.xml')
data = mujoco.MjData(model)
viewer = mujoco_viewer.MujocoViewer(model, data)

# === Optional GUI setups ===
monitor = OctopusMonitor(model, data) if USE_MONITOR else None

if USE_SLIDER_GUI:
    app = QApplication.instance() or QApplication(sys.argv)
    actuator_gui = ActuatorControlGUI(model, data)
    actuator_gui.show()
else:
    app = None
    actuator_gui = None

# === Main simulation loop ===
while viewer.is_alive:
    # Commented out so slider input works:
    # data.ctrl[:] = 0

    mujoco.mj_step(model, data)
    viewer.render()

    if monitor:
        monitor.update()

    if actuator_gui:
        app.processEvents()

    time.sleep(0.01)
