import mujoco
import mujoco_viewer
import time
import numpy as np
import sys
from PyQt5.QtWidgets import QApplication

# === GUI Toggles ===
USE_MONITOR = False       # Toggle monitor ON/OFF
USE_SLIDER_GUI = False     # Toggle slider GUI ON/OFF

# === Optional GUI Imports ===
if USE_MONITOR:
    from State_Info.State_Info import OctopusMonitor

if USE_SLIDER_GUI:
    from Control.ActuatorGUI import ActuatorControlGUI

# === Load model ===
model = mujoco.MjModel.from_xml_path('Final_OctopusV2_Submerged.xml')
data = mujoco.MjData(model)
viewer = mujoco_viewer.MujocoViewer(model, data)

# === Optional GUI Instantiation ===
monitor = OctopusMonitor(model, data) if USE_MONITOR else None

if USE_SLIDER_GUI:
    app = QApplication.instance() or QApplication(sys.argv)
    actuator_gui = ActuatorControlGUI(model, data)
    actuator_gui.show()
else:
    app = None
    actuator_gui = None

# === Dynamic current parameters ===
CURRENT_AMPLITUDE = 0.02      # Max force (N)
CURRENT_FREQUENCY = 5.5       # Frequency (Hz)
CURRENT_DIRECTION = np.array([1.0, 0.2, 0.0])
CURRENT_DIRECTION /= np.linalg.norm(CURRENT_DIRECTION)  # Normalize

# === Simulation Loop ===
while viewer.is_alive:
    data.xfrc_applied[:] = 0  # Clear external forces

    # Compute sinusoidal water current
    current_magnitude = CURRENT_AMPLITUDE * np.sin(2 * np.pi * CURRENT_FREQUENCY * data.time)
    current_force = current_magnitude * CURRENT_DIRECTION

    # Apply force to all bodies (except world)
    for i in range(1, model.nbody):
        data.xfrc_applied[i, 0:3] = current_force

    mujoco.mj_step(model, data)
    viewer.render()

    if monitor:
        monitor.update()

    if actuator_gui:
        app.processEvents()

    time.sleep(0.01)
