import mujoco
import mujoco_viewer
import time

# Enable or disable the GUI monitor
USE_MONITOR = False                       # ON or OFF for Data Logging

if USE_MONITOR:
    from State_Info.State_Info import OctopusMonitor

# Load model and data
model = mujoco.MjModel.from_xml_path('Final_OctopusV2.xml')
data = mujoco.MjData(model)
viewer = mujoco_viewer.MujocoViewer(model, data)

# Only create monitor if enabled
monitor = OctopusMonitor(model, data) if USE_MONITOR else None

while viewer.is_alive:
    data.ctrl[:] = 0

    mujoco.mj_step(model, data)
    viewer.render()

    if monitor:
        monitor.update()

    time.sleep(0.01)
