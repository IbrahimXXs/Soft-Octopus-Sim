import mujoco
import mujoco_viewer
import time

# Load model
model = mujoco.MjModel.from_xml_path('Final_OctopusV2.xml')
data = mujoco.MjData(model)
ctrl = data.ctrl
viewer = mujoco_viewer.MujocoViewer(model, data)

while viewer.is_alive:
    # Set controls to zero (or constant value if you want to see a specific activation)
 
    mujoco.mj_step(model, data)
    viewer.render()
    time.sleep(0.01)  # slow down for human viewing