import mujoco
import mujoco_viewer
import time
import numpy as np

# Load model
model = mujoco.MjModel.from_xml_path('Final_OctopusV2_Submerged.xml')
data = mujoco.MjData(model)
ctrl = data.ctrl
viewer = mujoco_viewer.MujocoViewer(model, data)

# Parameters for dynamic current
CURRENT_AMPLITUDE = 0.02   # max force magnitude (N)
CURRENT_FREQUENCY = 5.5    # Hz — how fast it changes
CURRENT_DIRECTION = np.array([1.0, 0.2, 0.0])  # vector direction of the current

# Normalize the current direction
CURRENT_DIRECTION = CURRENT_DIRECTION / np.linalg.norm(CURRENT_DIRECTION)

while viewer.is_alive:
    data.ctrl[:] = 0
    data.xfrc_applied[:] = 0  # Clear previous external forces

    # Compute time-varying current magnitude (sinusoidal)
    current_magnitude = CURRENT_AMPLITUDE * np.sin(2 * np.pi * CURRENT_FREQUENCY * data.time)
    current_force = current_magnitude * CURRENT_DIRECTION

    for i in range(1, model.nbody):
        data.xfrc_applied[i, 0:3] = current_force  # apply current as external force

    mujoco.mj_step(model, data)
    viewer.render()
    time.sleep(0.01)
