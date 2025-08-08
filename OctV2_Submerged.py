import mujoco
import mujoco_viewer
import time
import numpy as np

# === Optional GUI Monitor Toggle ===
USE_MONITOR = False # <<< Toggle ON or OFF here

if USE_MONITOR:
    from State_Info.State_Info import OctopusMonitor

# === Load model ===
model = mujoco.MjModel.from_xml_path('Final_OctopusV2_Submerged.xml')
data = mujoco.MjData(model)
viewer = mujoco_viewer.MujocoViewer(model, data)

# === Optional GUI Monitor Initialization ===
monitor = OctopusMonitor(model, data) if USE_MONITOR else None

# === Dynamic current parameters ===
CURRENT_AMPLITUDE = 0.02     # max force magnitude (N)
CURRENT_FREQUENCY = 5.5      # Hz — how fast it changes
CURRENT_DIRECTION = np.array([1.0, 0.2, 0.0])
CURRENT_DIRECTION /= np.linalg.norm(CURRENT_DIRECTION)  # normalize direction

# === Simulation Loop ===
while viewer.is_alive:
    data.ctrl[:] = 0
    data.xfrc_applied[:] = 0  # Clear all external forces

    # Compute sinusoidal water current
    current_magnitude = CURRENT_AMPLITUDE * np.sin(2 * np.pi * CURRENT_FREQUENCY * data.time)
    current_force = current_magnitude * CURRENT_DIRECTION

    # Apply current to all bodies (except the world)
    for i in range(1, model.nbody):
        data.xfrc_applied[i, 0:3] = current_force  # Apply force only, no torque

    mujoco.mj_step(model, data)
    viewer.render()

    if monitor:
        monitor.update()

    time.sleep(0.01)
