import mujoco
import mujoco_viewer
import time
import numpy as np
import random

# === Optional GUI Monitor Toggle ===
USE_MONITOR = False  # <<< Turn GUI monitor ON or OFF here

if USE_MONITOR:
    from State_Info.State_Info import OctopusMonitor

# === Load Model and Data ===
model = mujoco.MjModel.from_xml_path('Final_OctopusV2.xml')
data = mujoco.MjData(model)
viewer = mujoco_viewer.MujocoViewer(model, data)

# === Optional GUI Monitor Initialization ===
monitor = OctopusMonitor(model, data) if USE_MONITOR else None

# === Control Settings ===
total_actuators = model.nu
num_active = 100  # Number of actuators to activate at once

dt = model.opt.timestep
activation_duration = 0.2
activation_steps = int(activation_duration / dt)

# === Simulation Loop ===
while viewer.is_alive:

    # Randomly activate some actuators
    active_indices = random.sample(range(total_actuators), num_active)
    data.ctrl[:] = 0.0
    for idx in active_indices:
        data.ctrl[idx] = 1.0

    # Step through the activation duration
    for _ in range(activation_steps):
        mujoco.mj_step(model, data)
        viewer.render()
        if monitor:
            monitor.update()
        time.sleep(dt)

    # Deactivate all and rest briefly
    data.ctrl[:] = 0.0
    for _ in range(10):
        mujoco.mj_step(model, data)
        viewer.render()
        if monitor:
            monitor.update()
        time.sleep(dt)
