import mujoco
import mujoco_viewer
import time
import numpy as np
import random

# Load model
model = mujoco.MjModel.from_xml_path('Final_OctopusV2.xml')
data = mujoco.MjData(model)
viewer = mujoco_viewer.MujocoViewer(model, data)

total_actuators = model.nu  # model.nu is the number of control inputs
# Number of actuators to activate at the same time
num_active = 100  # Change this to whatever number you want

dt = model.opt.timestep
activation_duration = 0.2
activation_steps = int(activation_duration / dt)

while viewer.is_alive:

    active_indices = random.sample(range(total_actuators), num_active)
    
    data.ctrl[:] = 0.0  # Reset all
    for idx in active_indices:
        data.ctrl[idx] = 1.0

    for _ in range(activation_steps):
        mujoco.mj_step(model, data)
        viewer.render()
        time.sleep(dt)

    data.ctrl[:] = 0.0
    for _ in range(10):  # Let the viewer catch up and let the system rest briefly
        mujoco.mj_step(model, data)
        viewer.render()
        time.sleep(dt)
