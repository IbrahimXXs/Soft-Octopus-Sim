import mujoco
import mujoco_viewer
import time

# Load model and data
model = mujoco.MjModel.from_xml_path('/home/ibrahim/Documents/Octopus/Octopus Repo/Final_OctopusV2_Submerged.xml')
data = mujoco.MjData(model)
viewer = mujoco_viewer.MujocoViewer(model, data)

# Dynamically generate actuator names
def generate_group1_names():
    names = []
    for r in range(1, 9):  # R1 to R8
        names.append(f"A_B1S1_5_R{r}")
        for s in range(1, 8):  # S1S2 to S7S8
            names.append(f"A_S{s}S{s+1}_5_R{r}")
    return names

def generate_group2_names():
    names = []
    for r in range(1, 9):  # R1 to R8
        names.extend([f"A_B1S1_1_R{r}", f"A_B1S1_9_R{r}"])
        for s in range(1, 20):  # S1S2 to S19S20
            names.extend([f"A_S{s}S{s+1}_1_R{r}", f"A_S{s}S{s+1}_9_R{r}"])
    return names

def print_sim_status():
    sim_time = data.time
    head_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "head")
    head_pos = data.xpos[head_id]
    print(f"Head Pos: x={head_pos[0]:.3f}, y={head_pos[1]:.3f}, z={head_pos[2]:.3f}")


group1_names = generate_group1_names()
group2_names = generate_group2_names()

# Resolve actuator IDs
def get_ids(names):
    ids = []
    for name in names:
        try:
            aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            ids.append(aid)
        except:
            print(f"Warning: Actuator '{name}' not found in the model.")
    return ids

group1_ids = get_ids(group1_names)
group2_ids = get_ids(group2_names)

# Helper to linearly ramp activation
def ramp_actuators(act_ids, start_val, end_val, duration, timestep=0.01):
    steps = int(duration / timestep)
    for step in range(steps):
        progress = step / steps
        value = start_val + (end_val - start_val) * progress
        for aid in act_ids:
            data.ctrl[aid] = value
        mujoco.mj_step(model, data)
        print_sim_status()
        viewer.render()
        time.sleep(timestep)

# Helper to set activation and hold
def hold_actuators(act_ids, value, duration, timestep=0.01):
    for _ in range(int(duration / timestep)):
        for aid in act_ids:
            data.ctrl[aid] = value
        mujoco.mj_step(model, data)
        print_sim_status()
        viewer.render()
        time.sleep(timestep)

# Main control loop
while viewer.is_alive:
    # Phase 1: Ramp group1 from 0 to 1 in 10s
    ramp_actuators(group1_ids, 0.0, 1.0, 10.0)

    # Phase 2: Hold group1 at 1 for 1s
    hold_actuators(group1_ids, 1.0, 1.0)

    # Phase 3: Release all to 0
    hold_actuators(group1_ids + group2_ids, 0.0, 0.1)

    # Phase 4: Ramp group2 from 0 to 1 in 0.1s
    ramp_actuators(group2_ids, 0.0, 1.0, 0.1)

    # Phase 5: Hold group2 at 1 for 5s
    hold_actuators(group2_ids, 1.0, 1.0)

    # Phase 6: Release group2 to 0
    hold_actuators(group2_ids, 0.0, 0.1)

    # Phase 7: Wait 5s
    hold_actuators([], 0.0, 5.0)
