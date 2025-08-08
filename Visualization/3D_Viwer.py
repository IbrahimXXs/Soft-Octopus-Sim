import json
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider
from scipy.spatial.transform import Rotation as R

# === Load JSON file ===
with open('/home/ibrahim/Documents/Octopus/Octopus Repo/output/activation_data_20250808_152633.json', 'r') as f:
    all_frames = json.load(f)

# === Extract all component names from the first frame ===
component_keys = [k for k in all_frames[0].keys() if isinstance(all_frames[0][k], dict) and 'pos' in all_frames[0][k]]

# === Create the figure and axis ===
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')
plt.subplots_adjust(bottom=0.2)  # Leave space for slider

# === Plot elements ===
points = ax.scatter([], [], [], c='blue', s=60)
lines, = ax.plot([], [], [], c='gray', linewidth=2)
annotations = []

# === Slider axis ===
ax_slider = plt.axes([0.2, 0.05, 0.6, 0.03])
slider = Slider(ax_slider, 'Frame', 0, len(all_frames) - 1, valinit=0, valstep=1)

# === Update function ===
def update(frame_idx):
    frame = all_frames[int(frame_idx)]

    positions = []
    euler_angles = []

    # Clear old annotations
    for ann in annotations:
        ann.remove()
    annotations.clear()

    # Extract position and quaternion -> convert to Euler
    for key in component_keys:
        pos = frame[key]['pos']
        quat = frame[key]['quat']
        positions.append(pos)

        # Convert quaternion to Euler angles (in degrees)
        r = R.from_quat([quat[1], quat[2], quat[3], quat[0]])  # Note: [x, y, z, w]
        euler = r.as_euler('xyz', degrees=True)
        euler_angles.append(euler)

    positions = np.array(positions)
    xs, ys, zs = positions[:, 0], positions[:, 1], positions[:, 2]

    # Update scatter
    points._offsets3d = (xs, ys, zs)

    # Connect segments
    segment_labels = [k for k in component_keys if k.startswith("segment")]
    segment_labels.sort()
    segment_coords = [frame[k]['pos'] for k in segment_labels]
    segment_coords = np.array(segment_coords)
    if len(segment_coords) > 0:
        lines.set_data(segment_coords[:, 0], segment_coords[:, 1])
        lines.set_3d_properties(segment_coords[:, 2])
    else:
        lines.set_data([], [])
        lines.set_3d_properties([])

    # Annotate each point
    for i, key in enumerate(component_keys):
        pos = positions[i]
        euler = euler_angles[i]
        label = f"{key}\n({euler[0]:.0f}°, {euler[1]:.0f}°, {euler[2]:.0f}°)"
        ann = ax.text(pos[0], pos[1], pos[2], label, fontsize=8)
        annotations.append(ann)

    # Update title with timestamp
    timestamp = frame.get("timestamp", 0)
    ax.set_title(f"Frame {int(frame_idx)} | Time: {timestamp:.3f}s", fontsize=12)

    # Update axis limits dynamically (optional)
    ax.set_xlim([-0.1, 0.1])
    ax.set_ylim([-0.1, 0.1])
    ax.set_zlim([0.8, 1.05])

    fig.canvas.draw_idle()

# === Connect the slider to update function ===
slider.on_changed(update)

# === Initial plot ===
update(0)
plt.show()
