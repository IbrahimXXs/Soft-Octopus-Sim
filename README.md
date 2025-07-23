# 🐙 Soft-Octopus-Sim

**Soft-Octopus-Sim** is a soft robotics simulation built in MuJoCo, modeling a biologically inspired octopus with **8 flexible limbs**, each composed of **27 segments**. The robot is actuated via muscle-like tendons, and supports both visualization and randomized control behaviors.

---

## 🚀 Project Overview

This project simulates a high-DOF soft robotic octopus using MuJoCo, exploring:
- Soft-bodied motion and control
- Periodic and random actuation of muscle actuators
- Flexible limb kinematics and biologically inspired morphology

It is intended for researchers, hobbyists, or developers working on soft robotics, biologically inspired systems, or MuJoCo-based simulations.

---

## 📁 Repository Structure

Soft-Octopus-Sim/
│
├── Final_OctV2_Viewer.py # Viewer script to visualize the octopus
├── Random_Actuation.py # Periodic random actuation of tendons
│
├── Generator/ # Auto-build scripts for model generation/modification
│ └── (Python/XML/etc.)
│
├── 3D_OctV2/ # STL mesh files for 3D model geometry
│ └── (Octopus limb and body components)
│
├── Final_OctopusV2.xml # Main MuJoCo model file
├── README.md # This file
└── .gitignore # Git exclusions



---

## 🧰 Requirements

- Python 3.8+
- [MuJoCo](https://mujoco.org/)
- `mujoco` Python bindings
- `mujoco_viewer` (`pip install mujoco` installs viewer utilities as of MuJoCo 2.3+)
- `numpy`

To install dependencies:
```bash
pip install mujoco numpy


## 🖥️ Usage

### 🔹 1. View the Octopus Model

To launch the MuJoCo viewer with the soft octopus model:

```bash
python Final_OctV2_Viewer.py
```

This opens an interactive viewer window showing the octopus robot in its default state. Use mouse controls to navigate the 3D environment and observe the model structure.

### 🔹 2. Run Random Periodic Actuation

To randomly activate muscle tendons (simulating twitching or control attempts):

```bash
python Random_Actuation.py
```

This script performs the following actions:
- Randomly selects actuators from the muscle tendon system
- Sets full activation (`1.0`) for `0.2 seconds`
- Repeats the pattern with short pauses between activations
- Demonstrates emergent movement patterns through random stimulation

## 🧪 Model Description

| Component | Details |
|-----------|---------|
| **Limbs** | 8 symmetrically arranged |
| **Segments per Limb** | 27 articulated segments |
| **Total Muscle Actuators** | ~1872 (auto-counted via model) |
| **Control Mechanism** | Muscle tendons with continuous actuation signals |
| **Physics Engine** | MuJoCo with soft-body dynamics |

The simulation uses tendon-based control to replicate soft-bodied movement similar to that of a real octopus. Each limb can bend, twist, and contract independently, allowing for complex locomotive behaviors.

## 🔧 Generator Scripts

Located in the `Generator/` directory, these scripts are designed to:

- **Automatically generate** or modify XML model definitions
- **Programmatically build** tendons, segments, and body structure  
- **Scale and customize** the octopus design parameters
- **Useful for procedural scaling** or design changes without manual XML editing

## 🧱 3D Assets

The `3D_OctV2/` folder contains `.stl` mesh files used for rendering the octopus limbs and body parts in MuJoCo. These assets provide:

- High-quality visual representation
- Accurate collision geometry
- Optimized mesh topology for real-time rendering

## 🎯 Applications

This simulation framework can be used for:

- **Soft robotics research** and development
- **Bio-inspired locomotion** studies
- **Control algorithm** testing and validation
- **Educational demonstrations** of soft-body physics
- **Reinforcement learning** environments for robotic control

## 📌 Future Improvements

- [ ] Integration with reinforcement learning frameworks (e.g., stable-baselines3)
- [ ] More biologically accurate muscle modeling
- [ ] Sensing and feedback mechanisms (proprioception, touch)
- [ ] Gait learning and control optimization algorithms
- [ ] Underwater dynamics and fluid interaction
- [ ] Multi-octopus swarm simulation capabilities

## 🤝 Contributing

Contributions are welcome! Please feel free to submit pull requests, report bugs, or suggest new features.

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📄 License

This project is open-source under the MIT License, unless otherwise specified. See `LICENSE` file for details.

## 📚 References

- MuJoCo Physics Engine Documentation
- Biological studies on octopus locomotion and muscle structure
- Soft robotics control methodologies
