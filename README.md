# 🤖 fbot_learning — My Solutions

> Personal solutions repository for the [fbot_learning](https://github.com/fbotathome/fbot_learning) challenge series, developed as part of the onboarding process for the **FBOT robotics team**.



## 📖 Project Description

This repository contains my personal solutions to the **fbot_learning** challenge series, a structured learning path designed to introduce members of the FBOT robotics team to the core tools and technologies used in robot software development.

The challenges cover progressively advanced topics, starting from basic ROS 2 node creation and robot motion control, moving on to autonomous navigation with obstacle avoidance, and culminating in behavior orchestration using finite state machines with the **YASMIN** library.


---

## ⚙️ System Behavior

Each challenge represents a self-contained ROS 2 pkg and node that interacts with a simulator. The general behavior pattern across all challenges follows a **Sense → Think → Act** loop:

**Challenge 1 — Turtle Challenge 🐢**
The node publishes `Twist` messages to the `/turtle1/cmd_vel` topic to drive a turtle in the TurtleSim simulator, making it trace a square. The timing and velocity values determine the accuracy of the shape.

**Challenge 2 — Stage Challenge 🗺️**
A differential robot is spawned at coordinates `(x=-7.0, y=-7.0)` and must autonomously navigate to the target `(x=4.5, y=4.0)` in a cave-like environment. The node subscribes to `/odom` (odometry) and `/base_scan` (LiDAR with 1080 readings over 270°) and publishes velocity commands to `/cmd_vel`. Obstacle avoidance logic runs continuously to ensure the robot reaches its goal within an error margin of ±0.4 on both axes.

**Challenge 3 — YASMIN Challenge 🔀**
Using the YASMIN state machine framework, the node orchestrates a turtle in TurtleSim to draw the uppercase letter **F**. The state machine alternates between `MOVE` (forward motion) and `ROTATE` (angular motion) states, with transitions determined by pre-defined outcomes. A shared `Blackboard` object is used to pass data between states.

---

## 🗂️ Code Structure

```
fbot_learning_my_solutions/
│
├── 1-turtle_challenge/
│   └── turtle_challenge/
│       ├── challenge_node.py       # Main ROS 2 node: publishes Twist to draw a square
│       ├── package.xml
│       └── setup.py
│
├── 2-stage_challenge/
│   └── stage_challenge/
│       ├── challenge_node.py       # Navigation node with obstacle avoidance logic
│       ├── package.xml
│       └── setup.py
│
├── 3-yasmin_challenge/
│   └── yasmin_challenge/
│       ├── challenge_node.py       # YASMIN state machine to draw the letter F
│       ├── package.xml
│       └── setup.py
│
└── README.md
```

> **Note:** Each challenge is a standalone ROS 2 Python package. They share no code between them and must be built and run independently.


## 📝 Notes

- All solutions require a **sourced ROS 2 workspace** before execution (`source install/setup.bash`).
- The workspace must be named `fbot_ws` and located in the home directory, as per the official fbot_learning instructions.
- The **Stage Challenge** requires the `stage_ros2` package to be installed and the `cave` world file to be available.
- The **YASMIN Challenge** requires `ros-yasmin` and `ros-yasmin-ros` to be installed. 
- The LiDAR in Challenge 2 covers **270 degrees** with **1080 samples**: index `0` points right, index `539` points forward, and index `1079` points left. This indexing must be carefully handled in the obstacle avoidance logic.


## 🐛 Common Errors

| Error | Possible Cause | Solution |
|---|---|---|
| `Package 'turtle_challenge' not found` | Package not built or workspace not sourced | Run `colcon build` then `source install/setup.bash` |
| Turtle draws incorrect shape | Wrong velocity or sleep duration values | Tune `linear.x`, `angular.z`, and `time.sleep()` values carefully |
| Robot not moving in Stage | `/cmd_vel` topic name mismatch | Check exact topic name with `ros2 topic list` |
| Robot stuck in obstacle loop | LiDAR index slicing incorrect | Verify front/left/right index ranges for the 270° sensor |
| `NaN` values from LiDAR | Sensor readings out of range | Filter `inf` and `nan` values before using `min(ranges)` |
| YASMIN state not transitioning | Outcome string mismatch | Ensure outcome returned by `execute()` matches declared outcomes exactly |
| `ros-humble-yasmin` not found | Package unavailable in apt | Build YASMIN from source inside `fbot_ws/src` |
| Robot overshoots target | Distance threshold too large | Reduce the error tolerance below `0.4` if needed |
| `ModuleNotFoundError` for YASMIN | YASMIN not installed or not built | Re-run `colcon build --symlink-install` after cloning YASMIN |
| Turtlesim not spawning turtle | `turtlesim_node` not running | Start the simulator before launching the challenge node |

---

## 🏷️ Version

| Version | Date | Description |
|---|---|---|
| `1.0.0` | 2026 | Initial solutions: Turtle Challenge (square), Stage Challenge (navigation + obstacle avoidance), YASMIN Challenge (letter F state machine) |

---

## 👥 Team

| Name | Role |
|---|---|
| **Gabriel** | Author — Solution developer for all 3 fbot_learning challenges |
| **Gabriel Dorneles** ([gadorneles](https://github.com/gadorneles)) | Challenge creator and maintainer (fbot_learning original repo) |
| **Ricardo Grando** ([ricardoGrando](https://github.com/ricardoGrando)) | Stage Challenge original concept |

---

> 🗺️ *"Every autonomous journey begins with a single `cmd_vel` message."*

---

<div align="center">

**📦 Based on:** [fbotathome/fbot_learning](https://github.com/fbotathome/fbot_learning) · **🤖 Team:** FBOT

</div>