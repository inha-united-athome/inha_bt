<div align="center">
  <img src="https://github.com/inha-united-athome/.github/raw/main/profile/inha_logo.png" width="110" /><br/>
  <h1>inha_bt</h1>
  <span><b>ROS 2 Behavior Tree package for @Home missions</b></span><br/><br/>

  <img src="https://img.shields.io/badge/Ubuntu-22.04-orange?style=flat&logo=ubuntu" />
  <img src="https://img.shields.io/badge/C++-17-blue?style=flat&logo=cplusplus" />
  <img src="https://img.shields.io/badge/ROS2-Humble-blue?style=flat&logo=ros" />
  <img src="https://img.shields.io/badge/BehaviorTree.CPP-4.x-green?style=flat" />
  <br/><br/>

  <a href="#overview">Overview</a> • <a href="#architecture">Architecture</a> • <a href="#installation">Installation</a> • <a href="#usage">Usage</a> • <a href="#bt-nodes">BT Nodes</a> • <a href="#interfaces">Interfaces</a> • <a href="#contributing">Contributing</a>
</div>

---

## Overview

`inha_bt` is a Behavior Tree (BT) based task planner used by the Inha-United@Home robot stack. It leverages BehaviorTree.CPP and BehaviorTree.ROS2 to provide modular BT nodes for HRI, navigation, manipulation, perception, and mission orchestration.

---

## Architecture

```text
src/
├── 📁 inha_bt_src/
│   ├── 📁 BehaviorTree.CPP/       # BT.CPP library
│   ├── 📁 BehaviorTree.ROS2/      # BT-ROS2 integration
│   └── 📁 inha_bt_pkg/
│       ├── 📁 bt_xmls/            # Behavior Tree XML definitions
│       │   ├── 📁 missions/       # main mission trees
│       │   ├── 📁 subtrees/       # reusable subtrees
│       │   └── 📁 tests/          # test trees
│       ├── 📁 configs/            # parameters and configs
│       ├── 📁 include/            # headers
│       └── 📁 src/                # BT node implementations
│           ├── 📁 hri/            # speech, gesture, HRI nodes
│           ├── 📁 navigation/     # navigation nodes
│           ├── 📁 manipulation/   # manipulation nodes
│           ├── 📁 perception/     # perception nodes
│           └── 📁 utils/          # utility nodes
├── 📁 inha_interfaces/            # custom action/service definitions
└── 📁 backend_node/               # backend nodes (e.g. VLM)
```

---

## Installation

**Prerequisites**
- Ubuntu 22.04
- ROS 2 Humble
- `colcon`, `rosdep`

```bash
mkdir -p ~/bt_ws/src
cd ~/bt_ws/src
git clone --recursive https://github.com/inha-united-athome/inha_bt.git

cd ~/bt_ws
sudo rosdep init  # skip if already initialized
rosdep update
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
source install/setup.bash
```

---

## Usage

```bash
# Run the BT node
ros2 run inha_bt_pkg bt_node <xml_path>

# Example: restaurant mission
ros2 run inha_bt_pkg bt_node missions/restaurant_main.xml

# Example: speak test
ros2 run inha_bt_pkg bt_node tests/speak_test.xml
```


---

## Interfaces

**Actions** (`inha_interfaces/action/`)
| Name | Description |
|------|-------------|
| `CaptureFaceCrop` | Capture face crop |
| `Detection` | Object detection |
| `FollowHuman` | Follow a person |
| `Listen` | Speech recognition / listening |
| `SetRobotPose` | Set robot pose |
| `Speak` | Text-to-speech / speak |
| `VisualAlign` | Visual alignment |
| `Vlm` | Vision-Language Model inference |
| `WaitPerson` | Wait for person |
| `Waving` | Detect waving gesture |
| `WavingApproach` | Approach a waving person |

**Services** (`inha_interfaces/srv/`)
| Name | Description |
|------|-------------|
| `ExecuteSuccess` | Acknowledge execution success |
| `GraspgenEnable` | Enable grasp generation |
| `MoveitEnable` | Enable MoveIt integration |
| `SetEnable` | Toggle feature enable/disable |
| `StartReplay` | Start replay mode |
| `StopReplay` | Stop replay mode |

---

## Contributing

- Create a branch: `git checkout -b feat/<name>`
- Commit: `git commit -m "Add <feature>"`
- Push: `git push origin feat/<name>`
- Open a Pull Request
