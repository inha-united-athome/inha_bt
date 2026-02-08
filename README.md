<div align="center">
  <img src="https://github.com/inha-united-athome/.github/raw/main/profile/inha_logo.png" width="110" /><br/>
  <h1>inha_bt</h1>
  <span><b>ROS 2 Behavior Tree package for autonomous mission control</span><br/><br/>

  <img src="https://img.shields.io/badge/Ubuntu-22.04-orange?style=flat&logo=ubuntu" />
  <img src="https://img.shields.io/badge/Python-3.10-blue?style=flat&logo=python" />
  <img src="https://img.shields.io/badge/ROS2-Humble-blue?style=flat&logo=ros" />
  <br/><br/>

  <a href="#overview">Overview</a> • <a href="#architecture">Architecture</a> • <a href="#installation">Installation</a> • <a href="#usage">Usage</a> • <a href="#bt-nodes">BT Nodes</a> • <a href="#bt-flows">BT Flows</a> • <a href="#contributing">Contributing</a>
</div>


Overview
---
`inha_bt` provides behavior tree-based mission control for the Inha-United@Home robot stack. It orchestrates navigation, exploration, and human interaction tasks using BehaviorTree.CPP for autonomous mission execution.

Architecture
---
```text
inha_bt/
├── 📁 backend_node/        # Backend processing nodes
├── 📁 inha_bt_src/         # Core BT implementation
│   ├── 📁 missions/        # Main mission XML files
│   │   └── restaurant_main.xml
│   │   └── receptionist_main.xml
│   └── 📁 tests/           # Test BT XML files
│       └── speak_test.xml
├── 📁 inha_interfaces/     # Custom messages and services
└── 📄 README.md
```

Installation
---
**Prerequisites**
- Ubuntu 22.04
- ROS 2 Humble
- `colcon`, `rosdep`
- BehaviorTree.CPP 3.x

```bash
mkdir -p ~/inha_ws/src
cd ~/inha_ws/src
git clone https://github.com/inha-united-athome/inha_bt.git

cd ~/inha_ws
sudo rosdep init  # skip if already initialized
rosdep update
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
source install/setup.bash
```

Usage
---
**Run Behavior Tree**
```bash
ros2 run inha_bt_pkg bt_node <xml_file>
```

**Available missions:**
- `missions/restaurant_main.xml` – Restaurant service mission
- `missions/receptionist_main.xml` – Receptionist mission
- `tests/speak_test.xml` – Speech test

**Example:**
```bash
ros2 run inha_bt_pkg bt_node missions/restaurant_main.xml
```

BT Nodes
---
| BT Node | Publishes | Subscribes | Action |
|---|---|---|---|
| `Waving` |  | `/human/states`<br>`/human/hand_up_goal` |  |
| `GoToPose` |  |  | `/navigate_to_pose` |
| `ExplorationAction` | `/explore/resume` |  |  |
| `ExplorationSaveGoal` | `/explore/save` |  |  |
| `ExploreTrigger` | `/explore/init`<br>`/explore/return` |  |  |
| `WaitGoalReached` |  | `/navigate_to_pose/_action/status` |  |

BT Flows
---

### Restaurant Main (`restaurant_main.xml`)

1. **`Parallel`**
   - `ExplorationAction` – Start exploration
   - `Waving` – Detect waving human → writes `{w_pose}`

2. **`GoToPose(goal={w_pose})`** – Navigate to detected person

3. **`ExplorationSaveGoal`** – Save current exploration goal

4. **`ExploreTrigger(cmd=init)`** – Initialize exploration  
   **`WaitGoalReached`** – Wait for goal

5. **`ExploreTrigger(cmd=return)`** – Return to saved goal  
   **`WaitGoalReached`** – Wait for return

6. **`ExploreTrigger(cmd=init)`** – Resume exploration  
   **`WaitGoalReached`** – Wait for completion

### Receptionist Main (`receptionist_main.xml`)

_(To be documented)_

Contributing
---
- Create a branch: `git checkout -b feat/<name>`
- Commit: `git commit -m "Add <feature>"`
- Push: `git push origin feat/<name>`
- Open a Pull Request