## Run BT
```
ros2 run inha_bt_pkg bt_node xml
```

xml :
missions/restaurant_main.xml

tests/speak_test.xml




| BT Node | Publishes | Subscribes | Action |
|---|---|---|---|
| `Waving` |  | `/human/states`<br>`/human/hand_up_goal` |  |
| `GoToPose` |  |  | `/navigate_to_pose` |
| `ExplorationAction` | `/explore/resume` |  |  |
| `ExplorationSaveGoal` | `/explore/save` |  |  |
| `ExploreTrigger` | `/explore/init`<br>`/explore/return` |  |  |
| `WaitGoalReached` |  | `/navigate_to_pose/_action/status` |  |

## BT Flow (restaurant_main)

1) `Parallel`
   - `ExplorationAction`
   - `Waving`  → writes `{w_pose}`

2) `GoToPose(goal={w_pose})`

3) `ExplorationSaveGoal`

4) `ExploreTrigger(cmd=init)`
5) `WaitGoalReached`

6) `ExploreTrigger(cmd=return)`
7) `WaitGoalReached`

8) `ExploreTrigger(cmd=init)`
9) `WaitGoalReached`

## BT Flow (receptionist_main)

