#!/usr/bin/env python3
import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer, ActionClient

from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.z = math.sin(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    return q


class FakeNavExploreDebug(Node):
    """
    (1) 가짜 Nav2 ActionServer: /navigate_to_pose (항상 켜짐)
        - goal 오면 succeed_delay_sec 후 SUCCEEDED

    (2) /explore/init, /explore/return 구독:
        - True 받으면 trigger_delay_sec 후에 (자기 자신에게) /navigate_to_pose goal 전송
        - 결과적으로 /navigate_to_pose/_action/status 에 SUCCEEDED 뜸 (WaitGoalReached 흉내)

    (3) 노드 시작 후 publish_delay_sec 지나면:
        - /human/hand_up_goal (PoseStamped)   [frame_id는 무조건 "map"으로 고정]
        - /human/states (String)             [HAND_UP_LEFT/RIGHT 포함되면 Waving SUCCESS 조건]
      를 "같이" publish (1회 or 주기)
    """

    def __init__(self):
        super().__init__("fake_nav_explore_debug")
        self.cb = ReentrantCallbackGroup()

        # -------- Params --------
        self.declare_parameter("action_name", "/navigate_to_pose")

        self.declare_parameter("publish_delay_sec", 5.0)
        self.declare_parameter("publish_rate_hz", 1.0)  # 0이면 1회만

        # hand_up_goal pose
        self.declare_parameter("goal_x", 2.8)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_yaw", 0.0)

        # states string (여기에 HAND_UP_LEFT/RIGHT만 포함되면 detected=true)
        self.declare_parameter("states_text", "P0:HAND_UP_LEFT, P1:UNKNOWN")

        # explore trigger -> nav goal
        self.declare_parameter("trigger_delay_sec", 5.0)   # init/return 받은 뒤 goal 보내기까지
        self.declare_parameter("succeed_delay_sec", 5.0)   # goal 받은 뒤 SUCCEEDED까지

        self.declare_parameter("init_goal_x", 1.0)
        self.declare_parameter("init_goal_y", 0.0)
        self.declare_parameter("init_goal_yaw", 0.0)

        self.declare_parameter("return_goal_x", 0.0)
        self.declare_parameter("return_goal_y", 0.0)
        self.declare_parameter("return_goal_yaw", 0.0)

        self.action_name = str(self.get_parameter("action_name").value)

        # -------- Publishers (Waving 입력 맞춤) --------
        self.pub_hand_goal = self.create_publisher(PoseStamped, "/human/hand_up_goal", 10)
        self.pub_states = self.create_publisher(String, "/human/states", 10)

        # -------- Subscribers (BT가 쏘는 것들 보기) --------
        self.sub_init = self.create_subscription(Bool, "/explore/init", self._on_init, 10)
        self.sub_return = self.create_subscription(Bool, "/explore/return", self._on_return, 10)
        self.sub_resume = self.create_subscription(Bool, "/explore/resume", self._on_resume, 10)
        self.sub_save = self.create_subscription(Bool, "/explore/save", self._on_save, 10)

        # -------- Fake Nav2 (Action Server + self-client) --------
        self._server = ActionServer(
            self,
            NavigateToPose,
            self.action_name,
            execute_callback=self._execute_cb,
            callback_group=self.cb,
        )
        self._client = ActionClient(
            self,
            NavigateToPose,
            self.action_name,
            callback_group=self.cb,
        )

        self.get_logger().info(f"[FakeNav] ActionServer up: {self.action_name}")
        self.get_logger().info("[FakeExplore] listening: /explore/init, /explore/return (+ /explore/save, /explore/resume)")
        self.get_logger().info("[FakeWaving] will publish /human/hand_up_goal + /human/states after delay")

        # 시작 후 delay 지나면 publish 시작
        self._t0 = time.time()
        self._started_pub = False
        self.create_timer(0.1, self._maybe_start_publish)

    # ---------------- publish goal + states (together) ----------------
    def _maybe_start_publish(self):
        if self._started_pub:
            return
        delay = float(self.get_parameter("publish_delay_sec").value)
        if time.time() - self._t0 < delay:
            return

        self._started_pub = True
        rate = float(self.get_parameter("publish_rate_hz").value)
        self.get_logger().info(f"[FakeWaving] publishing started (rate_hz={rate})")

        # 즉시 1회 같이 publish
        self._publish_goal_and_states_once()

        # 주기 publish 원하면 타이머 추가
        if rate > 0.0:
            period = 1.0 / rate
            self.create_timer(period, self._publish_goal_and_states_once)

    def _publish_goal_and_states_once(self):
        x = float(self.get_parameter("goal_x").value)
        y = float(self.get_parameter("goal_y").value)
        yaw = float(self.get_parameter("goal_yaw").value)
        states_text = str(self.get_parameter("states_text").value)

        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = "map"  # ✅ 무조건 map 고정
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = 0.0
        ps.pose.orientation = yaw_to_quat(yaw)

        s = String()
        s.data = states_text  # 여기에 HAND_UP_LEFT/RIGHT 포함되면 Waving SUCCESS 조건

        # ✅ 같이 publish
        self.pub_hand_goal.publish(ps)
        self.pub_states.publish(s)

        self.get_logger().info(
            f"[TX] /human/hand_up_goal ({x:.2f},{y:.2f}) frame=map  +  /human/states='{states_text}'"
        )

    # ---------------- explore triggers ----------------
    def _on_resume(self, msg: Bool):
        self.get_logger().info(f"[RX] /explore/resume={msg.data}")

    def _on_save(self, msg: Bool):
        self.get_logger().info(f"[RX] /explore/save={msg.data}")

    def _on_init(self, msg: Bool):
        if not msg.data:
            return
        self.get_logger().info("[RX] /explore/init=True -> will send NAV goal after trigger_delay_sec")
        self._schedule_goal(kind="init")

    def _on_return(self, msg: Bool):
        if not msg.data:
            return
        self.get_logger().info("[RX] /explore/return=True -> will send NAV goal after trigger_delay_sec")
        self._schedule_goal(kind="return")

    def _schedule_goal(self, kind: str):
        delay = float(self.get_parameter("trigger_delay_sec").value)
        t = threading.Timer(delay, lambda: self._send_goal(kind))
        t.daemon = True
        t.start()

    def _send_goal(self, kind: str):
        if not self._client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error(f"[FakeExplore] action server not ready: {self.action_name}")
            return

        if kind == "init":
            gx = float(self.get_parameter("init_goal_x").value)
            gy = float(self.get_parameter("init_goal_y").value)
            gyaw = float(self.get_parameter("init_goal_yaw").value)
        else:
            gx = float(self.get_parameter("return_goal_x").value)
            gy = float(self.get_parameter("return_goal_y").value)
            gyaw = float(self.get_parameter("return_goal_yaw").value)

        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = "map"
        ps.pose.position.x = gx
        ps.pose.position.y = gy
        ps.pose.position.z = 0.0
        ps.pose.orientation = yaw_to_quat(gyaw)

        goal = NavigateToPose.Goal()
        goal.pose = ps

        self.get_logger().info(f"[FakeExplore] send_goal({kind}) -> ({gx:.2f},{gy:.2f}) frame=map")
        self._client.send_goal_async(goal)

    # ---------------- fake nav2 action server ----------------
    def _execute_cb(self, goal_handle):
        pose = goal_handle.request.pose
        succeed_delay = float(self.get_parameter("succeed_delay_sec").value)

        self.get_logger().info(
            f"[FakeNav] goal accepted -> executing {succeed_delay:.1f}s "
            f"target=({pose.pose.position.x:.2f},{pose.pose.position.y:.2f})"
        )
        time.sleep(succeed_delay)

        goal_handle.succeed()
        self.get_logger().info("[FakeNav] SUCCEEDED (status topic should reflect this)")
        return NavigateToPose.Result()


def main():
    rclpy.init()
    node = FakeNavExploreDebug()
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(node)

    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
