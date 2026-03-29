#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.logging import LoggingSeverity
import time
import random

TIMER_PERIOD = 0.5


class IdleBehavior(Node):

    def __init__(self):
        super().__init__('idle_behavior')
        self.get_logger().set_level(LoggingSeverity.INFO)

        self.pub = self.create_publisher(String, 'esp_tx', 10)
        self.create_subscription(String, 'obstacle_status',
                                 self.obstacle_callback, 10)

        # ================= MOVEMENT =================
        self.state = "STOP"
        self.state_end_time = time.time() + 3.0
        self.last_action = None

        # ================= OBSTACLE =================
        self.avoid_mode = False
        self.avoid_stage = None
        self.avoid_end_time = None

        # ================= ARM LIMITS =================
        self.ELBOW_MIN = 0
        self.ELBOW_MAX = 30
        self.BEND_MIN = 20
        self.BEND_MAX = 60
        self.SHOULDER_MIN = 50
        self.SHOULDER_MAX = 100

        # ================= ARM STATE =================
        self.elbow = 15
        self.bend = 40
        self.shoulder = 75

        self.target_elbow = 15
        self.target_bend = 40
        self.target_shoulder = 75

        self.grip_open = True
        self.grip_close_time = None

        self.last_arm_update = time.time()
        self.last_arm_publish = time.time()

        self.timer = self.create_timer(TIMER_PERIOD, self.loop)

        self.get_logger().info("🟢 Idle started")

    # ==================================================
    def obstacle_callback(self, msg):

        if msg.data == "OBSTACLE" and not self.avoid_mode:
            self.get_logger().warn("🚨 Obstacle detected")

            self.avoid_mode = True
            self.avoid_stage = "PAUSE"
            self.avoid_end_time = time.time() + random.uniform(1.0, 2.0)

    # ==================================================
    def publish(self, action):

        if action != self.last_action:
            self.pub.publish(String(data=f"IDLE:{action}"))
            self.get_logger().info(f"IDLE → {action}")
            self.last_action = action

    # ==================================================
    def publish_arm(self):

        if time.time() - self.last_arm_publish < 1.0:
            return

        self.last_arm_publish = time.time()

        self.pub.publish(String(data=f"ARM:ELBOW:{self.elbow}"))
        self.pub.publish(String(data=f"ARM:BEND:{self.bend}"))
        self.pub.publish(String(data=f"ARM:SHOULDER:{self.shoulder}"))

        if self.grip_open:
            self.pub.publish(String(data="ARM:GRIP:OPEN"))
        else:
            self.pub.publish(String(data="ARM:GRIP:CLOSE"))

    # ==================================================
    def update_arm_targets(self):

        if time.time() - self.last_arm_update < 2.0:
            return

        self.last_arm_update = time.time()

        self.target_elbow = random.randint(self.ELBOW_MIN, self.ELBOW_MAX)
        self.target_bend = random.randint(self.BEND_MIN, self.BEND_MAX)
        self.target_shoulder = random.randint(self.SHOULDER_MIN, self.SHOULDER_MAX)

        if random.random() > 0.6 and self.grip_open:
            self.grip_open = False
            self.grip_close_time = time.time()

    # ==================================================
    def smooth_move(self):
        self.elbow = self.target_elbow
        self.bend = self.target_bend
        self.shoulder = self.target_shoulder

    # ==================================================
    def choose_next_state(self):

        r = random.random()

        if r < 0.40:
            self.state = "STOP"
            duration = random.uniform(2.0, 4.0)

        elif r < 0.70:
            self.state = "FORWARD"
            duration = random.uniform(1.0, 2.0)

        else:
            self.state = random.choice(["LEFT", "RIGHT"])
            duration = random.uniform(0.8, 1.5)

        self.state_end_time = time.time() + duration

    # ==================================================
    def handle_grip_auto_open(self):

        if not self.grip_open and self.grip_close_time:
            if time.time() - self.grip_close_time > 0.5:
                self.grip_open = True
                self.grip_close_time = None

    # ==================================================
    def loop(self):

        now = time.time()

        # ================= OBSTACLE =================
        if self.avoid_mode:

            if self.avoid_stage == "PAUSE":

                self.publish("STOP")

                if now >= self.avoid_end_time:
                    self.avoid_stage = "TURN"
                    self.state = random.choice(["LEFT", "RIGHT"])
                    self.avoid_end_time = now + random.uniform(1.5, 2.5)

            elif self.avoid_stage == "TURN":

                self.publish(self.state)

                if now >= self.avoid_end_time:
                    self.avoid_mode = False
                    self.state = "STOP"
                    self.state_end_time = now + random.uniform(2.0, 3.0)

            return

        # ================= NORMAL =================
        if now >= self.state_end_time:
            self.choose_next_state()

        self.publish(self.state)

        # Arm only during STOP
        if self.state == "STOP":
            self.update_arm_targets()
            self.smooth_move()
            self.handle_grip_auto_open()
            self.publish_arm()

    # ==================================================
    def destroy_node(self):
        print("🔴 Idle stopped")
        super().destroy_node()


def main():
    rclpy.init()
    node = IdleBehavior()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 Ctrl+C pressed")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
