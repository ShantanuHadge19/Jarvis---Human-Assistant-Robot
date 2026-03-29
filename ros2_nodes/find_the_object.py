#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

import cv2
import numpy as np
import time
import signal
import sys

# ================= SETTINGS =================
FRAME_WIDTH = 320
DEADZONE_RATIO = 0.15
STOP_SIZE = 140

TURN_TIME = 0.08
STOP_TIME = 0.4


class ClockTracker(Node):

    def __init__(self):
        super().__init__("clock_tracker")

        signal.signal(signal.SIGINT, self.shutdown)
        signal.signal(signal.SIGTERM, self.shutdown)

        self.bridge = CvBridge()

        self.sub = self.create_subscription(Image, "/image_raw", self.cb, 10)
        self.pub = self.create_publisher(String, "esp_tx", 10)

        self.last_cmd = None
        self.finished = False

        self.turning = False
        self.turn_until = 0
        self.turn_pause = False

        # SEARCH
        self.searching = False
        self.search_pause = False
        self.search_until = 0

        # LOCK
        self.object_locked = False
        self.lost_count = 0
        self.LOST_THRESHOLD = 5

        cv2.namedWindow("Clock Tracker", cv2.WINDOW_NORMAL)

        self.get_logger().info("🔥 Clock Tracker Started")

    # =====================================================
    def send(self, cmd):
        msg = f"GESTURE:{cmd}"
        if msg != self.last_cmd:
            self.pub.publish(String(data=msg))
            self.last_cmd = msg

    # =====================================================
    def cb(self, msg):

        # 🔥 GET FRAME
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        frame = cv2.flip(frame, 1)
        frame = cv2.resize(frame, (320, 240))   # important for performance

        now = time.time()

        found, cx, size = self.detect(frame)

        # ================= SEARCH =================
        if not found and not self.object_locked:

            if not self.searching and not self.search_pause:
                self.send("CW")
                self.searching = True
                self.search_until = now + TURN_TIME

            elif self.searching and now >= self.search_until:
                self.send("STOP")
                self.searching = False
                self.search_pause = True
                self.search_until = now + STOP_TIME

            elif self.search_pause and now >= self.search_until:
                self.search_pause = False

        if not found and self.object_locked:
            self.send("STOP")

        # ================= FOUND / LOST =================
        if found:
            self.object_locked = True
            self.lost_count = 0
        else:
            self.lost_count += 1

            

        # ================= TRACK =================
        if found:

            h, w = frame.shape[:2]
            center = w // 2
            error = cx - center
            deadzone = int(DEADZONE_RATIO * w)

            now = time.time()

            if abs(error) > deadzone:

                if not self.turning and not self.turn_pause:

                    if error > 0:
                        self.send("CCW")
                    else:
                       self.send("CW")

                    self.turning = True
                    self.turn_until = now + 0.05   # 🔥 VERY SHORT TURN

                elif self.turning and now >= self.turn_until:
                    self.send("STOP")
                    self.turning = False
                    self.turn_pause = True
                    self.turn_until = now + 0.08   # small pause

                elif self.turn_pause and now >= self.turn_until:
                    self.turn_pause = False

            else:
                if size < STOP_SIZE:
                    self.send("FORWARD")
                else:
                    self.send("STOP")

                    if not self.finished:
                        self.get_logger().info("✅ CLOCK REACHED")
                        self.pub.publish(String(data="MODE:GESTURE"))
                        self.finished = True

        # ================= ALWAYS DISPLAY =================
        cv2.imshow("Clock Tracker", frame)
        cv2.waitKey(1)

    # =====================================================
    def detect(self, frame):

        h, w = frame.shape[:2]
        scale = FRAME_WIDTH / w
        small = cv2.resize(frame, (FRAME_WIDTH, int(h * scale)))

        hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)

        # 🟡 YELLOW MASK
        lower_yellow = np.array([15, 100, 100])
        upper_yellow = np.array([35, 255, 255])
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # ⚫ BLACK MASK
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 60])
        mask_black = cv2.inRange(hsv, lower_black, upper_black)

        # CLEAN
        kernel = np.ones((5,5), np.uint8)
        mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return False, None, None

        for c in contours:

            area = cv2.contourArea(c)
            if area < 800:
                continue

            x, y, w_box, h_box = cv2.boundingRect(c)

            # 🧠 circular shape filter
            aspect_ratio = w_box / h_box
            if aspect_ratio < 0.7 or aspect_ratio > 1.3:
                continue

            # 🧠 black border check
            pad = 5
            x1 = max(0, x - pad)
            y1 = max(0, y - pad)
            x2 = min(mask_black.shape[1], x + w_box + pad)
            y2 = min(mask_black.shape[0], y + h_box + pad)

            border = mask_black[y1:y2, x1:x2]
            black_ratio = np.sum(border > 0) / (border.size + 1)

            if black_ratio < 0.05:
                continue

            # ✅ VALID CLOCK
            cx_small = x + w_box // 2
            cx = int(cx_small / scale)
            size = max(w_box, h_box)

            # DRAW
            cv2.rectangle(frame,
                          (int(x/scale), int(y/scale)),
                          (int((x+w_box)/scale), int((y+h_box)/scale)),
                          (0,255,0), 2)

            cv2.putText(frame, "CLOCK",
                        (int(x/scale), int(y/scale)-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

            return True, cx, size

        return False, None, None

    # =====================================================
    def shutdown(self, *args):
        cv2.destroyAllWindows()
        rclpy.shutdown()
        sys.exit(0)


def main():
    rclpy.init()
    node = ClockTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
