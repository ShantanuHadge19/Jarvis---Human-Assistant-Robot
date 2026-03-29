#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

import cv2
import numpy as np
import cv2.aruco as aruco
import time

# ===== SETTINGS =====
MARKER_SIZE = 0.047
TARGET_DISTANCE = 0.4

CENTER_TOL = 80
BIG_ERROR = 180

TURN_TIME = 0.08
STOP_TIME = 0.4

CAM_MATRIX_PATH = "/home/pi/camera_calibration/camera_matrix.npy"
DIST_PATH = "/home/pi/camera_calibration/dist_coeffs.npy"


class Docking(Node):

    def __init__(self):
        super().__init__("pick_the_cup")

        self.bridge = CvBridge()

        self.camera_matrix = np.load(CAM_MATRIX_PATH)
        self.dist_coeffs = np.load(DIST_PATH)

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

        self.sub = self.create_subscription(Image, "/image_raw", self.cb, 10)
        self.pub = self.create_publisher(String, "esp_tx", 10)

        self.filtered_error = None
        self.locked = False

        self.last_turn_time = 0
        self.turn_cooldown = 0.15


        # ===== SEARCH =====
        self.searching = False
        self.search_pause = False
        self.search_until = 0

        # ===== DETECTION LOCK =====
        self.marker_locked = False
        self.lost_count = 0
        self.LOST_THRESHOLD = 3

        cv2.namedWindow("Docking View", cv2.WINDOW_NORMAL)

    # -----------------
    def send(self, cmd):
        self.pub.publish(String(data=f"GESTURE:{cmd}"))

    # -----------------
    def cb(self, msg):

        if self.locked:
            self.send("STOP")
            return

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        frame = cv2.resize(frame, (640, 480))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        now = time.time()

        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict)

        # ================= SEARCH =================
        if ids is None and not self.marker_locked:

            # CW start
            if not self.searching and not self.search_pause:
                self.send("CW")
                self.searching = True
                self.search_until = now + TURN_TIME
                return

            # STOP after turn
            if self.searching and now >= self.search_until:
                self.send("STOP")
                self.searching = False
                self.search_pause = True
                self.search_until = now + STOP_TIME
                return

            # resume
            if self.search_pause and now >= self.search_until:
                self.search_pause = False

            cv2.imshow("Docking View", frame)
            cv2.waitKey(1)
            return

        # ================= MARKER FOUND =================
        if ids is not None:
            self.marker_locked = True
            self.lost_count = 0
        else:
            self.lost_count += 1
            if self.lost_count < self.LOST_THRESHOLD:
                self.send("STOP")
                return

        aruco.drawDetectedMarkers(frame, corners)

        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners, MARKER_SIZE, self.camera_matrix, self.dist_coeffs
        )

        # 🔥 SAFETY CHECK
        if tvecs is None or len(tvecs) == 0:
            self.send("STOP")
            return

        z = tvecs[0][0][2]

        pts = corners[0][0]
        cx = int(np.mean(pts[:, 0]))
        frame_center = frame.shape[1] // 2

        error = cx - frame_center

        # ===== SMOOTH =====
        if self.filtered_error is None:
            self.filtered_error = error

        self.filtered_error = 0.7 * self.filtered_error + 0.3 * error
        e = self.filtered_error

        # ===== DRAW =====
        cv2.circle(frame, (cx, int(np.mean(pts[:, 1]))), 5, (0,255,0), -1)
        cv2.line(frame, (frame_center, 0), (frame_center, 480), (255,0,0), 2)

        # ================= CONTROL =================

        if e > BIG_ERROR:
            self.send("CW")

        elif e < -BIG_ERROR:
            self.send("CCW")

        now = time.time()

        if e > CENTER_TOL:
            if now - self.last_turn_time > self.turn_cooldown:
                self.send("CW")
                time.sleep(0.05)   # short pulse
                self.send("STOP")
                self.last_turn_time = now

        elif e < -CENTER_TOL:   
            if now - self.last_turn_time > self.turn_cooldown:
                self.send("CCW")
                time.sleep(0.05)
                self.send("STOP")
                self.last_turn_time = now

        else:
            if z > TARGET_DISTANCE:
                self.send("FORWARD")
            else:
                self.send("STOP")
                time.sleep(0.2)

                # 🔥 SWITCH TO LINE FOLLOW
                self.pub.publish(String(data="MODE:GESTURE"))

                self.locked = True

                # 🔥 STOP THIS NODE (VERY IMPORTANT)
                self.destroy_node()
                return

        # ===== DISPLAY =====
        cv2.imshow("Docking View", frame)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = Docking()
    rclpy.spin(node)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
