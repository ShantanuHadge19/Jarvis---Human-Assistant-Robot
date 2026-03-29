#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import cv2
import os
import numpy as np
import time

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

# ================= CONFIG =================
IMAGE_TOPIC = "/image_raw"
DATASET_PATH = "/home/pi/ros2_ws/src/jarvis1/data/known_persons/shantanu"

FRAME_SKIP = 3

TURN_TIME = 0.05
STOP_TIME = 0.5

CENTER_TOL = 60
TOO_FAR = 120

CONF_THRESHOLD = 110


class HumanTracker(Node):

    def __init__(self):
        super().__init__('human_tracker_node')

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, IMAGE_TOPIC, self.image_cb, 10)
        self.pub = self.create_publisher(String, "esp_tx", 10)

        self.face_cascade = cv2.CascadeClassifier(
            "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml"
        )

        # LBPH
        self.recognizer = cv2.face.LBPHFaceRecognizer_create()
        self.train_model()

        # STATES
        self.frame_count = 0
        self.target_acquired = False

        # SEARCH
        self.searching = False
        self.search_pause = False
        self.search_until = 0

        # anti-spam
        self.last_cmd = None

        cv2.namedWindow("Human Tracker", cv2.WINDOW_NORMAL)

        print("🔥 LBPH TRACKER STARTED")

    # ================= TRAIN =================
    def train_model(self):
        faces = []
        labels = []

        for file in os.listdir(DATASET_PATH):
            path = os.path.join(DATASET_PATH, file)

            img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
            if img is None:
                continue

            img = cv2.resize(img, (200, 200))
            faces.append(img)
            labels.append(1)

        self.recognizer.train(faces, np.array(labels))
        print("✅ TRAINED ON", len(faces), "IMAGES")

    # ================= SEND =================
    def send(self, cmd):
        if cmd != self.last_cmd:
            self.pub.publish(String(data=f"GESTURE:{cmd}"))
            self.last_cmd = cmd

    # ================= CALLBACK =================
    def image_cb(self, msg):

        self.frame_count += 1
        if self.frame_count % FRAME_SKIP != 0:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        frame = cv2.resize(frame, (320, 240))

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        now = time.time()

        faces = self.face_cascade.detectMultiScale(gray, 1.1, 4)

        found = False
        target = None

        for (x, y, w, h) in faces:

            face = gray[y:y+h, x:x+w]
            face = cv2.resize(face, (200, 200))

            label, confidence = self.recognizer.predict(face)

            if confidence < CONF_THRESHOLD:
                found = True
                target = (x, y, w, h)

                self.target_acquired = True
                self.searching = False
                self.search_pause = False

                # DRAW GREEN BOX
                cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 2)
                cv2.putText(frame, f"SHANTANU {int(confidence)}",
                            (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                break
            else:
                cv2.rectangle(frame, (x,y), (x+w,y+h), (0,0,255), 2)

        # ================= SEARCH =================
        if not found and not self.target_acquired:

            if not self.searching and not self.search_pause:
                self.send("CW")
                self.searching = True
                self.search_until = now + TURN_TIME
                return

            if self.searching and now >= self.search_until:
                self.send("STOP")
                self.searching = False
                self.search_pause = True
                self.search_until = now + STOP_TIME
                return

            if self.search_pause and now >= self.search_until:
                self.search_pause = False

            cv2.imshow("Human Tracker", frame)
            cv2.waitKey(1)
            return

        # ================= LOST AFTER LOCK =================
        if not found and self.target_acquired:
            self.send("STOP")

            cv2.imshow("Human Tracker", frame)
            cv2.waitKey(1)
            return

        # ================= TRACK =================
        if found:

            x, y, w, h = target

            center = frame.shape[1] // 2
            face_center = x + w // 2
            error = face_center - center

            if error > CENTER_TOL:
                self.send("CW")

            elif error < -CENTER_TOL:
                self.send("CCW")

            else:
                if w < TOO_FAR:
                    self.send("FORWARD")
                else:
                    self.send("STOP")

        cv2.imshow("Human Tracker", frame)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = HumanTracker()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
