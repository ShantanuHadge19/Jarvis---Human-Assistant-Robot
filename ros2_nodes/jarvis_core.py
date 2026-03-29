#!/usr/bin/env python3
import cmd

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import serial
import subprocess
import signal
import os
import socket
import threading
import time
import lgpio
from rclpy.logging import LoggingSeverity

UDP_PORT = 5005


class JarvisCore(Node):
    def __init__(self):
        super().__init__('jarvis_core')
        self.get_logger().set_level(LoggingSeverity.INFO)

        # ---------- SERIAL ----------
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.01)
            self.get_logger().info("✅ ESP serial connected")
        except Exception as e:
            self.get_logger().error(f"❌ Serial error: {e}")
            self.ser = None

        # ---------- ROS ----------
        self.create_subscription(String, 'esp_tx', self.esp_tx_cb, 10)
        self.esp_tx_pub = self.create_publisher(String, 'esp_tx', 10)
        self.obstacle_pub = self.create_publisher(String, 'obstacle_status', 10)

        # ---------- STATE ----------
        self.current_mode = "IDLE"
        self.human_proc = None
        self.spot_proc = None
        self.pick_proc = None
        self.idle_proc = None
        self.last_idle_cmd = None
        self.remote_active = False

        self.busy = False
        self.busy_until = 0


        self.create_timer(0.1, self.line_follow)

        # =====================================================
        # GLOBAL ULTRASONIC SAFETY (STABLE VERSION)
        # =====================================================
        self.TRIG_PIN = 23
        self.ECHO_PIN = 24
        self.STOP_DISTANCE_CM = 25.0

        self.chip = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.chip, self.TRIG_PIN)
        lgpio.gpio_claim_input(self.chip, self.ECHO_PIN)

        self.obstacle_detected = False

        #ir sensor

        self.LEFT = 17
        self.CENTER = 27
        self.RIGHT = 22

        lgpio.gpio_claim_input(self.chip, self.LEFT)
        lgpio.gpio_claim_input(self.chip, self.CENTER)
        lgpio.gpio_claim_input(self.chip, self.RIGHT)

        # Stability counters
        self.obstacle_counter = 0
        self.clear_counter = 0

        self.create_timer(0.1, self.ultrasonic_check)
        self.create_timer(0.1, self.repeat_idle)

        # ---------- START IDLE ----------
        self.start_idle()

        # ---------- UDP ----------
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.bind(('', UDP_PORT))

        threading.Thread(target=self.udp_listener, daemon=True).start()
        threading.Thread(target=self.serial_listener, daemon=True).start()

        self.send_mode("IDLE")

    # =====================================================
    # ULTRASONIC CHECK (STABLE)
    # =====================================================
    def ultrasonic_check(self):

        lgpio.gpio_write(self.chip, self.TRIG_PIN, 0)
        time.sleep(0.002)

        lgpio.gpio_write(self.chip, self.TRIG_PIN, 1)
        time.sleep(0.00001)
        lgpio.gpio_write(self.chip, self.TRIG_PIN, 0)

        start_time = time.time()
        timeout = start_time + 0.04

        while lgpio.gpio_read(self.chip, self.ECHO_PIN) == 0:
            start_time = time.time()
            if start_time > timeout:
                return

        stop_time = time.time()

        while lgpio.gpio_read(self.chip, self.ECHO_PIN) == 1:
            stop_time = time.time()
            if stop_time > timeout:
                break

        elapsed = stop_time - start_time
        distance = (elapsed * 34300) / 2

        # ---- Stability Logic ----
        if distance <= self.STOP_DISTANCE_CM:
            self.obstacle_counter += 1
            self.clear_counter = 0
        else:
            self.clear_counter += 1
            self.obstacle_counter = 0

        if self.obstacle_counter >= 3:
            if not self.obstacle_detected:
                self.get_logger().warn(
                    f"🚨 OBSTACLE DETECTED at {round(distance,1)} cm"
                )
            self.obstacle_detected = True
            self.obstacle_pub.publish(String(data="OBSTACLE"))

        elif self.clear_counter >= 5:
            self.obstacle_detected = False
            self.obstacle_pub.publish(String(data="CLEAR"))

    # =====================================================
    # PROCESS HELPERS
    # =====================================================
    def start_proc(self, cmd, name):
        self.get_logger().info(f"▶ Starting {name}")
        return subprocess.Popen(cmd, preexec_fn=os.setsid)

    def stop_proc(self, proc, name):
        if not proc:
            return None
        self.get_logger().info(f"⏹ Stopping {name}")
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=2)
        except Exception:
            pass
        return None




    def line_follow(self):

        if self.current_mode != "GESTURE":
            return

        now = time.time()

        # turn complete → STOP bhejo
        if self.busy and now >= self.busy_until:
            self.esp_tx_pub.publish(String(data="GESTURE:STOP"))
            self.busy = False
            return

        # if currently turning → wait until done
        if self.busy and now < self.busy_until:
            return

        # reset busy
        self.busy = False

        l = lgpio.gpio_read(self.chip, self.LEFT)
        c = lgpio.gpio_read(self.chip, self.CENTER)
        r = lgpio.gpio_read(self.chip, self.RIGHT)

        #self.get_logger().info(f"L:{l} C:{c} R:{r}", throttle_duration_sec=0.5)

        # ========= NO LINE =========
        if l == 1 and c == 1 and r == 1:
            self.get_logger().info("NONE (ALL BLACK)")
            self.esp_tx_pub.publish(String(data="GESTURE:STOP"))
            return

        if l == 0 and c == 0 and r == 0:
            #self.get_logger().info("NONE (NO LINE)")
            return

        # ========= FORWARD =========
        if l == 0 and c == 1 and r == 0:
            self.get_logger().info("FORWARD")
            self.esp_tx_pub.publish(String(data="GESTURE:FORWARD"))
            return

    # ========= LEFT PULSE =========
        if l == 1 and c == 0 and r == 0:
            self.esp_tx_pub.publish(String(data="GESTURE:CCW"))
            self.get_logger().info("LEFT")
            self.busy = True
            self.busy_until = now + 0.3
            return

        # ========= RIGHT PULSE =========
        if l == 0 and c == 0 and r == 1:
            self.esp_tx_pub.publish(String(data="GESTURE:CW"))
            self.get_logger().info("RIGHT")
            self.busy = True
            self.busy_until = now + 0.3
            return
        
       

    

    



        

    # =====================================================
    # IDLE CONTROL
    # =====================================================
    def start_idle(self):

        if self.spot_proc or self.human_proc or self.pick_proc:
            return
        
        if self.idle_proc:
            return
        self.idle_proc = self.start_proc(
            ["ros2", "run", "jarvis1", "idle_behavior"],
            "idle_behavior"
        )
        time.sleep(0.5)

    def stop_idle(self):
        if not self.idle_proc:
            return

        self.get_logger().info("⛔ FORCE STOP IDLE")

        try:
            os.killpg(os.getpgid(self.idle_proc.pid), signal.SIGKILL)
        except Exception:
            pass

        self.idle_proc = None
        time.sleep(0.3)


    # =====================================================
    # HUMAN TRACK MODE
    # =====================================================

    def start_human_track(self, target="person"):

        if self.human_proc:
            return

        self.stop_idle()

        self.human_proc = self.start_proc(
            ["ros2", "run", "jarvis1", "human_tracker_node"],
            "human_tracker"
        )

        self.send_mode("GESTURE")
        time.sleep(0.5)

        self.esp_tx_pub.publish(String(data="ARM:ELBOW:30"))
        self.esp_tx_pub.publish(String(data="ARM:BEND:100"))
        self.esp_tx_pub.publish(String(data="ARM:SHOULDER:40"))
        self.esp_tx_pub.publish(String(data="ARM:GRIP:OPEN"))


    def stop_human_track(self):

        if not self.human_proc:
            return

        self.human_proc = self.stop_proc(self.human_proc, "human_tracker")

        if not self.remote_active:   # ✅ IMPORTANT
            self.start_idle()
            self.send_mode("IDLE")



    def start_spot_object(self, target):

        if self.spot_proc:
            return

        self.get_logger().info(f"🔍 Starting spot object: {target}")

        self.last_idle_cmd = None   # 🔥 ADD THIS
        self.current_mode = "GESTURE" 
        self.stop_idle()
        
        time.sleep(0.5)   

        self.spot_proc = self.start_proc(
            ["ros2", "run", "jarvis1", "spot_the_object", "--ros-args", "--param", f"object:={target}"],
            "spot_the_object"
        )

        self.send_mode("GESTURE")   # or make new mode if you want

        time.sleep(0.2)   # 🔥 allow node to start

        # ================= ARM INITIAL POSITION =================
        self.esp_tx_pub.publish(String(data="ARM:ELBOW:20"))
        self.esp_tx_pub.publish(String(data="ARM:BEND:72"))
        self.esp_tx_pub.publish(String(data="ARM:SHOULDER:60"))
        self.esp_tx_pub.publish(String(data="ARM:GRIP:OPEN"))

    def stop_spot_object(self):

        if not self.spot_proc:
            return

        self.spot_proc = self.stop_proc(self.spot_proc, "spot_the_object")

        if not self.remote_active:
            self.start_idle()
            self.send_mode("IDLE")


    # =====================================================
    # PICK THE CUP MODE
    # =====================================================

    def start_pick(self):
        self.last_idle_cmd = None   # 🔥 VERY IMPORTANT
        if self.pick_proc:
            return

        self.stop_idle()

        time.sleep(0.5)

        self.pick_proc = self.start_proc(
            ["ros2", "run", "jarvis1", "pick_the_cup"],
            "pick_the_cup"
        )

        self.send_mode("GESTURE")

        time.sleep(0.2)

        self.esp_tx_pub.publish(String(data="ARM:ELBOW:20"))
        self.esp_tx_pub.publish(String(data="ARM:BEND:80"))
        self.esp_tx_pub.publish(String(data="ARM:SHOULDER:60"))
        self.esp_tx_pub.publish(String(data="ARM:GRIP:OPEN"))

    def stop_pick(self):
        self.pick_proc = self.stop_proc(self.pick_proc, "pick_the_cup")
        
        if not self.remote_active:   # ✅ IMPORTANT
            self.start_idle()
            self.send_mode("IDLE")


    # =====================================================
    # REMOTE MODE
    # =====================================================

    def start_remote(self):
        
        if self.remote_active:
            return

        self.get_logger().info("🎮 REMOTE MODE START")

        self.remote_active = True

        self.stop_idle()
        self.stop_human_track()
        self.stop_pick()

        
        self.send_mode("REMOTE")


    def stop_remote(self):
        if not self.remote_active:
            return

        self.get_logger().info("🛑 REMOTE MODE STOP")

        self.remote_active = False

        self.start_idle()
        self.send_mode("IDLE")

    # =====================================================
    # MODE HANDLING
    # =====================================================
    def send_mode(self, mode):
        self.current_mode = mode
        self.esp_tx_pub.publish(String(data="GESTURE:STOP"))
        self.esp_tx_pub.publish(String(data=f"MODE:{mode}"))
        self.get_logger().info(f"🔁 MODE → {mode}")

    # =====================================================
    # SERIAL LISTENER (UNCHANGED)
    # =====================================================
    def serial_listener(self):
        while rclpy.ok():
            if not self.ser:
                continue
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode(errors='ignore').strip()
                    print("FROM ESP:", line)

                    if line.startswith("CMD:FOLLOW_"):
                        self.start_human_track(
                            line.replace("CMD:FOLLOW_", "").lower())

                    elif line == "CMD:STOP_FOLLOW":
                        self.stop_human_track()

                    elif line == "CMD:GESTURE_ON":
                        self.stop_idle()
                        self.send_mode("GESTURE")

                    elif line == "CMD:GESTURE_OFF":
                        self.start_idle()
                        self.send_mode("IDLE")

                    elif line.startswith("CMD:FIND:"):
                        target = line.replace("CMD:FIND:", "").lower()
                        self.start_spot_object(target)

                    elif line == "CMD:STOP_FIND":
                        self.stop_spot_object()

                    elif line == "CMD:PICK_THE_CUP":
                        self.start_pick()

                    elif line == "CMD:PUT_THE_CUP":
                        self.stop_pick()

                    elif line == "CMD:REMOTE_ON":
                        self.start_remote()

                    elif line == "CMD:REMOTE_OFF":
                        self.stop_remote()

            except Exception as e:
                self.get_logger().error(f"Serial error: {e}")

    # =====================================================
    # UDP LISTENER (UNCHANGED)
    # =====================================================
    def udp_listener(self):
        while rclpy.ok():
            try:
                data, addr = self.udp_sock.recvfrom(1024)
                msg = data.decode().strip()

                if self.current_mode == "GESTURE" and not self.remote_active:
                    self.get_logger().info(f"📡 UDP → {msg}")

                # Directly publish everything coming from laptop
                if not self.remote_active:
                    if msg.startswith("GESTURE:") or msg.startswith("ARM:"):
                        self.esp_tx_pub.publish(String(data=msg))

            except Exception as e:
                self.get_logger().error(f"UDP error: {e}")

    # =====================================================
    # ESP OUTPUT GATE (UPDATED FOR ARM SUPPORT)
    # =====================================================
    def esp_tx_cb(self, msg):
        if not self.ser:
            return

        data = msg.data

        # -----------------------------
        # 1️⃣ ARM COMMANDS ALWAYS PASS
        # -----------------------------
        if data.startswith("ARM:"):
            self.ser.write((data + "\n").encode())
            return

        

        # -----------------------------
        # 3️⃣ MODE
        # -----------------------------
        if data.startswith("MODE:"):
            self.ser.write((data + "\n").encode())
            return

        # -----------------------------
        # 4️⃣ IDLE WHEEL MAPPING
        # -----------------------------
        if data.startswith("IDLE:"):
            self.last_idle_cmd = data
            clean = data.replace("IDLE:", "")
            mapping = {
                "FORWARD": "GESTURE:FORWARD",
                "BACKWARD": "GESTURE:BACKWARD",
                "LEFT": "GESTURE:CCW",
                "RIGHT": "GESTURE:CW",
                "STOP": "GESTURE:STOP"
            }
            if clean in mapping:
                self.ser.write((mapping[clean] + "\n").encode())
            return

        # -----------------------------
        # 5️⃣ NORMAL GESTURE FORWARDING
        # -----------------------------
        if data.startswith("GESTURE:") and self.current_mode == "GESTURE":
            self.ser.write((data + "\n").encode())
            return

    # =====================================================
    def destroy_node(self):
        print("🔴 Shutting down Jarvis Core")

        self.stop_idle()
        self.stop_human_track()
        self.spot_proc = self.stop_proc(self.spot_proc, "spot_object")
        self.pick_proc = self.stop_proc(self.pick_proc, "pick_the_cup")

        if self.ser:
            self.ser.close()

        try:
            self.udp_sock.close()
        except:
            pass

        try:
            lgpio.gpiochip_close(self.chip)
        except:
            pass

        super().destroy_node()

    def repeat_idle(self):

        if self.current_mode != "IDLE":
           return

        if self.spot_proc:   # 🔥 ADD THIS
            return

        if not self.last_idle_cmd:
            return

        clean = self.last_idle_cmd.replace("IDLE:", "")

        mapping = {
            "FORWARD": "GESTURE:FORWARD",
            "BACKWARD": "GESTURE:BACKWARD",
            "LEFT": "GESTURE:CCW",
            "RIGHT": "GESTURE:CW",
            "STOP": "GESTURE:STOP"
        }

        if clean in mapping:
            self.ser.write((mapping[clean] + "\n").encode())


def main():
    rclpy.init()
    node = JarvisCore()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 Ctrl+C pressed")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main() 
