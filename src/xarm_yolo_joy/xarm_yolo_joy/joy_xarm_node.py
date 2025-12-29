#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

import threading
import time
import sys
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

# ===== xArm SDK PATH =====
XARM_SDK_PATH = "/home/robot/xArm-Python-SDK-master-main/xArm-Python-SDK-master-main"
sys.path.append(XARM_SDK_PATH)
from xarm.wrapper import XArmAPI


# ================= CONFIG =================
ROBOT_IP = "192.168.1.165"
MODEL_PATH = "/home/robot/xArm-Python-SDK-master-main/content/runs/detect/train10/weights/best.pt"

Z_PICK = 343
Z_GRAB = 162
Z_LIFT = 133

READY_POSE = dict(
    x=20, y=299, z=Z_PICK,
    roll=180, pitch=0, yaw=90
)

K_MM_PER_PIXEL = 0.3
PIXEL_TOL = 5
MAX_STEP = 10

OFFSET_X = -31.6
OFFSET_Y = 69.4


# ================= SHARED =================
shared = {
    "frame": None,
    "target": None,
    "exit": False
}
lock = threading.Lock()


# ================= NODE =================
class XArmYoloJoy(Node):

    def __init__(self):
        super().__init__("xarm_yolo_joy")

        # ROS
        self.sub = self.create_subscription(
            Joy, "/joy", self.joy_cb, 10
        )

        # xArm
        self.arm = XArmAPI(ROBOT_IP)
        self.arm.motion_enable(True)
        self.arm.set_mode(0)
        self.arm.set_state(0)

        # YOLO
        self.model = YOLO(MODEL_PATH)

        # State
        self.detect_running = False
        self.last_pick_pose = None   # <<< FULL 6D pose lưu ở đây

        # Threads
        threading.Thread(target=self.camera_thread, daemon=True).start()
        threading.Thread(target=self.robot_thread, daemon=True).start()
        threading.Thread(target=self.display_thread, daemon=True).start()

        self.get_logger().info("xArm YOLO Joy Node READY")

    # ================= JOY =================
    def joy_cb(self, msg: Joy):
        btn = msg.buttons

        # button 0: ready pose
        if btn[0] == 1:
            self.arm.set_position(**READY_POSE, speed=70, wait=True)

        # button 1: detect + grab
        if btn[1] == 1 and not self.detect_running:
            self.detect_running = True

        # button 2: return box
        if btn[2] == 1:
            self.return_box_to_last_pick()

    # ================= CAMERA THREAD =================
    def camera_thread(self):
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        pipeline.start(config)

        try:
            while not shared["exit"]:
                frames = pipeline.wait_for_frames()
                color = frames.get_color_frame()
                if not color:
                    continue

                frame = np.asanyarray(color.get_data())

                results = self.model.predict(frame, conf=0.5, verbose=False)[0]

                target = None
                if results.boxes:
                    b = results.boxes[0]
                    x1, y1, x2, y2 = map(int, b.xyxy[0])
                    cx = (x1 + x2) // 2
                    cy = (y1 + y2) // 2
                    target = (cx, cy, x1, y1, x2, y2)

                with lock:
                    shared["frame"] = frame
                    shared["target"] = target

        finally:
            pipeline.stop()

    # ================= ROBOT THREAD =================
    def robot_thread(self):
        while not shared["exit"]:
            if not self.detect_running:
                time.sleep(0.05)
                continue

            with lock:
                frame = shared["frame"]
                target = shared["target"]

            if frame is None or target is None:
                time.sleep(0.05)
                continue

            cx, cy, _, _, _, _ = target
            h, w, _ = frame.shape

            # ===== ALIGN LOGIC CỦA MÀY =====
            eu = cx - w // 2
            ev = cy - h // 2

            if abs(eu) <= PIXEL_TOL and abs(ev) <= PIXEL_TOL:
                _, pos = self.arm.get_position()

                target_x = pos[0] + OFFSET_X
                target_y = pos[1] + OFFSET_Y

                # move to pick
                self.arm.set_position(
                    x=target_x, y=target_y, z=Z_PICK,
                    roll=180, pitch=0, yaw=90,
                    wait=True
                )

                self.arm.open_lite6_gripper(True)
                time.sleep(0.3)
                self.arm.set_position(
                    x=target_x, y=target_y, z=Z_GRAB,
                    roll=180, pitch=0, yaw=90,
                    wait=True
                )
                
                self.arm.set_position(
                    x=target_x, y=target_y, z=Z_LIFT,
                    roll=180, pitch=0, yaw=90,
                    wait=True
                )
                self.arm.close_lite6_gripper(True)
                time.sleep(0.3)
                # ===== LƯU FULL POSE 6D =====
                self.last_pick_pose = {
                    "x": target_x,
                    "y": target_y,
                    "z": Z_GRAB,
                    "roll": 180,
                    "pitch": 0,
                    "yaw": 90
                }
                self.arm.set_position(
                    x=target_x, y=target_y, z=Z_PICK,
                    roll=180, pitch=0, yaw=90,
                    wait=True
                )
                # về home
                self.arm.move_gohome(wait=True)
                self.detect_running = False

            else:
                dx = max(min(-K_MM_PER_PIXEL * eu, MAX_STEP), -MAX_STEP)
                dy = max(min(-K_MM_PER_PIXEL * ev, MAX_STEP), -MAX_STEP)

                _, pos = self.arm.get_position()
                self.arm.set_position(
                    x=pos[0] - dx,
                    y=pos[1] + dy,
                    z=Z_PICK,
                    speed=80,
                    wait=True
                )

            time.sleep(0.02)

    # ================= RETURN BOX =================
    def return_box_to_last_pick(self):
        if self.last_pick_pose is None:
            self.get_logger().warn("No saved pick pose")
            return

        p = self.last_pick_pose

        # trên cao
        self.arm.set_position(
            x=p["x"], y=p["y"], z=Z_PICK,
            roll=p["roll"], pitch=p["pitch"], yaw=p["yaw"],
            wait=True
        )

        # hạ xuống
        self.arm.set_position(
            x=p["x"], y=p["y"], z=Z_GRAB,
            roll=p["roll"], pitch=p["pitch"], yaw=p["yaw"],
            wait=True
        )

        self.arm.open_lite6_gripper(True)
        time.sleep(0.3)

        # rút lên
        self.arm.set_position(
            x=p["x"], y=p["y"], z=Z_PICK,
            roll=p["roll"], pitch=p["pitch"], yaw=p["yaw"],
            wait=True
        )

        self.arm.move_gohome(wait=True)

    # ================= DISPLAY =================
    def display_thread(self):
        while rclpy.ok():
            with lock:
                frame = shared["frame"]
                target = shared["target"]

            if frame is not None:
                h, w, _ = frame.shape
                cv2.circle(frame, (w//2, h//2), 6, (255, 0, 0), -1)

                if target:
                    cx, cy, x1, y1, x2, y2 = target
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

                cv2.imshow("YOLO SERVO", frame)
                cv2.waitKey(1)

            time.sleep(0.01)


# ================= MAIN =================
def main():
    rclpy.init()
    node = XArmYoloJoy()
    rclpy.spin(node)
    shared["exit"] = True
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

