#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pyrealsense2 as rs
import numpy as np
import os
import sys
import time
import cv2
import threading
from ultralytics import YOLO

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))
from xarm.wrapper import XArmAPI


# ================= CONFIG =================
ROBOT_IP = '192.168.1.165'
MODEL_PATH = r"/home/robot/xArm-Python-SDK-master-main/content/runs/detect/train10/weights/best.pt"

CAM_ID = 1
CONF_THRES = 0.5

# Robot params
Z_PICK = 343
Z_GRAB = 162
Z_LIFT = 133
Z_MOVE_ON = 300
                                
SPEED_MOVE = 80
SPEED_DOWN = 80

# Visual servo params
K_MM_PER_PIXEL = 0.3
PIXEL_TOL = 5
MAX_STEP = 10

# Tool offset
OFFSET_X = -69.4
OFFSET_Y = 39.1

# Lock bbox
LOCK_DIST_THRES = 80


#destination cordinate for box placement

des_pos = [[-86.9, -307.8, Z_PICK, 180, 0, -90],
           [-11.5, -307.8, Z_PICK, 180, 0, -90],
           [75.4, -307.8, Z_PICK, 180, 0, -90],
           [139.9, -307.8, Z_PICK, 180, 0, -90],
           [-86.9, -237.8, Z_PICK, 180, 0, -90],
           [-11.5, -237.8, Z_PICK, 180, 0, -90],
           [75.4, -237.8, Z_PICK, 180, 0, -90],
           [139.9, -237.8, Z_PICK, 180, 0, -90]
           ]

max_boxes = len(des_pos)
current_box = 0

# ================= SHARED DATA =================
shared = {
    "target": None,          # (cx, cy, x1, y1, x2, y2)
    "locked_center": None,   # (cx, cy)
    "frame": None,
    "exit": False
}
lock = threading.Lock()


# ================= INIT ROBOT =================
arm = XArmAPI(ROBOT_IP)
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)

# arm.move_gohome(wait=True)
time.sleep(1)

arm.open_lite6_gripper(True)
time.sleep(0.5)
arm.stop_lite6_gripper(True)

arm.set_position(x=20, y=299, z=Z_PICK, speed=70, roll=180, pitch=0, yaw=90, wait=True)

def camera_thread():
    model = YOLO(MODEL_PATH)

    # ===== RealSense init =====
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    pipeline.start(config)

    try:
        while not shared["exit"]:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            frame = np.asanyarray(color_frame.get_data())

            h, w, _ = frame.shape
            cam_cx, cam_cy = w // 2, h // 2

            results = model.predict(frame, conf=CONF_THRES, verbose=False)[0]

            candidates = []
            for b in results.boxes:
                if model.names[int(b.cls[0])] != "box":
                    continue

                x1, y1, x2, y2 = map(int, b.xyxy[0])
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                candidates.append((cx, cy, x1, y1, x2, y2))

            with lock:
                target = None
                locked = shared["locked_center"]

                if candidates:
                    if locked is None:
                        target = min(
                            candidates,
                            key=lambda p: (p[0]-cam_cx)**2 + (p[1]-cam_cy)**2
                        )
                        shared["locked_center"] = (target[0], target[1])
                    else:
                        target = min(
                            candidates,
                            key=lambda p: (p[0]-locked[0])**2 + (p[1]-locked[1])**2
                        )
                        dist = ((target[0]-locked[0])**2 + (target[1]-locked[1])**2)**0.5
                        if dist < LOCK_DIST_THRES:
                            shared["locked_center"] = (target[0], target[1])
                        else:
                            shared["locked_center"] = None
                            target = None

                shared["target"] = target
                shared["frame"] = frame

    finally:
        pipeline.stop()



# # ================= CAMERA + YOLO THREAD =================
# def camera_thread():
#     model = YOLO(MODEL_PATH)
#     cap = cv2.VideoCapture(CAM_ID)

#     if not cap.isOpened():
#         print("Camera open failed")
#         shared["exit"] = True
#         return

#     while not shared["exit"]:
#         ret, frame = cap.read()
#         if not ret:
#             continue

#         h, w, _ = frame.shape
#         cam_cx, cam_cy = w // 2, h // 2

#         results = model.predict(frame, conf=CONF_THRES, verbose=False)[0]

#         candidates = []
#         for b in results.boxes:
#             if model.names[int(b.cls[0])] != "box":
#                 continue

#             x1, y1, x2, y2 = map(int, b.xyxy[0])
#             cx = (x1 + x2) // 2
#             cy = (y1 + y2) // 2
#             candidates.append((cx, cy, x1, y1, x2, y2))

#         with lock:
#             target = None
#             locked = shared["locked_center"]

#             if candidates:
#                 if locked is None:
#                     target = min(
#                         candidates,
#                         key=lambda p: (p[0]-cam_cx)**2 + (p[1]-cam_cy)**2
#                     )
#                     shared["locked_center"] = (target[0], target[1])
#                 else:
#                     target = min(
#                         candidates,
#                         key=lambda p: (p[0]-locked[0])**2 + (p[1]-locked[1])**2
#                     )
#                     dist = ((target[0]-locked[0])**2 + (target[1]-locked[1])**2)**0.5
#                     if dist < LOCK_DIST_THRES:
#                         shared["locked_center"] = (target[0], target[1])
#                     else:
#                         shared["locked_center"] = None
#                         target = None

#             shared["target"] = target
#             shared["frame"] = frame

#     cap.release()


# ================= ROBOT THREAD =================
def robot_thread():
    global current_box
    STATE_SERVO, STATE_GRAB, STATE_DONE = 0, 1, 2
    state = STATE_SERVO

    target_x = target_y = None

    while not shared["exit"]:
        with lock:
            target = shared["target"]

        if state == STATE_SERVO and target:
            cx, cy, _, _, _, _ = target

            eu = cx - h//2
            ev = cy - w//2

            dx = max(min(-K_MM_PER_PIXEL * eu, MAX_STEP), -MAX_STEP)
            dy = max(min(-K_MM_PER_PIXEL * ev, MAX_STEP), -MAX_STEP)

            # _, pos = arm.get_position()
            # arm.set_position(
            #     x = pos[0] - dy,
            #     y = pos[1] - dx,
            #     z = Z_PICK,
            #     speed = SPEED_MOVE,
            #     wait = True
            # )

            if abs(eu) <= PIXEL_TOL and abs(ev) <= PIXEL_TOL:
                _, pos = arm.get_position()
                target_x = pos[0] + OFFSET_X
                target_y = pos[1] + OFFSET_Y

                arm.set_position(x=target_x, y=target_y, z=Z_PICK,
                                 speed=SPEED_MOVE, wait=True)
                arm.set_position(x=target_x, y=target_y, z=Z_GRAB,
                                 speed=SPEED_DOWN, wait=True)
                state = STATE_GRAB
            else:
                dx = max(min(-K_MM_PER_PIXEL * eu, MAX_STEP), -MAX_STEP)
                dy = max(min(-K_MM_PER_PIXEL * ev, MAX_STEP), -MAX_STEP)

                _, pos = arm.get_position()
                arm.set_position(x=pos[0] - dx, y=pos[1] + dy,
                                 z=Z_PICK, speed=SPEED_MOVE, wait=True)

        elif state == STATE_GRAB:
            time.sleep(0.3)
            arm.open_lite6_gripper(True)
            time.sleep(0.3)
            
            arm.set_position(x=target_x, y=target_y, z=Z_LIFT,
                             speed=SPEED_MOVE, wait=True)
            state = STATE_DONE
            time.sleep(0.5)
            arm.close_lite6_gripper(True)
        elif state == STATE_DONE:
            time.sleep(0.5)
            arm.set_position(x=target_x, y=target_y, z=Z_MOVE_ON,
                             speed=SPEED_MOVE, wait=True)
            
            time.sleep(0.5)
            # arm.set_position(x = 0, y = -237, z = Z_PICK, roll= 180, pitch= 0 , yaw = -90, speed= SPEED_MOVE, wait= True)
            # time.sleep(0.5)
            # arm.set_position(x=des_pos[current_box][0], y=des_pos[current_box][1], z=Z_PICK,
            #                  roll=des_pos[current_box][3], pitch=des_pos[current_box][4], yaw=des_pos[current_box][5],
            #                  speed=SPEED_MOVE, wait=True)
            # time.sleep(0.5)
            # arm.set_position(x=des_pos[current_box][0], y=des_pos[current_box][1], z=Z_LIFT,
            #                  roll=des_pos[current_box][3], pitch=des_pos[current_box][4], yaw=des_pos[current_box][5],
            #                  speed=SPEED_MOVE, wait=True)
            # arm.open_lite6_gripper(True)
            # time.sleep(0.5)
            # arm.stop_lite6_gripper(True)
            # time.sleep(0.5)
            # arm.set_position(x=des_pos[current_box][0], y=des_pos[current_box][1], z=Z_MOVE_ON,
            #                  roll=des_pos[current_box][3], pitch=des_pos[current_box][4], yaw=des_pos[current_box][5],
            #                  speed=SPEED_MOVE, wait=True)
            # current_box += 1
            # state = STATE_SERVO
            # time.sleep(0.5)
            arm.set_position(x=252, y=-24, z=Z_PICK,roll= 180, pitch= 0 , yaw = 0, speed=70, wait=True)
            arm.move_gohome(wait=True)
            time.sleep(1)
            arm.open_lite6_gripper(True)
            time.sleep(0.5)
            arm.stop_lite6_gripper(True)
            state = STATE_SERVO
            arm.set_position(x=20, y=299, z=Z_PICK, speed=70, roll=180, pitch=0, yaw=90, wait=True)
        time.sleep(0.02)


# ================= START THREADS =================
t_cam = threading.Thread(target=camera_thread, daemon=True)
t_robot = threading.Thread(target=robot_thread, daemon=True)

t_cam.start()
t_robot.start()

# ================= MAIN DISPLAY LOOP =================
while True:
    with lock:
        frame = shared["frame"]
        target = shared["target"]

    if frame is not None:
        h, w, _ = frame.shape
        cv2.circle(frame, (w//2, h//2), 6, (255,0,0), -1)

        if target:
            cx, cy, x1, y1, x2, y2 = target
            cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 2)
            cv2.circle(frame, (cx,cy), 5, (0,0,255), -1)

        cv2.imshow("Visual Servo Threaded", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        shared["exit"] = True
        break

cv2.destroyAllWindows()
arm.disconnect()
print(">>> Program exited cleanly")
 