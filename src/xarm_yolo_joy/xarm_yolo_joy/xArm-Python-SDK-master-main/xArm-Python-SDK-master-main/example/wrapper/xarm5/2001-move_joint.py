#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Description: Move Joint
"""

import os
import sys
import time
import math

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI


#######################################################
"""
Just for test example
"""
if len(sys.argv) >= 2:
    ip = sys.argv[1]
else:
    try:
        from configparser import ConfigParser
        parser = ConfigParser()
        parser.read('../robot.conf')
        ip = parser.get('xArm', 'ip')
    except:
        ip = '192.168.1.165'
        if not ip:
            print('input error, exit')
            sys.exit(1)
########################################################


arm = XArmAPI(ip)
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)

arm.move_gohome(wait=True)




arm.set_position(x=261., y=0, z=258.3, roll=180, pitch=0, yaw=0, speed=50, wait=True)
time.sleep(0.1)
arm.set_position(x=104.8, y=-105.6, z=148.8, roll=180, pitch=0, yaw=0, speed=50, wait=True)
time.sleep(0.1)
arm.set_position(x=104.8, y=-105.6, z=138.8, roll=180, pitch=0, yaw=0, speed=50, wait=True)
time.sleep(0.1)
arm.set_position(x=104.8, y=-105.6, z=148.8, roll=180, pitch=0, yaw=0, speed=50, wait=True)
time.sleep(0.1)
arm.set_position(x=104.8000, y=-35.6000, z=148.8000, roll=180, pitch=0, yaw=0, speed=50, wait=True)
time.sleep(0.1)
arm.set_position(x=104.8000, y=-35.6000, z=138.8000, roll=180, pitch=0, yaw=0, speed=50, wait=True)
time.sleep(0.1)
arm.set_position(x=145.2145, y=34.4000, z=138.8000, roll=180, pitch=0, yaw=0, speed=50, wait=True)
time.sleep(0.1)
arm.set_position(x=226.0436, y=34.4000, z=138.8000, roll=180, pitch=0, yaw=0, speed=50, wait=True)
time.sleep(0.1)
arm.set_position(x=266.4581, y=-35.6000, z=138.8000, roll=180, pitch=0, yaw=0, speed=50, wait=True)
time.sleep(0.1)
arm.set_position(x=226.0436, y=-105.6000, z=138.8000, roll=180, pitch=0, yaw=0, speed=50, wait=True)
time.sleep(0.1)
arm.set_position(x=145.2145, y=-105.6000, z=138.8000, roll=180, pitch=0, yaw=0, speed=50, wait=True)
time.sleep(0.1)
arm.set_position(x=145.2145, y=-105.6000, z=148.8000, roll=180, pitch=0, yaw=0, speed=50, wait=True)
time.sleep(0.1)
arm.set_position(x=266.4581, y=-35.6000, z=148.8000, roll=180, pitch=0, yaw=0, speed=50, wait=True)
time.sleep(0.1)
arm.set_position(x=266.4581, y=-35.6000, z=138.8000, roll=180, pitch=0, yaw=0, speed=50, wait=True)
time.sleep(0.1)
arm.set_position(x=185.6290, y=34.4000, z=138.8000, roll=180, pitch=0, yaw=0, speed=50, wait=True)
time.sleep(0.1)
arm.set_position(x=226.0436, y=-105.6000, z=138.8000, roll=180, pitch=0, yaw=0, speed=50, wait=True)
time.sleep(0.1)
arm.set_position(x=104.8000, y=-35.6000, z=138.8000, roll=180, pitch=0, yaw=0, speed=50, wait=True)
time.sleep(0.1)
arm.set_position(x=145.2145, y=-105.6000, z=138.8000, roll=180, pitch=0, yaw=0, speed=50, wait=True)
time.sleep(0.1)
arm.set_position(x=266.4581, y=-35.6000, z=138.8000, roll=180, pitch=0, yaw=0, speed=50, wait=True)
time.sleep(0.1)
arm.set_position(x=104.8000, y=-35.6000, z=138.8000, roll=180, pitch=0, yaw=0, speed=50, wait=True)
time.sleep(0.1)
arm.set_position(x=185.6290, y=34.4000, z=138.8000, roll=180, pitch=0, yaw=0, speed=50, wait=True)
time.sleep(0.1)
arm.set_position(x=145.2145, y=-105.6000, z=138.8000, roll=180, pitch=0, yaw=0, speed=50, wait=True)
time.sleep(0.1)
arm.set_position(x=226.0436, y=34.4000, z=138.8000, roll=180, pitch=0, yaw=0, speed=50, wait=True)
time.sleep(0.1)
arm.set_position(x=185.6290, y=-105.6000, z=138.8000, roll=180, pitch=0, yaw=0, speed=50, wait=True)
time.sleep(0.1)
arm.set_position(x=145.2145, y=34.4000, z=138.8000, roll=180, pitch=0, yaw=0, speed=50, wait=True)
time.sleep(0.1)
arm.set_position(x=226.0436, y=-105.6000, z=138.8000, roll=180, pitch=0, yaw=0, speed=50, wait=True)
time.sleep(0.1)
arm.set_position(x=226.0436, y=-105.6000, z=148.8000, roll=180, pitch=0, yaw=0, speed=50, wait=True)
time.sleep(0.1)
arm.set_position(x=104.8000, y=-105.6000, z=148.8000, roll=180, pitch=0, yaw=0, speed=50, wait=True)
time.sleep(0.1)
arm.set_position(x=104.8000, y=-105.6000, z=138.8000, roll=180, pitch=0, yaw=0, speed=50, wait=True)
time.sleep(0.1)
arm.set_position(x=104.8, y=-105.6, z=148.8, roll=180, pitch=0, yaw=0, speed=50, wait=True)
time.sleep(0.1)
arm.set_position(x=261., y=0, z=258.3, roll=-180, pitch=0, yaw=0, speed=50, wait=True)
time.sleep(0.1)
arm.disconnect()
