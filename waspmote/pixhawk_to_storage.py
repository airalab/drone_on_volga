#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import serial
import time
from datetime import datetime
from pymavlink import mavutil


try:
    device = sys.argv[1]
except IndexError:
    device = '/dev/ttyUSB0'
print('pixhawk starting with ' + device + '...')
px = mavutil.mavlink_connection(device, 57600)
px.wait_heartbeat()
px.mav.request_data_stream_send(px.target_system, px.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 1, 1)
while True:
    try:
        msg = px.recv_match(blocking=True)
        if msg.get_type() == 'GLOBAL_POSITION_INT':
            print(str(datetime.now()) + ' ' + str(msg))
            with open('/root/px.log', 'a+') as log:
                log.write(str(datetime.now().timestamp()) + ' ' + str(msg))
    except KeyboardInterrupt:
        exit(0)
    except Exception as e:
        print('reading error: ', e)
