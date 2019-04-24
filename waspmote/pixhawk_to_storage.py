#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import serial
from datetime import datetime


try:
    device = sys.argv[1]
except IndexError:
    device = '/dev/ttyUSB0'
print('pixhawk read and store starting with ' + device + '...')
pixhawk = serial.Serial(device, 115200)
try:
    data = pixhawk.readline()
    print(data)
except:
    print('reading error')
