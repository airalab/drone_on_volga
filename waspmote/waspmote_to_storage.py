#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import serial
from datetime import datetime


try:
    device = sys.argv[1]
except IndexError:
    device = '/dev/ttyUSB0'
print('starting with ' + device + '...')
waspmote = serial.Serial(device, 115200)
try:
    data = waspmote.readline().decode('ascii', 'backslashreplace') # read '\n' terminated line
    print(data)
    with open('/root/waspmote.log', 'a+') as log:
        log.write(str(datetime.now().timestamp()) + ' ' + data)
except:
    print('reading error')
