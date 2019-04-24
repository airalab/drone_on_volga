#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import serial
import time
from datetime import datetime


try:
    device = sys.argv[1]
except IndexError:
    device = '/dev/ttyUSB0'
print('waspmote starting with ' + device + '...')
waspmote = serial.Serial(device, 115200)
while True:
    try:
        data = waspmote.readline().decode('ascii', 'backslashreplace') # read '\n' terminated line
        print(str(datetime.now()) + ' ' + data, end='')
        with open('/root/waspmote.log', 'a+') as log:
            log.write(str(datetime.now().timestamp()) + ' ' + data)
    except KeyboardInterrupt:
        exit(0)
    except Exception as e:
        print('reading error: ', e)
