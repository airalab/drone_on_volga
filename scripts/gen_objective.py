# -*- coding: utf-8 -*-

import rosbag, rospy
from std_msgs.msg import Bool, String, Duration

days_to_sec = lambda days: days*24*60*60

data = { # key - topic, value - message
        '/virtual': Bool(data=True),
        '/expiration': Duration(data=rospy.Duration(days_to_sec(31))),
        '/sensors': String(data='Temperature, pH, Dissolved Oxygen, Conductivity'),
        '/waypoints': String(data='QmcS1uxwzxvQmoMbNZRdKxTShU4wBm97NKYFxKeQxmFJTy')
        }

with rosbag.Bag('./objective.bag', 'w') as bag:
    for topic, msg in data.items():
        bag.write(topic, msg)
