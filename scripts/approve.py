#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Standart, System and Third Party

# ROS
import rospy

# Robonomics communication
from robonomics_msgs.msg import Offer, Demand
from ethereum_common.msg import Address, UInt256
from ethereum_common.srv import Accounts, BlockNumber, Approve, ApproveRequest
from ipfs_common.msg import Multihash


if __name__ == '__main__':
    rospy.init_node('demand_publisher')
    rospy.loginfo('Launching...')

    rospy.wait_for_service('/eth/current_block')
    rospy.wait_for_service('/eth/accounts')
    rospy.wait_for_service('/eth/approve')

    accounts = rospy.ServiceProxy('/eth/accounts', Accounts)()
    rospy.loginfo(str(accounts)) # AIRA ethereum addresses
    
    req = ApproveRequest(spender=Address(address='0x7e384AD1FE06747594a6102EE5b377b273DC1225'), value=UInt256(uint256='100000000000000000'))
    approve = rospy.ServiceProxy('/eth/approve', Approve)(req)

    rospy.loginfo(approve)
    rospy.loginfo('Complete.')
