#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Standart, System and Third Party

# ROS
import rospy

# Robonomics communication
from robonomics_msgs.msg import Offer, Demand
from ethereum_common.msg import Address, UInt256
from ethereum_common.srv import Accounts, BlockNumber
from ipfs_common.msg import Multihash


MODEL = None or 'QmYb81uDNDHCnu9EZtYV4eoBDKRBAwJeNy1LT3p5Zbc357'
OBJECTIVE = None or 'Qmea8XkcSXmvLDKES7D886pfimsWh9Vjh1ZJsoHm9MWG4C'
TOKEN = None or '0xC02aaA39b223FE8D0A0e5C4F27eAD9083C756Cc2' # WETH
PRICE = None or 100000000000000000 # 0.1 WETH
LIFETIME =  None or 100

if __name__ == '__main__':
    rospy.init_node('demand_publisher')
    rospy.loginfo('Launching...')

    rospy.wait_for_service('/eth/current_block')
    rospy.wait_for_service('/eth/accounts')

    accounts = rospy.ServiceProxy('/eth/accounts', Accounts)()
    rospy.loginfo(str(accounts)) # AIRA ethereum addresses

    signing_demand = rospy.Publisher('/liability/infochan/eth/signing/demand', Demand, queue_size=128)
    signing_offer = rospy.Publisher('/liability/infochan/eth/signing/offer', Offer, queue_size=128)

    rospy.loginfo('Node launched')

    model = MODEL or input('Model IPFS hash: ')
    objective = OBJECTIVE or input('Objective IPFS hash: ')
    token = TOKEN or input('Token: ')
    price = PRICE or input('Price: ')
    lifetime = LIFETIME or input('Demand lifetime: ')

    deadline = str(rospy.ServiceProxy('/eth/current_block', BlockNumber)().number + int(lifetime))

    rospy.loginfo('Making demand...')

    demand = Demand()
    demand.model = Multihash()
    demand.model.multihash = model
    demand.objective = Multihash()
    demand.objective.multihash = objective
    demand.lighthouse = Address()
    demand.lighthouse.address = '0xD40AC7F1e5401e03D00F5aeC1779D8e5Af4CF9f1'
    demand.token = Address()
    demand.token.address = token
    demand.cost = UInt256()
    demand.cost.uint256 = str(price)
    demand.validatorFee = UInt256()
    demand.validatorFee.uint256 = '0'
    demand.validator = Address()
    demand.validator.address = '0x0000000000000000000000000000000000000000'
    demand.deadline = UInt256()
    demand.deadline.uint256 = deadline

    signing_demand.publish(demand)
    rospy.loginfo(demand)

    rospy.loginfo('Complete.')
