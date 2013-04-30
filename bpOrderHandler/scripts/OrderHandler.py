#!/usr/bin/env python
import roslib; roslib.load_manifest('bpOrderHandler')
from bpMessy2Controller.srv import *
from bpMsgs.msg import order
from bzrlib.switch import switch
from rospy.timer import sleep
from rospy.timer import Timer
from std_msgs.msg import String
import rospy
import threading
import time
import datetime

def orderHandler():
    rospy.init_node('OrderHandler')

    print "OrderHandler started"

    rospy.spin()
    
    print "OrderHandler shutting down..."

if __name__ == "__main__":
    orderHandler()



