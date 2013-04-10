#!/usr/bin/env python
import roslib; roslib.load_manifest('bpMessy2Controller')
import rospy
from std_msgs.msg import String
from bpMsgs.msg import order
from bpMessy2Controller.srv import *

import requests
from xml.dom.minidom import parseString
from nose import case
from bzrlib.switch import switch
    
def getOrderList(max_orders):
    
    ret = []
    r = requests.get("http://192.168.10.100/orders/",  stream = True)
    dom = parseString(r.text)
    n = dom.getElementsByTagName('order');
    number_of_orders = max_orders
    if number_of_orders > n.length:
        number_of_orders = n.length
    for i in range(0, number_of_orders):
        line = dom.getElementsByTagName('order')[i].toxml()
        requestString = line.replace('<order>','').replace('</order>','')
        r = requests.get(requestString,  stream = True)
        orderXml = parseString(r.text)
        orderData = order()
        xmlRed = orderXml.getElementsByTagName('red')[0].toxml()
        xmlBlue = orderXml.getElementsByTagName('blue')[0].toxml()
        xmlYellow = orderXml.getElementsByTagName('yellow')[0].toxml()
        xmlTime = orderXml.getElementsByTagName('time')[0].toxml()
        xmlStatus = orderXml.getElementsByTagName('status')[0].toxml()
        orderData.order_id = int(requestString.replace('http://192.168.10.100/orders/ord_',''))
        if xmlStatus.replace('<status>','').replace('</status>','') == "ready":
            orderData.status = orderData.STATUS_READY
        elif xmlStatus.replace('<status>','').replace('</status>','') == "taken":
            orderData.status = orderData.STATUS_TAKEN
        else:
            orderData.status = orderData.STATUS_UNKNOWN
        orderData.time = xmlTime.replace('<time>','').replace('</time>','')
        orderData.red_bricks = int(xmlRed.replace('<red>','').replace('</red>',''))
        orderData.blue_bricks = int(xmlBlue.replace('<blue>','').replace('</blue>',''))
        orderData.yellow_bricks = int(xmlYellow.replace('<yellow>','').replace('</yellow>',''))
        ret.append(orderData)
        
    return ret

def takeOrder(order):
    order = str(order).zfill(5)
    order = "http://192.168.10.100/orders/ord_" + order
    print order
    r = requests.put(order,  stream = True)
    dom = parseString(r.text)
    lines = dom.getElementsByTagName('ticket')[0].toxml()
    ticket = lines.replace('<ticket>ord_','').replace('</ticket>','')
      
    return ticket

def finishOrder(order, ticket):
    order = str(order).zfill(5)
    order = "http://192.168.10.100/orders/ord_" + order + "/ord_" + ticket
    r = requests.delete(order,  stream = True)
    print r.status_code
    
    return

def order_service_handler(req):
    print "Order number: %d"%(req.command_number)
    ret = commandResponse()
    if req.command_number == req.GET_ORDERS:
        print "Get Orders"
        ret.orders = getOrderList(req.max_orders)
    elif req.command_number == req.TAKE_ORDER:
        print "Take order"
        ret.order_ticket = takeOrder(req.order_id)
    elif req.order_ticket == req.FINISH_ORDER:
        print "Finish order"
        finishOrder(req.order_id, req.order_ticket)
        
    return ret

def order_service():
    rospy.init_node('Messy2Controller')
    s = rospy.Service('order_service', command, order_service_handler)
    print "Messy2Controller started and order_service started"
    rospy.spin()

if __name__ == "__main__":
    order_service()



