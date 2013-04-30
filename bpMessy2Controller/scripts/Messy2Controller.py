#!/usr/bin/env python
import roslib; roslib.load_manifest('bpMessy2Controller')
from bpMessy2Controller.srv import *
from bpMsgs.msg import order
from bzrlib.switch import switch
from nose import case
from rospy.timer import sleep
from std_msgs.msg import String
from xml.dom.minidom import parseString
from xml.dom.minidom import Document
import requests
import rospy
import threading
import time
import datetime

def putRequest(url):
    try:
        ret = requests.put(url,  stream = True, timeout=2)
    except requests.exceptions.RequestException:
        print "PutRequest failed: failed to connect to server"
        return -10, ""
    
    return ret.status_code, ret.text

def deleteRequest(url):
    try:
        ret = requests.delete(url,  stream = True, timeout=2)
    except requests.exceptions.RequestException:
        print "DeleteRequest failed: failed to connect to server"
        return -10
    
    return ret.status_code

def getRequest(url):
    try:
        ret = requests.get(url,  stream = True, timeout=2)
    except requests.exceptions.RequestException:
        print "GetRequest failed: failed to connect to server"
        return -10, ""
    
    return ret.status_code, ret.text

def postRequest(url, Data):
    try:
        ret = requests.post(url, data=Data, stream = True, timeout=2)
    except requests.exceptions.RequestException:
        print "Postfailed: failed to connect to server"
        return -10
    
    return ret.status_code
    
def takeOrder(order):
    succes = False
    ticket = ''
    order = str(order).zfill(5)
    orderUrl = serverAddress + "/orders/ord_" + order
    [status_code, response] = putRequest(orderUrl)
    if status_code == 200:
        succes = True
        dom = parseString(response)
        lines = dom.getElementsByTagName('ticket')[0].toxml()
        ticket = lines.replace('<ticket>ord_','').replace('</ticket>','')
        # Log to server
        placeLog(commandRequest.ORDER_START, str(order))
    else:
        print "Error taking order, error code: %d" % status_code
      
    return ticket, succes

def finishOrder(order, ticket):
    ret = False
    order = str(order).zfill(5)
    orderUrl = serverAddress + "/orders/ord_" + order + "/ord_" + ticket
    status_code = deleteRequest(orderUrl)
    if status_code == 200:
        ret = True
        # Log to server
        placeLog(commandRequest.ORDER_DONE, str(order))
    else:
        print "Error finishing order, error code: %d" % status_code
    return ret

def findOrders(request):
    orders = []
    dataLock.acquire()
    for i in range(len(ComThread.OrderList)):
        if (ComThread.OrderList[i].red_bricks >= request.red_bricks and ComThread.OrderList[i].blue_bricks >= request.blue_bricks and ComThread.OrderList[i].yellow_bricks >= request.yellow_bricks):
            orders.append(ComThread.OrderList[i])
    dataLock.release()
    if len(orders) == 0:
        print "FindOrder: No orders found with the specified content"
    return orders

def constructLogXml(event, comment):
    eventString = str(event)
    commentString = str(comment)
    doc = Document()
    #Create root
    root = doc.createElement('log_entry')
    doc.appendChild(root)
    #Add event
    event = doc.createElement('event')
    root.appendChild(event)
    eventText = doc.createTextNode(eventString)
    event.appendChild(eventText)
    #Add timestamp
    time = doc.createElement('time')
    root.appendChild(time)
    timeText = doc.createTextNode(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
    time.appendChild(timeText)
    #Add cell ID
    cell = doc.createElement('cell_id')
    root.appendChild(cell)
    cellText = doc.createTextNode('4')
    cell.appendChild(cellText)
    #add comment
    comment = doc.createElement('comment')
    root.appendChild(comment)
    commentText = doc.createTextNode(commentString)
    comment.appendChild(commentText)
    
    return doc.toxml(encoding="utf-8")

def placeLog(log_event, log_comment):
    ret = False
    if log_event == commandRequest.PML_IDLE:
        event = 'PML_Idle'
    elif log_event == commandRequest.PML_EXECUTE:
        event = 'PML_Execute'
    elif log_event == commandRequest.PML_COMPLETE:
        event = 'PML_Complete'
    elif log_event == commandRequest.PML_HELD:
        event = 'PML_Held'
    elif log_event == commandRequest.PML_SUSPENDED:
        event = 'PML_Suspend'        
    elif log_event == commandRequest.PML_ABORTED:
        event = 'PML_Aborted'
    elif log_event == commandRequest.PML_STOPPED:
        event = 'PML_Stopped'
    elif log_event == commandRequest.ORDER_START:
        event = 'Order_Start'              
    elif log_event == commandRequest.ORDER_DONE:
        event = 'Order_Done'
    else:
        print "Error placing log - unknown event"
        return ret
    
    logXml = constructLogXml(event, log_comment)
                 
    status_code = postRequest(serverAddress + "/log", logXml)
    
    if status_code == 201:
        ret = True
    else:
        print "Error in placing log post to Messy Server, Error code: %d" % status_code
    
    return ret

def order_service_handler(req):
    ret = commandResponse()
    if req.command_number == req.GET_ORDERS:
        print "Get Orders"
        dataLock.acquire()
        ret.orders = ComThread.OrderList
        print "Length of Orderlist: %d" % len(ComThread.OrderList)
        dataLock.release()
    elif req.command_number == req.TAKE_ORDER:
        print "Take order"
        [ret.order_ticket, ret.succes] = takeOrder(req.order_id)
    elif req.command_number == req.FINISH_ORDER:
        print "Finish order"
        ret.succes = finishOrder(req.order_id, req.order_ticket)
    elif req.command_number == req.FIND_ORDERS:
        print "Find orders"
        ret.orders = findOrders(req)
    elif req.command_number == req.LOG:
        print "Logging"
        ret.succes = placeLog(req.log_event, req.log_comment)
        
    return ret

dataLock = threading.Lock()
ComThread = 0
serverAddress = ''

class CommunicationThread (threading.Thread):
    global dataLock
    # Private variables
    __SleepTime = 10.0
    __LoopRate = 10.0
    __NoOfOrders = 0
    __OrderList = []
    __OrderUrls = []
    
    # Public variables
    exitFlag = 0
    OrderList = []
    
    def __init__(self, threadID, name, noOfOrders):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.__NoOfOrders = noOfOrders
    def run(self):
        print "Starting " + self.name
        timer = 0.0
        while not self.exitFlag:
            if timer <= 0.0:
                timer = self.__SleepTime
                self.updateOrderList(self.__NoOfOrders)
            timer -= 1.0/self.__LoopRate
            time.sleep(1.0/self.__LoopRate)
        print "Exiting " + self.name
        
    def updateOrderList(self, max_orders):
        orderUrl = serverAddress + "/orders/" + str(self.__NoOfOrders)
        [status_code, response] = getRequest(orderUrl)
        # Check if request was succesful
        if status_code != 200:
            print "Updating orderlist failed, error code: %d" % status_code
            return
        
        dom = parseString(response)
        n = dom.getElementsByTagName('order');
        number_of_orders = max_orders
        if number_of_orders > n.length:
            number_of_orders = n.length
        
        # create list of order urls
        OrderUrls = []
        for i in range(0, number_of_orders):
            OrderUrls.append(dom.getElementsByTagName('order')[i].toxml().replace('<order>','').replace('</order>',''))
        
        self.__OrderUrls = OrderUrls
        
        # Remove taken orders
        OrdersToRemove = []
        for i in range(len(self.__OrderList)):
            isNotPresent = 1
            for j in range(len(self.__OrderUrls)):
                if self.__OrderList[i].order_id == int(self.__OrderUrls[j].replace(serverAddress + '/orders/ord_','')):
                    isNotPresent = 0
                    break
            if isNotPresent:
                OrdersToRemove.append(self.__OrderList[i])
        
        for i in range(len(OrdersToRemove)):
            self.__OrderList.remove(OrdersToRemove[i])
                
        # Add new orders
        for i in range(len(self.__OrderUrls)):
            isPresent = 0
            for j in range(len(self.__OrderList)):
                if int(self.__OrderUrls[i].replace(serverAddress + '/orders/ord_','')) == self.__OrderList[j].order_id:
                    isPresent = 1
                    break
                
            if not isPresent:
                [status_code, response] = getRequest(self.__OrderUrls[i])
                if status_code != 200:
                    print "Error getting order, error_code: %d" % status_code
                else:
                    r = requests.get(self.__OrderUrls[i],  stream = True)
                    orderXml = parseString(response)
                    orderData = order()
                    orderData.order_id = int(self.__OrderUrls[i].replace(serverAddress + '/orders/ord_',''))
                    orderData.red_bricks = int(orderXml.getElementsByTagName('red')[0].toxml().replace('<red>','').replace('</red>',''))
                    orderData.blue_bricks = int(orderXml.getElementsByTagName('blue')[0].toxml().replace('<blue>','').replace('</blue>',''))
                    orderData.yellow_bricks = int(orderXml.getElementsByTagName('yellow')[0].toxml().replace('<yellow>','').replace('</yellow>',''))
                    orderData.time  = orderXml.getElementsByTagName('time')[0].toxml().replace('<time>','').replace('</time>','')
                    xmlStatus = orderXml.getElementsByTagName('status')[0].toxml()
                    if xmlStatus.replace('<status>','').replace('</status>','') == "ready":
                        orderData.status = orderData.STATUS_READY
                    elif xmlStatus.replace('<status>','').replace('</status>','') == "taken":
                        orderData.status = orderData.STATUS_TAKEN
                    else:
                        orderData.status = orderData.STATUS_UNKNOWN
                    self.__OrderList.append(orderData)
        
        # Copy order list to public orderlist
        dataLock.acquire()
        self.OrderList = self.__OrderList
        dataLock.release()
        return

def order_service():
    rospy.init_node('Messy2Controller')
    global serverAddress
    serverAddress = "http://192.168.10.100"
    s = rospy.Service('/Messy2Controller/order_service', command, order_service_handler)
    print "Messy2Controller started and order_service started"
    
    global ComThread
    ComThread = CommunicationThread(1, "MessyCommunication", 200)
    ComThread.start()
    
    #Go to infinite loop waiting for service calls
    rospy.spin()
    
    print "Messy2Controller shutting down..."
    ComThread.exitFlag = 1; #Exiting Communication thread

if __name__ == "__main__":
    order_service()



