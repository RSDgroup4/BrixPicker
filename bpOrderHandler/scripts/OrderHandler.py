#!/usr/bin/env python
import roslib; roslib.load_manifest('bpOrderHandler')
from bpMessy2Controller.srv import *
from bpMsgs.msg import order
from bzrlib.switch import switch
from rospy.timer import sleep
from rospy.timer import Timer
from std_msgs.msg import String
from bpMsgs.msg import brick
from bpMsgs.msg import system_state
from bpMsgs.msg import robot_pick
from bpMsgs.msg import oee_data
from bpMessy2Controller.srv import *
from dynamic_reconfigure.server import Server
from bpOrderHandler.cfg import offset_paramsConfig
import rospy
import threading
import time
import datetime
import genpy

#Global variables
state = system_state()
next_state = system_state()
loop_rate = 10
bricks = []
order1 = 0
currentOrders = [0, 0, 0]
bricksRemaining = [0, 0, 0]
offset_x = 0
offset_y = 0
belt_speed = 0
take_order = False
robot_publisher = 0
logPublisher = 0
orderDataPublisher = 0

take_all_bricks = False
robot_queue = 0;
dataLock = threading.Lock()
ordersDone = 0.0
totalOrders = 0.0

def dynamic_reconfiguration_callback(config, level):
    global offset_x
    global offset_y
    global belt_speed
    global take_order
    global take_all_bricks
    offset_x = config.offset_x
    offset_y = config.offset_y
    belt_speed = config.belt_speed
    take_order = config.take_order
    take_all_bricks = config.take_all_bricks
    return config

def hasStateChanged():
    if next_state.state != state.state:
        return True
    return False

def findOrder(blue, red, yellow):
    rospy.wait_for_service('/Messy2Controller/order_service')
    try:
        getOrders = rospy.ServiceProxy('/Messy2Controller/order_service', command)
        request = commandRequest()
        request.command_number = request.FIND_ORDERS
        request.blue_bricks = blue
        request.red_bricks = red
        request.yellow_bricks = yellow
        response = getOrders(request)
        if len(response.orders) > 0:
            for i in range(len(response.orders)):
                tmpOrder = response.orders[i]
                order_ticket = takeOrder(tmpOrder)
                if order_ticket != 0:
                    tmpOrder.order_ticket = order_ticket
                    tmpOrder.order_tray = 1 #TODO fix!
                    tmpOrder.header.stamp = rospy.Time.now()
                    #rospy.loginfo("ID: %d, ticket: %s, time: %s" % (order1.order_id, order1.order_ticket, order1.header.stamp))
                    #rospy.loginfo("New order chosen with id: %d - blue: %d, red %d, yellow %d" % (order1.order_id, order1.blue_bricks, order1.red_bricks, order1.yellow_bricks))
                    return tmpOrder
        return 0
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        
def takeOrder(order):
    rospy.wait_for_service('/Messy2Controller/order_service')
    try:
        takeOrder = rospy.ServiceProxy('/Messy2Controller/order_service', command)
        request = commandRequest()
        request.command_number = request.TAKE_ORDER
        request.order_id = order.order_id
        response = takeOrder(request)
        if response.succes:
            return response.order_ticket
        else:
            rospy.loginfo("Taking order failed, order_id: %d" % order.order_id)
            return 0
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return 0
    
    
def finishOrder(order):
    rospy.wait_for_service('/Messy2Controller/order_service')
    try:
        finishOrder = rospy.ServiceProxy('/Messy2Controller/order_service', command)
        request = commandRequest()
        request.command_number = request.FINISH_ORDER
        request.order_id = order.order_id
        request.order_ticket = order.order_ticket
        response = finishOrder(request)
        if response.succes:
            return True
        else:
            rospy.loginfo("Sending Finishing order message failed, order_id: %d" % order.order_id)
            return False
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return False

def execute():
    global bricks
    global bricksRemaining
    global robot_queue
    global logPublisher
    global totalOrders
    global ordersDone
    global currentOrders
    # Check if orders are done or too old
    for i in range(len(currentOrders)):
        # Check if order exists 
        if currentOrders[i] != 0:
            # Check if order is to old (Over 3 minuts)
            if rospy.Time.now().to_sec() - currentOrders[i].header.stamp.to_sec() > 180:
                rospy.loginfo("Order #%d from tray %d not done in time..! Order is being discarded" % (currentOrders[i].order_id, currentOrders[i].order_tray))
                logPublisher.publish("Order #%d from tray %d not done in time..! Order is being discarded" % (currentOrders[i].order_id, currentOrders[i].order_tray))
                totalOrders += 1
                currentOrders[i] = 0
            # Check if order is done
            elif currentOrders[i].blue_bricks == 0 and currentOrders[i].red_bricks == 0 and currentOrders[i].yellow_bricks == 0 and bricksRemaining[i] <= 0:
                totalOrders += 1
                if finishOrder(currentOrders[i]):
                    ordersDone += 1
                    logPublisher.publish("Order #%d done! in tray %d" % (order1.order_id, order1.order_tray))
                    rospy.loginfo("Order #%d done! in tray %d" % (order1.order_id, order1.order_tray))
                else:
                    rospy.loginfo("Order #%d done, but failed to be confirmed by MessyServer! in tray %d" % (order1.order_id, order1.order_tray))
                    logPublisher.publish("Order #%d done, but failed to be confirmed by MessyServer! in tray %d" % (order1.order_id, order1.order_tray))
                currentOrders[i] = 0
          
    if len(bricks) > 0 and robot_queue < 2:
        best_y = -10
        shortestTimeLeft = 1000
        order_to_get_brick = -1
        for i in range(len(bricks)-1,-1,-1):
                # check if brick is needed
                current_y = bricks[i].y + belt_speed * (rospy.Time.now().to_sec() - bricks[i].header.stamp.to_sec())
                # check if brick can be reached
                if current_y < 0.1:
                    # check if brick is closer to being out of reach
                    for j in range(len(currentOrders)):
                        if (currentOrders[i] != 0):
                            if ((bricks[i].type == bricks[i].RED and currentOrders[i].red_bricks > 0) or (bricks[i].type == bricks[i].YELLOW and currentOrders[i].yellow_bricks > 0) or (bricks[i].type == bricks[i].BLUE and currentOrders[i].blue_bricks > 0)):
                                if current_y >= best_y:
                                    # if so brick is best brick
                                    bestBrick = bricks[i]
                                    best_y = current_y
                                    if currentOrders[i].header.stamp.to_sec() - rospy.Time.now().to_sec() < shortest_time_left:
                                        order_to_get_brick = i
                else:
                    bricks.pop(i)
        # If brick is needed pick it..!
        if order_to_get_brick != -1:
            brickToPick = robot_pick()
            brickToPick.order = currentOrders[order_to_get_brick].order_tray
            brickToPick.belt_speed = belt_speed
            brickToPick.x = bestBrick.x
            brickToPick.y = bestBrick.y + belt_speed * (rospy.Time.now().to_sec() - bestBrick.header.stamp.to_sec())
            brickToPick.angle = 0 #bestBrick.angle
            brickToPick.id = bestBrick.id
            brickToPick.header.stamp = rospy.Time.now()
            brickToPick.type = bestBrick.type
            robot_publisher.publish(brickToPick)
            robot_queue += 1
            rospy.loginfo("X: %f, Y: %f, Type: %d Robot queue: %d" % (brickToPick.x, brickToPick.y, brickToPick.type, robot_queue))
            bricks.remove(bestBrick)
            if brickToPick.type == brickToPick.RED:
                currentOrders[order_to_get_brick].red_bricks -= 1
            elif brickToPick.type == brickToPick.YELLOW:
                currentOrders[order_to_get_brick].yellow_bricks -= 1
            elif brickToPick.type == brickToPick.BLUE:
                currentOrders[order_to_get_brick].blue_bricks -= 1
        # No orders need any of the bricks in the system - start new order..!
        else:
            # Start new order
            for i in range(len(currentOrders)):
                if currentOrders[i] == 0:
                    currentOrders[i] = findOrder(0,0,0)
                    currentOrders[i].order_tray = i+1
                    if currentOrders[i] != 0:
                        bricksRemaining[i] = currentOrders[i].red_bricks + currentOrders[i].yellow_bricks + currentOrders[i].blue_bricks
                        logPublisher.publish("Order #%d started in tray %d - ticket: \"%s\" at time: %s" % (order1.order_id, order1.order_tray, order1.order_ticket, order1.header.stamp))
                    break
                else:
                    rospy.loginfo("Error picking order")

    rospy.loginfo("Remaining: blue: %d, yellow: %d, red: %d - BricksInSys: %d - PicksRemain: %d, totalOrders: %d, ordersDone: %d" % (order1.blue_bricks, order1.yellow_bricks, order1.red_bricks, len(bricks), bricksRemaining, totalOrders, ordersDone))
        
            
def robotCallback(msg):
    global robot_queue
    robot_queue -= 1
    if (msg.succes):
        global bricksRemaining
        bricksRemaining[msg.order-1] -= 1
    else:
        if msg.type == msg.RED:
            currentOrder[msg.order-1].red_bricks += 1
        elif msg.type == msg.YELLOW:
            currentOrder[msg.order-1].yellow_bricks += 1
        elif msg.type == msg.BLUE:
            currentOrder[msg.order-1].blue_bricks += 1  
    rospy.loginfo("Brick picked, succes: %d, BricksRemaining: %d" % (msg.succes, bricksRemaining))
    
            

def changeStateCallback(msg):
    global next_state 
    rospy.loginfo("state changed!")
    next_state.state = msg.state
    
def brickCallback(msg):
    rospy.loginfo("Brick recieved")
    global bricks
    brickAlreadyExists = False
    for i in range(len(bricks)):
        if msg.id == bricks[i].id:
            brickAlreadyExists = True
    if not brickAlreadyExists:
        msg.x += offset_x
        msg.y -= offset_y
        bricks.append(msg)

def orderHandler():
    rospy.init_node('OrderHandler')
    
    print "OrderHandler started"
    # Init
    global loop_rate
    loop_rate = rospy.get_param("~loop_rate", 10)
    global state
    state.state = state.PML_IDLE
    global next_state
    next_state.state = state.PML_EXECUTE
    
    # Subscriber for external state change
    changeStateTopic = rospy.get_param("~change_state_sub_topic", "/bpOrderHandler/change_state")
    rospy.Subscriber(changeStateTopic, system_state, changeStateCallback)
    
    # Subscriber for detected legobricks
    brickTopic = rospy.get_param("~brick_sub_topic", "/bpBrickDetector/bricks")
    rospy.Subscriber(brickTopic, brick, brickCallback)
    
    # Publisher for the robot motion planner
    robotplannerPubTopic = rospy.get_param("~robot_planner_pub_topic", "/bpRobotMotionController/brick_command_topic")
    global robot_publisher
    robot_publisher = rospy.Publisher(robotplannerPubTopic, robot_pick)
    
    # Subscriber for the robot planner messages
    robotplannerSubTopic = rospy.get_param("~robot_planner_sub_topic", "/bpRobotMotionController/brick_response_topic")
    rospy.Subscriber(robotplannerSubTopic, robot_pick, robotCallback)
    
    # Add dynamic reconfiguration server
    dyn_srv = Server(offset_paramsConfig, dynamic_reconfiguration_callback)
    
    global robot_queue
    robot_queue = 0
    
    # Add publisher for the OEE data
    counter = 0 # counter for slowing down the calculations
    oeeDataPubTopic = rospy.get_param("~oee_data_publisher_topic", "/oee")
    oeeDataPublisher = rospy.Publisher(oeeDataPubTopic, oee_data)
    
    # timers for the OEE calculations
    TotalRuntime = 0
    ProductionTime = 0
    idealOrderTime = 45
    
    # System state publisher for the GUI
    systemStatePubTopic = rospy.get_param("~system_state_publisher_topic", "/bpOrderHandler/system_state")
    systemStatePublisher = rospy.Publisher(systemStatePubTopic, system_state)
   
    # Log publisher 
    logPubTopic = rospy.get_param("~log_publisher_topic", "/log")
    global logPublisher
    logPublisher = rospy.Publisher(logPubTopic, String)
    
    # Order data publisher
    orderDataPubTopic = rospy.get_param("~order_data_publisher_topic", "/bpOrderHandler/order_status")
    global orderDataPublisher 
    orderDataPublisher = rospy.Publisher(orderDataPubTopic, order)

    global ordersDone
    global totalOrders    

    # End of Init
    
    
    # PML statemachine loop
    while not rospy.is_shutdown():
        TotalRuntime += 1.0/loop_rate
        if hasStateChanged():
            rospy.loginfo("System state changed from %d to %d" % (state.state, next_state.state))
            state.state = next_state.state       
                 
        if state.state == state.PML_IDLE:
            rospy.loginfo("PML_IDLE")
        elif state.state == state.PML_EXECUTE:
            execute()
            ProductionTime += 1.0/loop_rate
        elif state.state == state.PML_COMPLETE:
            rospy.loginfo("PML_COMPLETE")
        elif state.state == state.PML_HELD:
            rospy.loginfo("PML_HELD")
        elif state.state == state.PML_SUSPENDED:
            rospy.loginfo("PML_SUSPENDED")
        elif state.state == state.PML_ABORTED:
            rospy.loginfo("PML_ABORTED")
        elif state.state == state.PML_STOPPED:
            rospy.loginfo("PML_STOPPED")
            
        # Calculate and publish data for GUI
        counter += 1
        if (counter > loop_rate): #Runs once a second
            counter = 0
            # OEE data
            oeeData = oee_data()
            oeeData.availability = (ProductionTime / TotalRuntime)
            oeeData.performance = ((idealOrderTime * ordersDone) / ProductionTime)
            oeeData.quality = ordersDone / (totalOrders + 0.00001)
            oeeData.oee = oeeData.availability*oeeData.performance*oeeData.quality
            oeeDataPublisher.publish(oeeData)       
            #System state
            systemStatePublisher.publish(state)
	    #Order data
	    if order1 != 0:
	    	tmpOrder = order()
            tmpOrder.red_bricks = order1.red_bricks
            tmpOrder.yellow_bricks = order1.yellow_bricks
            tmpOrder.blue_bricks = order1.blue_bricks
            tmpOrder.order_id = order1.order_id
            tmpOrder.order_tray = order1.order_tray
            tmpOrder.header.stamp = order1.header.stamp
            tmpOrder.time = str(int(180 - (rospy.Time.now().to_sec() - order1.header.stamp.to_sec())))
            orderDataPublisher.publish(tmpOrder)
        
        rospy.sleep(1.0/loop_rate)
        
    rospy.spin()
    
    print "OrderHandler shutting down..."

if __name__ == "__main__":
    orderHandler()



