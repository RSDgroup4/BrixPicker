#!/usr/bin/env python
import roslib; roslib.load_manifest('bpOrderHandler')
from bpMessy2Controller.srv import *
from bpMsgs.msg import order
from bzrlib.switch import switch
from rospy.timer import sleep
from rospy.timer import Timer
from std_msgs.msg import String
from std_msgs.msg import Bool
from bpMsgs.msg import brick
from bpMsgs.msg import system_state
from bpMsgs.msg import robot_pick
from bpMsgs.msg import oee_data
from bpPLCController.srv import *
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
last_state = system_state()
loop_rate = 10
bricks = []
queueBricks = []
currentOrders = [0, 0, 0]
bricksRemaining = [0, 0, 0]
offset_x = 0
offset_y = 0
belt_speed = 0
take_order = False
robot_publisher = 0
logPublisher = 0
orderDataPublisher = 0
emergencyStopStatus = False

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
                    tmpOrder.header.stamp = rospy.Time.now()
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

def execute(stopping):
    global bricks
    global bricksRemaining
    global robot_queue
    global logPublisher
    global totalOrders
    global ordersDone
    global currentOrders
    
    while len(queueBricks) > 0:
	bricks.append(queueBricks.pop())

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
                    logPublisher.publish("Order #%d done! in tray %d" % (currentOrders[i].order_id, currentOrders[i].order_tray))
                    rospy.loginfo("Order #%d done! in tray %d" % (currentOrders[i].order_id, currentOrders[i].order_tray))
                else:
                    rospy.loginfo("Order #%d done, but failed to be confirmed by MessyServer! in tray %d" % (currentOrders[i].order_id, currentOrders[i].order_tray))
                    logPublisher.publish("Order #%d done, but failed to be confirmed by MessyServer! in tray %d" % (currentOrders[i].order_id, currentOrders[i].order_tray))
                currentOrders[i] = 0
          
    if len(bricks) > 0 and robot_queue < 2:
        best_y = -10
        order_to_get_brick = -1
        bestBrick = -1
        for i in range(len(bricks)-1,-1,-1):
		shortestTimeLeft = 1000
                # check if brick is needed
                current_y = bricks[i].y + belt_speed * (rospy.Time.now().to_sec() - bricks[i].header.stamp.to_sec())
                # check if brick can be reached
                if current_y < 0.1:
                    # check if brick is closer to being out of reach
                    for j in range(len(currentOrders)):
                        if (currentOrders[j] != 0):
                            if ((bricks[i].type == bricks[i].RED and currentOrders[j].red_bricks > 0) or (bricks[i].type == bricks[i].YELLOW and currentOrders[j].yellow_bricks > 0) or (bricks[i].type == bricks[i].BLUE and currentOrders[j].blue_bricks > 0)):
                                if current_y >= best_y:
                                    # if so brick is best brick
                                    bestBrick = bricks[i]
                                    best_y = current_y
                                    if currentOrders[j].header.stamp.to_sec() - rospy.Time.now().to_sec() < shortestTimeLeft:
                                        order_to_get_brick = j
					shortestTimeLeft = currentOrders[j].header.stamp.to_sec() - rospy.Time.now().to_sec()
                else:
                    bricks.pop(i)
        # If brick is needed pick it..!
        if order_to_get_brick != -1 and bestBrick != -1:
            brickToPick = robot_pick()
            brickToPick.order = currentOrders[order_to_get_brick].order_tray
            brickToPick.belt_speed = belt_speed
            brickToPick.x = bestBrick.x
            brickToPick.y = bestBrick.y + belt_speed * (rospy.Time.now().to_sec() - bestBrick.header.stamp.to_sec())
            brickToPick.angle = bestBrick.angle
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
        elif (stopping == False):
            # Start new order
            for i in range(len(currentOrders)):
                if currentOrders[i] == 0:
                    blue = 0
                    red = 0
                    yellow = 0
                    for j in range(len(bricks)):
                        if bricks[j].type == bricks[j].RED:
                            red += 1
                        elif bricks[j].type == bricks[j].BLUE:
                            blue += 1
                        elif bricks[j].type == bricks[j].YELLOW:
                            yellow += 1
                    tmpOrder = findOrder(blue, red, yellow)
                    if tmpOrder == 0:
                        tmpOrder = findOrder(0,0,0)
                    currentOrders[i] = tmpOrder
                    if currentOrders[i] != 0:
                        currentOrders[i].order_tray = i+1
                        bricksRemaining[i] = currentOrders[i].red_bricks + currentOrders[i].yellow_bricks + currentOrders[i].blue_bricks
                        logPublisher.publish("Order #%d started in tray %d - ticket: \"%s\" at time: %s" % (currentOrders[i].order_id, currentOrders[i].order_tray, currentOrders[i].order_ticket, currentOrders[i].header.stamp))
                    break
        
            
def robotCallback(msg):
    global robot_queue
    global currentOrders
    robot_queue -= 1
    if (msg.succes):
        global bricksRemaining
        bricksRemaining[msg.order-1] -= 1
    else:
        if msg.type == msg.RED:
            currentOrders[msg.order-1].red_bricks += 1
        elif msg.type == msg.YELLOW:
            currentOrders[msg.order-1].yellow_bricks += 1
        elif msg.type == msg.BLUE:
            currentOrders[msg.order-1].blue_bricks += 1  
    rospy.loginfo("Brick picked, succes: %d, BrickType: %d, Remaining in order %d: %d" % (msg.succes, msg.type, msg.order-1, bricksRemaining[msg.order-1]))          

def changeStateCallback(msg):
    global next_state 
    rospy.loginfo("state changed!")
    next_state.state = msg.state
    
def brickCallback(msg):
    global queueBricks
    msg.x += offset_x
    msg.y -= offset_y
    queueBricks.append(msg)

def logState(last_state, new_state):
    if last_state.state != new_state.state:
        logService = rospy.ServiceProxy('/Messy2Controller/order_service', command)
        request = commandRequest()
        request.command_number = request.LOG
        request.log_event = new_state.state
        response = logService(request)

def emergencyStopCallback(msg):
    global emergencyStopStatus
    if msg.data == True:
        global next_state
        next_state.state = state.PML_ESTOP
        emergencyStopStatus = True
    elif msg.data == False:
        emergencyStopStatus = False

def orderHandler():
    rospy.init_node('OrderHandler')
    
    print "OrderHandler started"
    # Init
    global loop_rate
    loop_rate = rospy.get_param("~loop_rate", 10)
    global state
    state.state = state.PML_IDLE
    global next_state
    next_state.state = state.PML_IDLE
    global last_state
    last_state.state = state.PML_IDLE
    
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
    
    # Emergency stop subscriber
    emergencyStopSubTopic = rospy.get_param("~emergency_stop_topic", "/emergency_stop")
    rospy.Subscriber(emergencyStopSubTopic, Bool, emergencyStopCallback)
    
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
    global currentOrders  

    # End of Init
    
    
    # PML statemachine loop
    while not rospy.is_shutdown():
        TotalRuntime += 1.0/loop_rate
        if hasStateChanged():
            rospy.loginfo("System state changed from %d to %d" % (state.state, next_state.state))
            last_state.state = state.state
            state.state = next_state.state       
                 
        if state.state == state.PML_IDLE:
            logState(last_state, state)
            rospy.loginfo("PML_IDLE")
        elif state.state == state.PML_EXECUTE:
            logState(last_state, state)
            execute(False)
            ProductionTime += 1.0/loop_rate
        elif state.state == state.PML_COMPLETE:
            rospy.loginfo("PML_COMPLETE")
        elif state.state == state.PML_HELD:
            rospy.loginfo("PML_HELD")
        elif state.state == state.PML_SUSPENDED:
            rospy.loginfo("PML_SUSPENDED")
        elif state.state == state.PML_ABORTED:
            logState(last_state, state)
            rospy.loginfo("PML_ABORTED")
        elif state.state == state.PML_STOPPED:
            logState(last_state, state)
            rospy.loginfo("PML_STOPPED")
        # Transition states
        elif state.state == state.PML_START: #Start the system from Idle state ending in execute
            if (last_state.state != state.PML_IDLE):
                logPublisher.publish("System can only be started from IDLE.. Try resetting system")
                state.state = last_state.state
                next_state.state = last_state.state
            else:
                next_state.state = state.PML_EXECUTE
                plcService = rospy.ServiceProxy('/plc_controller/plc_command', plc_command)
                request = plc_commandRequest()
                request.command_number = request.START_BELT
                response = plcService(request)
            rospy.loginfo("PML_START")
        elif state.state == state.PML_STOP: #Controlled stop - finish orders
            rospy.loginfo("PML_STOP")
            if (last_state.state != state.PML_EXECUTE and last_state.state != state.PML_STOP):
                logPublisher.publish("System can only be stopped from EXECUTE..")
                state.state = last_state.state
                next_state.state = last_state.state
            else:
                if last_state != state.PML_STOP:
                    logPublisher.publish("System is stopping - current orders are finished first..")
                execute(True)
                if (currentOrders[0] == 0 and currentOrders[1] == 0 and currentOrders[2] == 0):
                    next_state.state = state.PML_STOPPED
                    plcService = rospy.ServiceProxy('/plc_controller/plc_command', plc_command)
                    request = plc_commandRequest()
                    request.command_number = request.STOP_BELT
                    response = plcService(request)
                         
        elif state.state == state.PML_RESET: #Reset after ESTOP and ABORTED
            rospy.loginfo("PML_RESET")
            if emergencyStopStatus == False:
                if last_state.state == state.PML_STOPPED:
                    next_state.state = state.PML_IDLE
                elif last_state.state == state.PML_ABORTED:
                    next_state.state = state.PML_IDLE
                else:
                    if last_state != state.PML_RESET:
                        logPublisher.publish("System can only be resetted from STOPPED and ABORTED..")                       
                    state.state = last_state.state
                    next_state.state = last_state.state
            elif last_state.state != state.PML_RESET:
                logPublisher.publish("Emergency stop is engaged - release before reset!")

                
        elif state.state == state.PML_ESTOP: #STOP the system and end in aborted
            currentOrders[0] = 0;
            currentOrders[1] = 0;
            currentOrders[2] = 0;            
            rospy.loginfo("PML_ESTOP")
            logPublisher.publish("ESTOP..! Release Emergency stop, reengage robot arm power, resume robot program, press reset and start to start system")
            plcService = rospy.ServiceProxy('/plc_controller/plc_command', plc_command)
            request = plc_commandRequest()
            request.command_number = request.STOP_BELT
            response = plcService(request)
            next_state.state = state.PML_ABORTED
            
        # Calculate and publish data for GUI
        counter += 1
        if (counter > loop_rate): #Runs once a second
            counter = 0
            # OEE data
            oeeData = oee_data()
            oeeData.availability = (ProductionTime / TotalRuntime)
            oeeData.performance = ((idealOrderTime * ordersDone) / (ProductionTime + 0.00001))
            oeeData.quality = ordersDone / (totalOrders + 0.00001)
            oeeData.oee = oeeData.availability*oeeData.performance*oeeData.quality
            oeeDataPublisher.publish(oeeData)       
            #System state
            systemStatePublisher.publish(state)
	    #Order data
	    for i in range(len(currentOrders)):
	    	if currentOrders[i] != 0:
		    tmpOrder = order()
		    tmpOrder.red_bricks = currentOrders[i].red_bricks
		    tmpOrder.yellow_bricks = currentOrders[i].yellow_bricks
		    tmpOrder.blue_bricks = currentOrders[i].blue_bricks
		    tmpOrder.order_id = currentOrders[i].order_id
		    tmpOrder.order_tray = currentOrders[i].order_tray
		    tmpOrder.header.stamp = currentOrders[i].header.stamp
		    tmpOrder.time = str(int(180 - (rospy.Time.now().to_sec() - currentOrders[i].header.stamp.to_sec())))
		    orderDataPublisher.publish(tmpOrder)
		else:
		    tmpOrder = order()
		    tmpOrder.red_bricks = 0
		    tmpOrder.yellow_bricks = 0
		    tmpOrder.blue_bricks = 0
		    tmpOrder.order_id = 0
		    tmpOrder.order_tray = i+1
		    tmpOrder.header.stamp = rospy.Time.now()
		    tmpOrder.time = str(0)
		    orderDataPublisher.publish(tmpOrder)		    
        
        last_state.state = state.state
        rospy.sleep(1.0/loop_rate)
        
    rospy.spin()
    
    print "OrderHandler shutting down..."

if __name__ == "__main__":
    orderHandler()



