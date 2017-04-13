#!/usr/bin/env python

import rospy, tf, math, numpy
import heapq
import copy
from geometry_msgs.msg import Twist
from nav_msgs.msg import GridCells
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Pose, Point, PoseWithCovarianceStamped
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
from numpy import arctan
# Add additional imports for each of the message types used

wheel_base = 0.2286 #m
wheel_diam = 0.07 #m


class PQueue:
    def __init__(self):
        self._queue = []
        self._index = 0

        def push(self, item, priority):
            heapq.heappush(self._queue, (priority, self._index, item))
            self._index +=1

        def pop(self):
            return heapq.heappop(self._queue)[-1]

class aNode:
    def __init__(self, index, val, hestimate, gcost, adjacent):
        self.index = index
        self.val = val
        self.hestimate = hestimate
        self.gcost = gcost
        self.adjacent = list()
        self.f = 0
        self.cameFrom = -1

    def appendAdjacent(self, index):
        self.adjacent.append(index)

    def setupParent(self, index):
        self.cameFrom = (index)

def noFilter(path): #takes the parsed path & tries to remove unecessary zigzags 
    returnPath = list()
    for i,node in enumerate(path):
        point = Point()
        currNode = path[i]
        point.x = getWorldPointFromIndex(currNode).x
        point.y = getWorldPointFromIndex(currNode).y
        point.z = 0
        
        returnPath.append(point)
        #print "Point in Path: X: %f Y: %f" % (point.x, point.y)
    return returnPath

def mapCallBack(data):
    global mapData
    global width
    global height
    global mapgrid
    global resolution
    global offsetX
    global offsetY

    mapgrid = data
    resolution =data.info.resolution
    mapData = data.data
    width = data.info.width
    height = data.info.height
    offsetX = data.info.origin.position.x
    offsetY = data.info.origin.position.y

    print data.info

def getStart(_startPos):
    global startRead
    startRead = True
    global startPosX
    global startPosY
    global startPos
    global startIndex

    startPos = _startPos
    startPosX = startPos.pose.pose.position.x
    startPosY = startPos.pose.pose.position.y

    startIndex = getIndexFromWorldPoint(startPosX, startPosY)

    point = getWorldPointFromIndex(startIndex)
    
    startIndex = getIndexFromWorldPoint(startPosX, startPosY)
    print "Printing start pose"
    print startPos.pose.pose
    point = getWorldPointFromIndex(startIndex)
    print "Calculated world position: %f, %f Index: %i" % (point.x, point.y, startIndex)

    pass

def readGoal(goal):
    global goalRead
    goalRead = True
    global goalX
    global goalY
    global goalIndex
    goalX= goal.pose.position.x
    goalY= goal.pose.position.y
    
    goalIndex = getIndexFromWorldPoint(goalX,goalY)
    print "Printing goal pose"
    print goal.pose

# returns the index number given a point in the world
def getIndexFromPoint(x,y):
    global Point

    return int(((y)*width) + x)

#returns in meters the point of the current index

def getWorldPointFromIndex(index):
    
    global Point
    global offsetX
    global offsetY

    point=Point()
    #print "GetX: %i" % getX(index)
    point.x=(getX(index)*resolution)+offsetX + (1.5 * resolution)
    point.y=(getY(index)*resolution)+offsetY + (.5 * resolution)
    point.z=0
    return point

# returns the index number given a point in the world
def getIndexFromWorldPoint(x,y):
    #calculate the index coordinates
    indexX = int(((x-offsetX) - (1.5*resolution))/resolution)
    indexY = int(((y-offsetY) - (.5*resolution))/resolution)
    
    index = int (((indexY)*width) + indexX) 
    
    print index 
    return index

def heuristic(index): 
    current = getWorldPointFromIndex(index)
    h = math.sqrt(pow(goalX-current.x,2)+pow(goalY-current.y,2))
    return h

def findConnected(node):
    neighborhood = G.neighbors(node)
    return neighborhood

#returns the x value of the index
def getX(index):
    adjusted = index + 1
    if (adjusted % width) == 0:
        return width - 1
    else:
        return (adjusted % width) - 1

#returns the y value of the index
def getY(index):
    adjusted = index
    return math.floor(adjusted/width)
    
#checks if the passed point is in the map
def isInMap(point):
    #catch if point is negative
    if(point.x < 0 or point.y < 0):
        return False
    # is point within 0 and width and 0 and height?
    if( ( 0 <= point.x and width > point.x) and ( 0 <= point.y and height > point.y)):
        return True
    else:
        return False

#returns index of point above this one, only works for non-first row
def pointAbove(point):
    output = copy.deepcopy(point)
    output.y += 1
    return output

#returns index of point below this one, only works for non-last row
def pointBelow(point):
    output = copy.deepcopy(point)
    output.y -= 1
    return output

#returns index of point right of this one, only works for non-last column
def pointRight(point):
    output = copy.deepcopy(point)
    output.x += 1
    return output

#returns index of point right of this one, only works for non-first column
def pointLeft(point):
    output = copy.deepcopy(point)
    output.x -= 1
    return output

def linkMap():   
    for i in range(0, height*width):
        currentPoint = Point()
        currentPoint.x = getX(i)
        currentPoint.y = getY(i)
        #print "I is %i, x is %i, y is %i" % (i, currentPoint.x, currentPoint.y)
        # try adding north
        if(isInMap(pointAbove(currentPoint))):  
            myPoint = pointAbove(currentPoint)
            #print "My Point X: %i Y: %i calc Index: %i" % (myPoint.x, myPoint.y,getIndexFromPoint(myPoint.x,myPoint.y))
            G[i].appendAdjacent(getIndexFromPoint(myPoint.x,myPoint.y))
        currentPoint.x = getX(i)
        currentPoint.y = getY(i)
        # try adding east
        if(isInMap(pointRight(currentPoint))):
            myPoint = pointRight(currentPoint)
            #print "My Point X: %i Y: %i calc Index: %i" % (myPoint.x, myPoint.y,getIndexFromPoint(myPoint.x,myPoint.y))
            G[i].appendAdjacent(getIndexFromPoint(myPoint.x,myPoint.y))
        currentPoint.x = getX(i)
        currentPoint.y = getY(i)
        # try adding south
        if(isInMap(pointBelow(currentPoint))):
            myPoint = pointBelow(currentPoint)
            #print "My Point X: %i Y: %i calc Index: %i" % (myPoint.x, myPoint.y,getIndexFromPoint(myPoint.x,myPoint.y))
            G[i].appendAdjacent(getIndexFromPoint(myPoint.x,myPoint.y))
        currentPoint.x = getX(i)
        currentPoint.y = getY(i)
        # try adding west
        if(isInMap(pointLeft(currentPoint))):
            myPoint = pointLeft(currentPoint)
            #print "My Point X: %i Y: %i  calc Index: %i" % (myPoint.x, myPoint.y,getIndexFromPoint(myPoint.x,myPoint.y))
            G[i].appendAdjacent(getIndexFromPoint(myPoint.x,myPoint.y))

def initMap(): 
    global frontier
    for i in range(0, width*height):
        node = aNode(i,mapData[i],heuristic(i),0.0, 0)
        G.append(node) 
        frontier.append(0)
    print len(G)    
    linkMap()

def calcG(currentG, neighborG):
    if (neighborG == 0): 
        neighborG = currentG + resolution
    return neighborG

def adjCellCheck(current):
    global adjList
    global traversal
    adjList =  current.adjacent ## list of indexes of neighbor 
    for index in adjList:
        currCell = G[index] 
        if(currCell.val != 100): 
            evalNeighbor(currCell, current) 
        traversal.append(G[index])
        if index == goalIndex:
            print "Goooooooooaaaaaaaallllllllll"
            break
    publishTraversal(traversal)

def evalNeighbor(nNode, current): #check to see if in the closed set
    if(nNode not in closedSet): 
        tentative = current.gcost + resolution 
        if (nNode not in openSet) or (tentative < nNode.gcost): 
            if (nNode not in openSet):
                openSet.append(nNode)
            nNode.gcost = calcG(current.gcost+nNode.gcost, nNode.gcost)
            nNode.f = nNode.gcost + 2*nNode.hestimate
            G[nNode.index].cameFrom = current.index
    else:
        lowestInQ(openSet)

def lowestInQ(nodeSet): 
    costList = list() 
    for node in nodeSet:
        costList.append(node.f)

    a = costList.index(min(costList))
    mapIndex = nodeSet[a].index
    return mapIndex

def reconPath(current, start): 
    total_path = list()
    total_path.append(current.index)
            
    while (current.cameFrom != -1):
        current = G[current.cameFrom]
        total_path.append(current.cameFrom)     
             
    return total_path

def aStar():
    
    global G
    G = list()
    initMap()  # add all nodes to grah, link all nodes

    global path 
    path = list()
    global openSet
    global closedSet

    global traversal
    traversal = list()
    global frontier
    frontier = list()

    openSet = list()
    openSet.append(G[startIndex])        #Add first node to openSet # set priority to distance
    closedSet = list()         #everything that has been examined
    
    print "start a*"
    
    print len(openSet)
    #print openSet[0].index

    while openSet:
        try:
            i = lowestInQ(openSet) 
            current = G[i]
            if current in frontier: 
                frontier.remove(current)
            #print G[i].cameFrom
            if (current.index == goalIndex): 
                print reconPath(current, G[startIndex])
                                
                return reconPath(current, startIndex)
                pass
            openSet.remove(current)
            closedSet.append(current)       
            adjCellList = adjCellCheck(current)
            if adjCellList:
                for node in adjCellList:
                    if node not in closedSet:
                        frontier.append(node)
                        publishFrontier(frontier)
        except KeyboardInterrupt: 
            break
    
    print "No route to goal"

def getWaypoints(path): #calculate waypoints from optimal path
    
    global Point

    returnPath = list()
    point = Point()
    pointNode = path[0]
    point.x = getWorldPointFromIndex(pointNode).x
    point.y = getWorldPointFromIndex(pointNode).y
    point.z = 0
    returnPath.append(point)
    for i,node in enumerate(path):
        currPoint = Point()
        currNode = path[i]
        currPoint.x = getWorldPointFromIndex(currNode).x
        currPoint.y = getWorldPointFromIndex(currNode).y
        currPoint.z = 0

        if (i+1 < len(path)):
            nextPoint = Point()
            nextNode = path[i+1]
            nextPoint.x = getWorldPointFromIndex(nextNode).x
            nextPoint.y = getWorldPointFromIndex(nextNode).y
            nextPoint.z = 0

            if(math.degrees(math.fabs(math.atan2(nextPoint.y-currPoint.y,nextPoint.x-nextPoint.y))) >= 10):
                returnPath.append(currPoint)
        else:
            returnPath.append(currPoint)
            pass

    return returnPath

def publishWaypoints(grid):
    global pubway
    #print "publishing traversal"

        # resolution and offset of the map
    k=0
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution 
    cells.cell_height = resolution

    for node in grid:
        point=Point()
        point = node
        cells.cells.append(point)
    print "Point in Waypoint: X: %f Y: %f" % (point.x, point.y)
    pubway.publish(cells) 

def publishCells(grid):
    global pub
    #print "publishing"

    # resolution and offset of the map
    k=0
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution 
    cells.cell_height = resolution

    for i in range(1,height): #height should be set to hieght of grid
        k=k+1
        for j in range(1,width): #width should be set to width of grid
            k=k+1
            #print k # used for debugging
            if (grid[k] == 100):
                point=Point()
                point.x=(j*resolution)+offsetX + (1.5 * resolution) # added secondary offset 
                point.y=(i*resolution)+offsetY - (.5 * resolution) # added secondary offset ... Magic ?
                point.z=0
                cells.cells.append(point)
    pub.publish(cells) 

def publishFrontier(grid):
    global pub_frontier
    print "publishing frontier"

        # resolution and offset of the map
    k=0
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution 
    cells.cell_height = resolution

    for node in grid:
        point=Point()
        point = getWorldPointFromIndex(node.index)
        cells.cells.append(point)
    pub_frontier.publish(cells)

def publishTraversal(grid):
    global pub_traverse
    #print "publishing traversal"

        # resolution and offset of the map
    k=0
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution 
    cells.cell_height = resolution

    for node in grid:
        point=Point()
        point = getWorldPointFromIndex(node.index)
        cells.cells.append(point)
    pub_traverse.publish(cells) 

def publishPath(grid):
    global pub_path
    #print "publishing traversal"

        # resolution and offset of the map
    k=0
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution 
    cells.cell_height = resolution

    for node in grid:
        point=Point()
        point = node
        cells.cells.append(point)
    #print "Point in Path: X: %f Y: %f" % (point.x, point.y)
    pub_path.publish(cells) 

def pubGoal(grid):
    global goal_pub
    
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution
    cells.cell_height = resolution

    for node in grid:
        point = getWorldPointFromIndex(node.index)
        cells.cells.append(point)
    goal_pub.publish(cells)

#drive to a goal subscribed as /move_base_simple/goal
#cry


#Odom "Callback" function.
def readOdom(msg):
    global pose
    pose = msg
    global startRead
    global startPosX
    global startPosY
    global startPos
    global startIndex
    startPos = pose
    startPosX = startPos.pose.pose.position.x
    startPosY = startPos.pose.pose.position.y

    startIndex = getIndexFromWorldPoint(startPosX, startPosY)

    point = getWorldPointFromIndex(startIndex)
    
    startIndex = getIndexFromWorldPoint(startPosX, startPosY)
    #print "Printing start pose"
    print startPos.pose.pose
    point = getWorldPointFromIndex(startIndex)
    print "Calculated world position: %f, %f Index: %i" % (point.x, point.y, startIndex)

#This function sequentially calls methods to perform a trajectory.
def executeTrajectory():
    driveStraight(0.3, 0.06)
    rotate(1.6)
    driveStraight(0.3, 0.045)
    rotate(-2.4)

#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, time):
    # compute wheel speeds
    w = (u1 - u2) / wheel_base
    u = (wheel_diam / 2) * (u1 + u2)
    start = rospy.Time().now().secs
    while ((rospy.Time().now().secs - start) < time):
        publishTwist(u, w)
    publishTwist(0, 0)

#This function accepts a speed and a distance for the robot to move in a straight line

def driveStraight(speed, distance):
    global pose

    xnaught = pose.pose.position.x #set an origin at the robot's current position
    currpos = Twist()
    objectiveReached = False
    print "values set"
    while (not objectiveReached):

        x = pose.pose.position.x
        dx = x-xnaught
        print "in while loop"
        if (dx >= distance):
            objectiveReached = True
            publishTwist(0, 0)
            print "fuken dun m8"

        else:
            publishTwist(speed, 0)
            print "moving"

#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
    global odom_list
    global pose

    transformer = tf.TransformerROS()   
    rotation = numpy.array([[math.cos(angle), -math.sin(angle), 0], #Create th goal rotation
                            [math.sin(angle), math.cos(angle), 0],
                            [0,          0,          1]])
    #Get all the transforms for frames
    odom_list.waitForTransform('odom', 'base_footprint', rospy.Time(0), rospy.Duration(4.0))
    (trans, rot) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
    T_o_t = transformer.fromTranslationRotation(trans, rot)
    R_o_t = T_o_t[0:3,0:3]
    #Setup goal matrix
    goal_rot = numpy.dot(rotation, R_o_t)
    goal_o = numpy.array([[goal_rot[0,0], goal_rot[0,1], goal_rot[0,2], T_o_t[0,3]],
                 [goal_rot[1,0], goal_rot[1,1], goal_rot[1,2], T_o_t[1,3]],
                 [goal_rot[2,0], goal_rot[2,1], goal_rot[2,2], T_o_t[2,3]],
                 [0,             0,             0,             1]])
    #This code continuously creates and matches coordinate transforms.
    done = False
    while (not done and not rospy.is_shutdown()):
        (trans, rot) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
        state = transformer.fromTranslationRotation(trans, rot)
        within_tolerance = abs((state - goal_o)) < .2
        if ( within_tolerance.all() ):
            spinWheels(0,0,0)
            done = True
        else:
            if (angle > 0):
                spinWheels(.1,-.1,.1)
            else:
                spinWheels(-.1,.1,.1)

#This function takes angular and linear speeds and publishes them to a twist-type message
def publishTwist(linearvalue, angularvalue):
    global pub
    twist = Twist()
    twist.linear.x = linearvalue
    twist.angular.z = angularvalue
    pub_vel.publish(twist)


#Bumper Event Callback function
def readBumper(msg):
    global bumper
    if (msg.state == 1):
        print "boop"
        bumper = 1

# (Optional) If you need something to happen repeatedly at a fixed interval, write the code here.
# Start the timer with the following line of code: 

def timerCallback(event):
    global pose
    pose = Pose()

    (position, orientation) = odom_list.lookupTransform('...','...', rospy.Time(0)) #finds the position and oriention of two objects relative to each other (hint: this returns arrays, while Pose uses lists)
    
    pass # Delete this 'pass' once implemented

###############################################LAB 4 FUNCTIONS################################
#Expand obstacles
#Assumes robot has a 5 gridcell radius
#Map should be an array of gridcells
"""
def expandObstacles(map):
    get the first cell
    while cells have not been checked
        if cellvalue >=0
            add to checkedmap
        else if cellvalue < 0
            check cells in 5-cell radius
            if any cell in radius .value >0
                skip it
            else cellsinradius.value = -1
                add all to checkedmap
    
    return checkedmap
"""

def navWithAStar():
    global pose 

    path = aStar()
    publishPath(noFilter(path))
    publishWaypoints(getWaypoints(path))
    newpathPoints = getWaypoints(path)
    posePath = list()
    for point in newpathPoints:
        newPose = (msg.pose.pose.position.x,
                msg.pose.pose.position.y)
        posePath.add(newPose)
    donePoses = list()

    for pose in posePath:
        #for distance
        desx = newPose.pose.position.x
        desy = newPose.pose.position.y
        thisx = pose.pose.position.x
        thisy = pose.pose.position.y
        deltax = desx-thisx
        deltay = desy-thisy
        distancetoTraverse=pow((pow(deltax,2)+pow(deltay,2)),.5)
        #for angle to rotate
        phi=arctan(desy/desx)
        thisphi = arctan(thisy/thisx)
        angletoRotate = (180-phi)-thisphi
        rotate(angletoRotate)
        driveStraight(distancetoTraverse)
        donePoses.add(newPose)
        posePath.remove(newPose)

# This is the program's main function
if __name__ == '__main__':

    # These are global variables. Write "global <variable_name>" in any other function to gain access to these global variables 
    global pub
    global pose
    global odom_tf
    global odom_list
    global bumper
    global startRead
    startRead = False
    goalRead = False
    global pub_frontier
    global pub_traverse
    global pub_path
    global pubway
    global frontier
    frontier = list()
    global goal_pub

    bumper = 0

    # Change this node name to include your username
    rospy.init_node('lab3')
   
    pub_vel = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, None, queue_size=10) # Publisher for commanding robot motion
    bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events
    odom_sub = rospy.Subscriber('/odom', Odometry, readOdom)

    sub = rospy.Subscriber('/map', OccupancyGrid, mapCallBack)
    pub = rospy.Publisher('/mapcheck', GridCells) # Publisher for commanding robot motion
    pub_path = rospy.Publisher('/path', GridCells)
    pubway = rospy.Publisher('/waypoints', GridCells, queue_size=1)


    goal_sub = rospy.Subscriber('/goalpose', PoseStamped, readGoal)
    pub_traverse = rospy.Publisher('/traversal', GridCells, queue_size=1)
    pub_frontier = rospy.Publisher('/frontier', GridCells, queue_size=1)
    goal_pub = rospy.Publisher("/goalpose", PoseStamped, queue_size=1)
    # wait a second for publisher, subscribers, and TF
    rospy.sleep(1)
    # Use this object to get the robot's Odometry 
    odom_list = tf.TransformListener()
    
    print "Starting Lab 3"

    while (1 and not rospy.is_shutdown()):
        publishCells(mapData) #publishing map data every 2 seconds
        if goalRead:
            navWithAStar()
            goalRead = False
        rospy.sleep(2)

    print "Starting Lab 3"

    #while (bumper == 0):
    #    print bumper

    #executeTrajectory()
        #rospy.sleep(0.5)
    #make the robot keep doing something...
    #rospy.Timer(rospy.Duration(1), timerCallback)


    #spinWheels(0.2, 0.5, 5)
    #driveStraight(.3, 0.25)
    #rotate(-0.5)
    #executeTrajectory()
    #readBumper()

    # Make the robot do stuff...

