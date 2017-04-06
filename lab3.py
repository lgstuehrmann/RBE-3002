#!/usr/bin/env python

import rospy, tf, numpy, math, sys
from sets import Set

from std_msgs.msg import String
from nav_msgs.msg import Odometry, MapMetaData
from std_msgs.msg import Header
from geometry_msgs.msg import Twist, Point, PoseStamped, PoseWithCovarianceStamped, PointStamped
from nav_msgs.srv import GetMap
from nav_msgs.srv import GetMapRequest, GetMapResponse
from nav_msgs.msg import GridCells
from tf.transformations import euler_from_quaternion
from __builtin__ import map
from rospy.rostime import Duration

global initPose
global goal



class map():
    map = [[]]
    map_res = .1
    map_posList =[0,0,0]
    map_orientList = [0,0,0,1]
    map_width = 1
    map_height = 1
    threshUp = 80
    threshLow = 20

    closedPts = []
    def __init__(self):
        self.getInitial = rospy.ServiceProxy('/startpose', getInitPose)
        self.getGoal = rospy.ServiceProxy('/goalpose', getGoalPose)
        self.getMapService = rospy.ServiceProxy('/static_map', GetMap)
        self.pathCellsPub = rospy.Publisher('/path_cells', GridCells, latch=True)
        self.explCellsPub = rospy.Publisher('/explored_cells', GridCells, latch=True)
        self.fronCellsPub = rospy.Publisher('/frontier_cells', GridCells, latch=True)

    def updateMapMetadata(self, msg):
        self.map_res = msg.resolution
        self.map_posList = [msg.origin.position.x, msg.origin.position.y, msg.origin.position.z]
        self.map_orientList = [msg.origin.orientation.x, msg.origin.orientation.y, msg.origin.orientation.z, msg.origin.orientation.w]
        self.map_width = msg.width
        self.map_height = msg.height

    def getMap(self):
        request = GetMapRequest()
        response = self.getMapService.call(request)

        self.updateMapMetadata(response.map.info)

    def Astar(self, start, goal):
        closedSet = Set()
        openSet = Set()
        came_from = {}

        gScore = {}
        fScore = {}

        gScore[start] = 0
        fScore[start] = gScore[start] + self.heuristic_cost_estimate(start, goal)

        openSet.add(start)

        while (len(openSet) > 0):
            frontier_set = openSet.intersection(fScore.keys())
            frontier_dict = {k: f_score[k] for k in frontier_set}
            current = min(frontier_dict, key=frontier_dict.get)
            if(current == goal):
                return self.getPath(came_from, goal)

            openSet.remove(current)
            closedSet.add(current)
            for neighbor, distance in self.getNeighbors(current):
                if (neighbor in closedSet):
                    continue
                self.pub_cells(closedSet, self.explCellsPub, cells_as_points=False)

                tent_gScore = gScore[current] + distance
                if(neighbor not in openSet) or (tent_gScore < gScore[neighbor]):
                    came_from[neighbor] = current
                    gScore[neighbor] = tent_gScore
                    fScore[neighbor] = gScore[neighbor]+self.heuristic_cost_estimate(neighbor, goal_cell)
                    if(neighbor not in openSet):
                        openSet.add(neighbor)

    def checkAdd(cell, delta, neighbors, distances):
        new_cell = tuple(map(add, cell, delta))
        if not ((self.map[new_cell[1]] [new_cell[0]]) ==1):
            neighbors.append(new_cell)
            distances.append(hypot(*map(sub, (0,0), delta)))

    def getNeighbors(cell):
        neighbors = []
        distances = []

        checkAdd(cell, (1,0), neighbors, distances)
        checkAdd(cell, (0,1), neighbors, distances)
        checkAdd(cell, (-1,0), neighbors, distances)
        checkAdd(cell, (0,-1), neighbors, distances)

        return zip(neighbors, distances)


    def heuristic_cost_estimate(start, goal):
        heuristic = sqrt(pow((goal.x-start.x), 2)+pow((goal.y-start.y),2))
        return heuristic

    def getPath(self, came_from, current):
        if current in came_from:
            path = self.getPath(came_from, came_from[current])
            path.append(current)
            return path
        else:
            return[current]

    def cellCoordsToPoints(self, coords):
        cellSize = [self.map_res, self.map_res]
        cellOffset = map(add, self.map_posList[:2], [self.map_res/2, self.map_res/2])
        return [Point(*(map(add, map(mul, coord, cellSize), cellOffset) + [0]) ) for coord in coords]

    def pubCells(self, cells, cellPub):
        msg = GridCells()
        msg.cell_height = map.map_res
        msg.cell_width = map.map_res
        msg.header.frame_id = '/map'

        msg.cells = map.cellCoordsToPts(cells)
        cellPub.publish(msg)

    def clearCells(self):
        emptymsg = GridCells(cell_width = self.map_res,
            cell_height = self.map_res,
            header = Header(frame_id ="/map"))
        self.pathCellsPub.publish(emptymsg)
        self.explCellsPub.publish(emptymsg)
        self.fronCellsPub.publish(emptymsg)

def getInitPose(msg):
    global initPose
    try:
        initPose = msg.pose
    except:
        print "no thing"

def getGoalPose():
    global goal
    try:
        goal = msg.pose
    except:
        print "no thing"

# This is the program's main function
if __name__ == '__main__':
    rospy.init_node('lgstuehrmann_Lab_3_node')
    
    global pub
    global pose
    global odom_tf
    global odom_list
    global initPose

    #Subscribers & 

    cellPub = rospy.Publisher
    button_sub = rospy.Subscriber('/clicked_point', PointStamped, map.Astar)
    #initpose_sub = rospy.Subscriber('/startpose', PoseWithCovarianceStamped, getInitPose)
    #goal_sub = rospy.Subscriber('/goalpose', PoseStamped, getGoal)

    #odom_sub = rospy.Subscriber('/odom', Odometry, readOdom)

    # Use this object to get the robot's Odometry 
    #odom_list = tf.TransformListener()
    rospy.sleep(rospy.Duration(1, 0))
    rospy.wait_for_service('/goalpose')

    print "Starting Lab 3"

    map = map()
    map.getMap()

    rospy.sleep(Duration(1,0))

    map.clearCells()

    rospy.sleep(Duration(1,0))
    path = map.Astar((initPose.position.x,initPose.position.y), (goal.position.x,goal.position.y))

    map.pubCells(path, map.pathCellsPub)

    print "Finished Lab 3"



