#!/usr/bin/env python

import rospy, tf, numpy, math, sys
from sets import Set

from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import *
from tf.transformations import euler_from_quaternion

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

#takes in current cell, start cell, and goal cell
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
            for(neighbor, distance in self.getNeighbors(current)):
                if (neighbor in closedSet):
                    continue
                self.pub_cells(closedSet, self.expl_cells_pub, cells_as_points=False)

                tent_gScore = gScore[current] + distance
                if(neighbor not in openSet) or (tent_gScore < gScore[neighbor]):
                    came_from[neighbor] = current
                    gScore[neighbor] = tent_gScore
                    fScore[neighbor] = gScore[neighbor]+self.heuristic_cost_estimate(neighbor, goal_cell)
                    if(neighbor not in openSet):
                        openSet.add(neighbor)

    def getNeighbors(cell):
        neighbors = []
        distances = []

        def checkAdd(cell, delta, neighbors, distances):
            new_cell = tuple(map(add, cell, delta))
            if not ((self.map[new_cell[1]] [new_cell[0]]) ==1):
                neighbors.append(new_cell)
                distances.append(hypot(*map(sub, (0,0), delta)))

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

    def cellCoordsToPoints(self, )
    def pubCells(self, cells, cellPub):
        msg = GridCells()
        msg.cell_height = map.map_res
        msg.cell_width = map.map_res
        msg.header.frame_id = '/map'

        msg.cells = map.cellCoordsToPts(cells)
        cellPub.publish(msg)

def getInitPose
    global initPose
    try:
        initPose = msg.pose
    except:
        print "no thing"

def getGoal
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
    #Subscribers & Publishers
    cellPub = rospy.Publisher
    initpose_sub = rospy.Subscriber('/startpose', Pose, getInitPose)
    goal_sub = rospy.Subscriber('/goalpose', Pose, getGoal)
    odom_sub = rospy.Subscriber('/odom', Odometry, readOdom)

    # Use this object to get the robot's Odometry 
    odom_list = tf.TransformListener()
    rospy.sleep(rospy.Duration(1, 0))
    print "Starting Lab 3"

    map = Map()
    map.getMap()

    rospy.sleep(Duration(1,0))

    map.clearCells()

    rospy.sleep(Duration(1,0))
    path = map.Astar((initPose.x,initPose.y), (goal.x,goal.y))

    map.pubCells(path, map.pathCellsPub)

    print "Finished Lab 3"



