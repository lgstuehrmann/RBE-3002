#!/usr/bin/env python

import rospy, tf, numpy, math
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class Cell{
    
    def __init__(self, x, y, parent):
        self.x = x
        self.y = y
        self.parent = parent
        self.self = self
}

def navToPose(goal):
    global pose
    angle = 5
    rotate(angle)
    print "spin!"
    speed = 5
    distance = 5
    driveStraight(speed, distance)
    print "move!"
    #find difference between current angle and desired angle
    #pick velocities & calc time
    rotate(angle)
    print "spin!"
    #check position
    print "done"
    pass

#This function sequentially calls methods to perform a trajectory.
def executeTrajectory():
    driveStraight(.25, .6)
    rotate(1.5708)
    driveStraight(.25, .45)
    rotate(-2.3562)
    print "done trajectory"
    done = True
    pass  # Delete this 'pass' once implemented

#This function accepts two wheel velocities(linear in in/s) and a time interval(s).
def spinWheels(u1, u2, time):
    global pub

    beginning_time = rospy.Time().now().secs

    omega = (u2-u1)/.229
    linear = (.229/2)*(u1-u2)

    rob_pos = Twist()
    rob_pos.angular.z = omega
    rob_pos.linear.x = linear

    while(rospy.Time().now().secs - beginning_time < time):
        pub.publish(rob_pos)

    print "spin complete"

#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
    global pose
    px = pose.pose.position.x
    desx = px+distance
    rob_pos = Twist()
    there = False
    while(not there):
        nowx = pose.pose.position.x
        newdist = abs(nowx - px)
        print str(newdist)+','+str(distance)
        if(newdist >= distance):
            there = True
            rob_pos.linear.x = 0
            pub.publish(rob_pos)
        else:
            rob_pos.linear.x = speed
            pub.publish(rob_pos)
    
    print "done straight"
   
#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
    print "in rotate"
    global pose

    rob_pos = Twist()
    #Had to separate out the quaternion because it was geometry_msgs, not transform
    quaternion = (
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w)
    initeuler = tf.transformations.euler_from_quaternion(quaternion)
    inityaw = initeuler[2]
    print inityaw
    print angle

    there = False
    while(not there):
        currquaternion = (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w)
        curreuler = tf.transformations.euler_from_quaternion(currquaternion)
        curryaw = curreuler[2]
        newang = abs(curryaw - inityaw)
        print str(newang) +','+ str(angle)
        if(angle >= 0):
            if(newang >= angle):
                there = True
                rob_pos.angular.z = 0
                pub.publish(rob_pos)
            else:
                rob_pos.angular.z = .25
                pub.publish(rob_pos)
        else:
            if(abs(newang) >= abs(angle)):
                there = True
                rob_pos.angular.z = 0
                pub.publish(rob_pos)
            else:
                rob_pos.angular.z = -.75
                pub.publish(rob_pos)

    print "done angle"


#Bumper Event Callback function
def readBumper(msg):
    print "bumper event logged"
    if (msg.state == 1):
        print "bumper pressed"
        executeTrajectory()


def readOdom(msg):
    global pose
    try:
        pose = msg.pose
    except:
        print "no thing"
# This is the program's main function
if __name__ == '__main__':
    rospy.init_node('lgstuehrmann_Lab_2_node')
    
    global pub
    global pose
    global odom_tf
    global odom_list

    #Subscribers & Publishers
    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, None, queue_size=10) # Publisher for commanding robot motion
    lplanner_pub = rospy.Publisher('/move_base/DWAPlannerROS/local_plan', Path, None, queue_size=10)
    gplanner_pub = rospy.Publisher('/move_base/DWAPlannerROS/global_plan', Path, None, queue_size=10)
    fullplan_pub = rospy.Publisher('/move_base/NavfnROS/plan', Path, None, queue_size=10)

    odom_sub = rospy.Subscriber('/odom', Odometry, readOdom)
    map_sub = rospy.Subscriber('/map', OccupancyGrid, readMap)
    gcostmap_sub = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, readGlobalCostmap)
    lcostmap_sub = rospy.Subscriber('/move_base/lcoal_costmap/costmap', OccupancyGrid, readLocalCostmap)
    costcloud_sub = rospy.Subscriber('/move_base/DWAPlannerROS/cost_cloud', PointCloud2, readCostCloud)
    trajcloud_sub = rospy.Subscriber('/move_base/DWAPlannerROS/trajectory_cloud', PointCloud2, readTrajCloud)

    # Use this object to get the robot's Odometry 
    odom_list = tf.TransformListener()
    rospy.sleep(rospy.Duration(1, 0))
    print "Starting Lab 2"
    while not rospy.is_shutdown():
        rospy.spin()
    #spinWheels(1,.5,5)
    #driveStraight(.3, .25)
    #rotate(-1.5708)
    #executeTrajectory()
    print "Lab 2 complete!"

