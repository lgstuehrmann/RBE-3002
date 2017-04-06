map_sub = rospy.Subscriber('/map', OccupancyGrid, readMap)
    gcostmap_sub = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, readGlobalCostmap)
    lcostmap_sub = rospy.Subscriber('/move_base/lcoal_costmap/costmap', OccupancyGrid, readLocalCostmap)
    costcloud_sub = rospy.Subscriber('/move_base/DWAPlannerROS/cost_cloud', PointCloud2, readCostCloud)
    trajcloud_sub = rospy.Subscriber('/move_base/DWAPlannerROS/trajectory_cloud', PointCloud2, readTrajCloud)
    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, None, queue_size=10) # Publisher for commanding robot motion
    lplanner_pub = rospy.Publisher('/move_base/DWAPlannerROS/local_plan', Path, None, queue_size=10)
    gplanner_pub = rospy.Publisher('/move_base/DWAPlannerROS/global_plan', Path, None, queue_size=10)
    fullplan_pub = rospy.Publisher('/move_base/NavfnROS/plan', Path, None, queue_size=10)
    global G
    G =list()
    initMap()

    global path
    path = list()
    global openSet
    global closedSet

    global visited
    visited = list()
    global frontier
    frontier = list()

    openSet = list()
    openSet.append(G[start])
    closedSet = list()

    print "Astar beginning!"

    print len(openSet)

    while (openSet):
            i = lowestInQueue(openSet)
            current = G[i]
            if (current in frontier):
                frontier.remove(current)

            if(current.index == goalIndex):
                print reconstruct_path(current, G[start])
                return reconstruct_path(current, start)
                pass

            openSet.remove(current)
            closedSet.append(current)
            adjCellList = adjCellCheck(current)
            if adjCellList:
                for node in adjCellList:
                    if node not in closedSet:
                        frontier.append(node)
                        publishFrontier(frontier)
    print "goal not reachable"
    G.clear()

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