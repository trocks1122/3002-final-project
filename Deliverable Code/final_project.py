import rospy
from geometry_msgs.msg import PoseStamped, Point, PoseWithCovarianceStamped
from nav_msgs.msg import GridCells, OccupancyGrid
from move_base_msgs.msg import MoveBaseActionResult
import tf
from threading import Lock
import random
from math import sin, cos
from mapUtils import *
from collections import deque
from time import sleep
from copy import deepcopy

ROBOT_RADIUS = 0.2

##
# Handles debugging intial pose events
# @param msg The debugging pose estimate
def initialPoseHandler(msg):
    global freshPose
    global pose
    pose = msg.pose.pose
    freshPose = True
    

##
# Handles map event
# @param msg The map
def mapHandler(msg):
    global freshMap
    global grid
    print "Handling map"
    
    grid = msg
    freshMap = True
    
##
# Handles map event
# @param msg The map
def costMapHandler(msg):
    global freshCostMap
    global costGrid
    print "Handling cost map"
    costGrid = msg
    freshCostMap = True

##
# Handles navigation status events
# @param msg The goal status message
def statusHandler(msg):
    global freshStatus
    global goalFailed
    global goalReached
    
    if msg.status.status == 4 or msg.status.status == 5 or msg.status.status == 8:
        if not goalFailed and not goalReached:
            print "Goal failed."
            goalFailed = True
            goalReached = False
            freshStatus = True
    elif msg.status.status == 3:
        if not goalReached and not goalFailed:
            print "Goal reached."
            goalFailed = False
            goalReached = True
            freshStatus = True

##
# Gets the frontiers from an occupancy grid
# @param occupancyGrid The occupancy grid in which to find frontiers
# @param pose The pose of the robot
# @param progressPub The GridCell publisher on which to publish wavefront progress (for debugging)
# @param resultPub The GridCell publisher on which to publish the result (for debugging)
# @return A list of lists of points in the occupancy grid which correspond to frontiers.
def getFrontiers(occupancyGrid, radius, pose, progressPub, resultPub):
    msg = occupancyGrid
    
    # Create a list of offsets for cells within ROBOT_RADIUS of a given cell
    radius = int(round(radius/msg.info.resolution))
    offsetTable = []
    row = -radius
    while row <= radius:
        height = int(round(sqrt(radius*radius-row*row)))
        column = -height
        while column <= height:
            offsetTable.append((row,column))
            column += 1
        row += 1
    
    rawMapList = mapToList(msg)
    mapList = deepcopy(rawMapList)
    
    # For each map cell
    # If this cell is within ROBOT_RADIUS of an obstacle, it too is an obstacle
    for rowNum in range(len(rawMapList)):
        for colNum in range(len(rawMapList[rowNum])):
            if rawMapList[rowNum][colNum] == 100:
                for offset in offsetTable:
                    rowOffset = rowNum + offset[0]
                    colOffset = colNum + offset[1]
                    if rowOffset >= 0 and rowOffset < len(rawMapList) and colOffset >= 0 and colOffset < len(rawMapList[0]):
                        mapList[rowOffset][colOffset] = 100

    expandedGrid = listToMap(mapList, msg.header, msg.info)
    
    expandedPub.publish(expandedGrid)

    gridList = mapToList(expandedGrid)
    
    location = poseToMapCoordinates(pose, occupancyGrid.info.resolution, occupancyGrid.info.origin)
    
    visited = [[False for cell in gridList[0]] for row in gridList]
    
    
    inFrontier = [[False for cell in gridList[0]] for row in gridList]
    
    wave = deque()
    wave.append(location)
    visited[location[0]][location[1]] = True
    
    i=0
    foundOpenSpace = False;
    
    # First, find every reachable point that falls on a boundary between the map and unknown
    while not rospy.is_shutdown() and len(wave) > 0:
        
        
        # Pop the last point in the wave
        point = wave.popleft()
        
        # Compute the neighbors of this point
        neighbors = [(point[0]+1, point[1]),
                     (point[0], point[1]+1),
                     (point[0]-1, point[1]),
                     (point[0], point[1]-1),
                     (point[0]-1, point[1]-1),
                     (point[0]+1, point[1]+1),
                     (point[0]+1, point[1]-1),
                     (point[0]-1, point[1]+1)]
        
        isDiagonal = [0,0,0,0,1,1,1,1]
        
        addPoint = False
        
        
        for index, neighbor in enumerate(neighbors):
            # If the neighbor is within the map bounds:
            if neighbor[0] >= 0 and neighbor[0] < len(gridList) and neighbor[1] >= 0 and neighbor[1] < len(gridList[0]):
                # If any of the non-diagonal neighbors is in the unknown, then we are at a frontier point.
                if gridList[neighbor[0]][neighbor[1]] == -1 and not isDiagonal[index]:
                    addPoint = True
                # If the neighbor is an obstacle, then it should not be explored again.
                # If it is open space, and has not been visited (and is not in the wave), it should be added to the wave
                # In the special case that a neighbor is diagonal and falls "between" two obstacles, it should be treated as if it were an obstacle
                threshold = 95
                
                limit = ((neighbor[0] != 0 and neighbor[0] != len(gridList)-1 and neighbor[1] != 0 and neighbor[1] < len(gridList[0])-1) and
                        ((neighbor==(point[0]-1,point[1]-1) and gridList[point[0]-1][neighbor[1]] >= threshold and gridList[neighbor[0]][neighbor[1]-1] >= threshold) or
                         (neighbor==(point[0]+1,point[1]+1) and gridList[point[0]][neighbor[1]+1] >= threshold and gridList[neighbor[0]+1][neighbor[1]] >= threshold) or
                         (neighbor==(point[0]+1,point[1]-1) and gridList[point[0]+1][neighbor[1]] >= threshold and gridList[neighbor[0]][neighbor[1]-1] >= threshold) or
                         (neighbor==(point[0]-1,point[1]+1) and gridList[point[0]][neighbor[1]+1] >= threshold and gridList[neighbor[0]-1][neighbor[1]] >= threshold)));
                
                if (gridList[neighbor[0]][neighbor[1]] != -1 and not isDiagonal[index] and (gridList[neighbor[0]][neighbor[1]] < threshold) and not visited[neighbor[0]][neighbor[1]] and not limit):
                    foundOpenSpace = True
                    wave.append(neighbor)
                    visited[neighbor[0]][neighbor[1]] = True
                    
                if not foundOpenSpace:
                    wave.append(neighbor)
                    visited[neighbor[0]][neighbor[1]] = True
        
        if addPoint and not inFrontier[point[0]][point[1]]:
            inFrontier[point[0]][point[1]] = True
        
        i+=1
        if i%100 == 0:
            if progressPub is not None:
                pubGrid(progressPub, wave, occupancyGrid.info.resolution, occupancyGrid.info.origin)
    
    
    frontierLookup = [[None for cell in gridList[0]] for row in gridList]
    frontiers = []
    
    # Now, group points into subfrontiers
    for row in range(len(inFrontier)):
        for col in range(len(inFrontier[row])):
            if inFrontier[row][col]:
                # If the current point is in the frontier, but not assigned to a subfrontier, find its neighbors
                neighbors = [(row+1, col),
                             (row, col+1),
                             (row-1, col),
                             (row, col-1),
                             (row-1, col-1),
                             (row+1, col+1),
                             (row+1, col-1),
                             (row-1, col+1)]
                             
                # If any neighbor is in a subfrontier already, add that neighbor to a list
                attachedFrontiers = []
                for neighbor in neighbors:
                    if neighbor[0] >= 0 and neighbor[0] < len(inFrontier) and neighbor[1] >= 0 and neighbor[1] < len(inFrontier[0]):
                        if frontierLookup[neighbor[0]][neighbor[1]] is not None:
                            if not frontierLookup[neighbor[0]][neighbor[1]] in attachedFrontiers:
                                attachedFrontiers.append(frontierLookup[neighbor[0]][neighbor[1]])
                
                # If our neighbors belong to different frontiers, then we need to join the frontiers.
                # Otherwise, we can just add ourselves to the single attached frontier
                if len(attachedFrontiers) >= 1:
                
                    if len(attachedFrontiers) != 1:
                        for index, frontier in enumerate(attachedFrontiers):
                            if index != 0:
                                frontiers.remove(frontier)
                                for point in frontier:
                                    attachedFrontiers[0].append(point)
                                    frontierLookup[point[0]][point[1]] = attachedFrontiers[0]
                        
                    attachedFrontiers[0].append((row,col))
                    frontierLookup[row][col] = attachedFrontiers[0]
                # If none of the neighbors is already in a frontier, create a new frontier for this point
                else:
                    newFrontier = [(row,col)]
                    frontierLookup[row][col] = newFrontier
                    frontiers.append(newFrontier)
            
            
    totalFrontier = []
    for frontier in frontiers:
        for point in frontier:
            totalFrontier.append(point)
             
    if resultPub is not None:
        pubGrid(resultPub, totalFrontier, occupancyGrid.info.resolution, occupancyGrid.info.origin)
        
    rospy.sleep(1.0)
    
    return frontiers



##
# Given a target that cannot be reached, finds a nearby target that is likely
# to be reachable by searching a more restrictive occupancy grid.
# @param occupancyGrid The expanded occupancy grid to use
# @param target The target point (in map coordinates)
# @param pose The pose of the robot
# @return The new target, which is more likely to be reachable
def getReachableTarget(occupancyGrid, target, pose):
    gridList = mapToList(occupancyGrid)
    
    location = poseToMapCoordinates(pose, occupancyGrid.info.resolution, occupancyGrid.info.origin)
    
    visited = [[False for cell in gridList[0]] for row in gridList]
    
    wave = deque()
    wave.append(location)
    visited[location[0]][location[1]] = True
    
    i=0
    
    # First, find every reachable point
    while not rospy.is_shutdown() and len(wave) > 0:
        
        
        # Pop the last point in the wave
        point = wave.popleft()
        
        # Compute the neighbors of this point
        neighbors = [(point[0]+1, point[1]),
                     (point[0], point[1]+1),
                     (point[0]-1, point[1]),
                     (point[0], point[1]-1),
                     (point[0]-1, point[1]-1),
                     (point[0]+1, point[1]+1),
                     (point[0]+1, point[1]-1),
                     (point[0]-1, point[1]+1)]
        
        addPoint = False
        
        for neighbor in neighbors:
            # If the neighbor is within the map bounds:
            if neighbor[0] >= 0 and neighbor[0] < len(gridList) and neighbor[1] >= 0 and neighbor[1] < len(gridList[0]):
                
                # If any of the neighbors is in the unknown, then we are at a frontier point or a point that is next to a frontier point; neither of these points
                # is likely to be reachable
                if gridList[neighbor[0]][neighbor[1]] == -1:
                    pass
                # If the neighbor is an obstacle, then it should not be explored again.
                # If it is open space, and has not been visited (and is not in the wave), it should be added to the wave
                # In the special case that a neighbor is diagonal and falls "between" two obstacles, it should be treated as if it were an obstacle
                threshold = 70
                
                limit = ((neighbor[0] != 0 and neighbor[0] != len(gridList)-1 and neighbor[1] != 0 and neighbor[1] < len(gridList[0])-1) and
                        ((neighbor==(point[0]-1,point[1]-1) and gridList[point[0]-1][neighbor[1]] >= threshold and gridList[neighbor[0]][neighbor[1]-1] >= threshold) or
                         (neighbor==(point[0]+1,point[1]+1) and gridList[point[0]][neighbor[1]+1] >= threshold and gridList[neighbor[0]+1][neighbor[1]] >= threshold) or
                         (neighbor==(point[0]+1,point[1]-1) and gridList[point[0]+1][neighbor[1]] >= threshold and gridList[neighbor[0]][neighbor[1]-1] >= threshold) or
                         (neighbor==(point[0]-1,point[1]+1) and gridList[point[0]][neighbor[1]+1] >= threshold and gridList[neighbor[0]-1][neighbor[1]] >= threshold)));
                
                if gridList[neighbor[0]][neighbor[1]] != -1 and (gridList[neighbor[0]][neighbor[1]] < threshold) and not visited[neighbor[0]][neighbor[1]] and not limit:
                    wave.append(neighbor)
                    visited[neighbor[0]][neighbor[1]] = True
        
        if addPoint and not inFrontier[point[0]][point[1]]:
            inFrontier[point[0]][point[1]] = True
        
    bestDistance = float("inf")
    bestRow = target[0];
    bestColumn = target[1];
    
    # Now, search for the closest reachable point to our target
    for i, row in enumerate(visited):
        for j, cell in enumerate(row):
            if cell:
                newDistance = sqrt((i - target[0])*(i - target[0]) + (j - target[1])*(j - target[1]))
                if newDistance < bestDistance:
                    bestDistance = newDistance
                    bestRow = i
                    bestColumn = j
    
    return (bestRow, bestColumn)

##
# Called repeatedly by rospy at an interval
# @param event Timing information about the timer callback
def timerCallback(event):
    global pose
    global freshPose

    odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))

    (position, orientation) = odom_list.lookupTransform('map','base_footprint', rospy.Time(0))

    roll, pitch, yaw = euler_from_quaternion([orientation[0], orientation[1], orientation[2], orientation[3]])
    
    with threadLock:
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.orientation.z = yaw
        freshPose = True
        
##
# Main block
#
if __name__ == '__main__':
    global freshPose
    global freshMap
    global freshCostMap
    global pose
    global grid
    global costGrid
    global threadLock
    
    global freshStatus
    global goalFailed
    global goalReached
    global expandedPub
    
    
    threadLock = Lock()
    
    freshPose = False
    freshMap = False
    freshCostMap = False
    freshStatus = False
    
    goalFailed = False
    goalReached = False
    
    rospy.init_node('final_project_nav')
    
    odom_list = tf.TransformListener()
    
    

    # Repeating odometry callback
    timer = rospy.Timer(rospy.Duration(0.1), timerCallback)

    rospy.sleep(rospy.Duration(0.1))
    
    # Subscribe to the map and navigation status
    statusSub = rospy.Subscriber("/move_base/result", MoveBaseActionResult, statusHandler)
    
    # Subscribe to the initial pose from rviz for debugging
    initialPoseSub = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, initialPoseHandler)
    
    # Publish to the goal topic and a grid view for previewing frontiers
    goalPub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    frontierPub = rospy.Publisher('/frontiers', GridCells, queue_size=1)
    expandedPub = rospy.Publisher('/expanded', OccupancyGrid, queue_size=1)
    
    
    while (not freshPose) and not rospy.is_shutdown():
        pass
    freshPose = False

    # If this flag is set, the nav engine will be sent a target that is approximated as being reachable
    tryClosest=False
    
        
    while not rospy.is_shutdown():
            
        print "Waiting for map update..."
        
        rospy.sleep(1.0)
        costMapSub = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, costMapHandler)
        mapSub = rospy.Subscriber("/map", OccupancyGrid, mapHandler)
        
        while (not freshMap or not freshCostMap or not freshPose) and not rospy.is_shutdown():
            pass
            
        freshMap = False
        freshCostMap = False
        mapSub.unregister()
        costMapSub.unregister()
        
        with threadLock:
            localPose = pose
    
        print "Computing frontiers."
        # Sort the frontiers according to size
        if not tryClosest:
            frontiers_raw = getFrontiers(grid, ROBOT_RADIUS, localPose, None, frontierPub)
        else:
            frontiers_raw = getFrontiers(grid, grid.info.resolution, localPose, None, frontierPub)
            
        if len(frontiers_raw) == 0:
            print "Map Complete"
            break
            
        frontiers = sorted(frontiers_raw, key=lambda elt: len(elt), reverse=True)
        
        if len(frontiers[0]) <= 6:
            print "Map Complete"
            break
            
        # For each frontier, starting with the largest, attempt to drive to the centroid. If centroid cannot be reached, move
        # on to the next smallest frontier. Repeat this until two frontiers have been successfully reached, or no all frontiers have been attempted at least once.
        frontierNum = 0;
        print len(frontiers)
        i = 0
        j = 0
        centroids = []
        
        # Find the centroids of the frontiers
        for frontier in frontiers:
            sumRow = 0
            sumCol = 0
            for point in frontier:
                sumRow += point[0]
                sumCol += point[1]
            centroid = (sumRow/len(frontier), sumCol/len(frontier))
            centroids.append(centroid)
        
        
        for centroid in centroids:
        
            if(j > 4 and not tryClosest):
                tryClosest = True
                break
            
            with threadLock:
                localPose = pose
            
            if tryClosest:    
                reachableCentroid = getReachableTarget(costGrid, centroid, localPose)
                target = mapToPoseCoordinates(reachableCentroid, grid.info.resolution, grid.info.origin)
            else:
                target = mapToPoseCoordinates(centroid, grid.info.resolution, grid.info.origin)
                
            target.pose.orientation.x = 0
            target.pose.orientation.y = 0
            target.pose.orientation.z = 1
            target.pose.orientation.w = random.uniform(0.0, 0.99)
            
            target.header.frame_id = '/map'
            t = rospy.Time.now()
            target.header.stamp.secs = t.secs - 1
            target.header.stamp.nsecs = t.nsecs
            
            print "Publishing goal..."
            goalPub.publish(target)
            goalReached = False
            goalFailed = False
            print "Goal published."
            
            if tryClosest:
                print "Trying closest reachable point."
            
            while not rospy.is_shutdown():
                if freshStatus:
                    freshStatus = False
                    if goalFailed:
                        print "Trying new frontier."
                        break
                    elif goalReached:
                        i+=1;
                        break
                        
            if (i >= 2 or i >= len(centroids)) and not tryClosest:
                break
            if (i >= 1 or i >= len(centroids)) and tryClosest:
                tryClosest = False
                break
            if j >= 2:
                tryClosest = True
                break
            
            j += 1;
            
                    
