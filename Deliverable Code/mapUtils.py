
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point, Twist, Pose
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from tf.transformations import euler_from_quaternion
from math import sin, cos, sqrt

##
# Publishes a grid visualization
# @param publisher The publisher to use
# @param cells The set of cells to highlight, referenced within the rows and columns
#              of an occupancy grid as a list: [[row,column],[row,column], etc...]
# @param resolution The map resolution
# @param origin The map origin pose
def pubGrid(publisher, cells, resolution, origin):
    
    gridMessage = GridCells()
    gridMessage.header.frame_id = 'map'
    gridMessage.cell_width=resolution
    gridMessage.cell_height=resolution
    theta = euler_from_quaternion([origin.orientation.x,origin.orientation.y,origin.orientation.z,origin.orientation.w])[2]
    cosine = cos(theta)
    sine = sin(theta)

    for cell in cells:
        localX = cell[1]*resolution+resolution/2
        localY = cell[0]*resolution+resolution/2
        
        x = localX*cosine-localY*sine
        y = localX*sine+localY*cosine
        
        x += origin.position.x
        y += origin.position.y
        
        gridMessage.cells.append(Point(x,y,0))

    publisher.publish(gridMessage)

##
# Gets the linear distance between two int points
# @param pointA The first point as a tuple of two ints
# @param pointB The second point as a tuple of two ints
# @return The linear distance between the two input points
def intLinearDistance(pointA, pointB):
    return sqrt((pointA[0]-pointB[0])*(pointA[0]-pointB[0])+(pointA[1]-pointB[1])*(pointA[1]-pointB[1]))

##
# Converts an occupancy grid into a two dimensional list
# @param occupancyGrid The grid to convert
# @return The occupancyGrid converted to a list of row lists
def mapToList(occupancyGrid):
    gridList = []
    
    # For each row
    for rowNumber in range(occupancyGrid.info.height):
        # Insert a row list
        gridList.append([])
        for columnNumber in range(occupancyGrid.info.width):
            # Index off the first element of the row, adding each element to the row list
            gridList[rowNumber].append(occupancyGrid.data[columnNumber + rowNumber*occupancyGrid.info.width])
            
    return gridList

##
# Converts a two dimensional list into an occupancy grid
# @param listMap The list to convert
# @param header The header to use
# @param metadata The map metadata to use
# @return The list of row lists converted to an occupancyGrid
def listToMap(listMap, header, metadata):
    grid = OccupancyGrid()
    grid.info = metadata
    grid.header = header
    grid.header.frame_id='/map'
    
    # For each row
    for rowNumber in range(len(listMap)):
        # For each column
        for columnNumber in range(len(listMap[rowNumber])):
            grid.data.append(listMap[rowNumber][columnNumber])
            
    return grid

##
# Converts a dimensioned pose in meters into the occupancy grid (dimensionless) universe
# @param pose The pose to convert
# @param resolution The resolution of the map
# @param origin The map origin pose
# @return The pose converted into the nearest occupancy grid location in tuple format: (row, column)
def poseToMapCoordinates(pose, resolution, origin):
    x = pose.position.x - origin.position.x
    y = pose.position.y - origin.position.y
    
    theta = euler_from_quaternion([origin.orientation.x,origin.orientation.y,origin.orientation.z,origin.orientation.w])[2]
    
    localX = x*cos(-theta) - y*sin(-theta)
    localY = x*sin(-theta) + y*cos(-theta)
    
    localX = int(round((localX - resolution/2)/resolution))
    localY = int(round((localY - resolution/2)/resolution))
    
    return (localY, localX)

##
# Converts a pose from the map universe into the real world
# @param coords The map coordinates to convert
# @param resolution The resolution of the map
# @param origin The map origin pose
# @return The pose converted into a real-world pose
def mapToPoseCoordinates(coords, resolution, origin):
    
    localX = coords[1]*resolution+resolution/2
    localY = coords[0]*resolution+resolution/2
    
    theta = euler_from_quaternion([origin.orientation.x,origin.orientation.y,origin.orientation.z,origin.orientation.w])[2]
        
    x = localX*cos(theta)-localY*sin(theta)
    y = localX*sin(theta)+localY*cos(theta)
    
    x += origin.position.x
    y += origin.position.y

    pose = PoseStamped()
    pose.pose.position.x = x;
    pose.pose.position.y = y;

    return pose

