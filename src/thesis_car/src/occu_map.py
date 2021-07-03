# -*- coding: utf-8 -*-

import numpy as np

import rospy
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion



map_topic = "map/local_map"
OG_publisher = rospy.Publisher(map_topic, OccupancyGrid, queue_size=5, latch=True)

MAP_RESOLUTION = 0.1    #Unit: Meter
MAP_RESOLUTION_H = 0.1    #Unit: Meter

MAP_SIZE       = 16     #Unit: Meter, Shape: Square with center "base_link"
MAP_SIZE_H       = 32     #Unit: Meter, Shape: Square with center "base_link"
# map_img = np.zeros(430,360,1], \
#                     dtype=np.uint8)
# simulate an update with adding all of it again

   
def publish_map(map_img, angle,x,y):
    global OG_publisher 
    map_img = np.flipud(map_img)
    occupancy_grid = map_img.flatten()
    occupancy_grid = occupancy_grid.tolist()

    map_msg = OccupancyGrid()

    map_msg.header = Header()
    map_msg.header.frame_id = "base_link_cam"
    map_msg.header.stamp    = rospy.Time.now()

    map_msg.info= MapMetaData()
    map_msg.info.map_load_time = rospy.Time.now()
    map_msg.info.height = 160      #Unit: Pixel
    map_msg.info.width  = 320    #Unit: Pixel
    map_msg.info.resolution = MAP_RESOLUTION

    map_msg.info.origin = Pose()
    map_msg.info.origin.position = Point()
    map_msg.info.origin.position.x = -16    #Unit: Meter
    map_msg.info.origin.position.y = 0      #Unit: Meter
    map_msg.info.origin.position.z = 0
    map_msg.info.origin.orientation = Quaternion()
    map_msg.info.origin.orientation.x = 0
    map_msg.info.origin.orientation.y = 0
    map_msg.info.origin.orientation.z = 0
    map_msg.info.origin.orientation.w = 1

    # map_msg.info.origin = Pose()
    # map_msg.info.origin.position = Point()
    # map_msg.info.origin.position.x = x     #Unit: Meter
    # map_msg.info.origin.position.y = y     #Unit: Meter
    # map_msg.info.origin.position.z = 0

    # map_msg.info.origin.orientation = Quaternion()
    # map_msg.info.origin.orientation.x = angle[0]
    # map_msg.info.origin.orientation.y = angle[1]
    # map_msg.info.origin.orientation.z = angle[2]
    # map_msg.info.origin.orientation.w = angle[3]
    # ogu = OccupancyGridUpdate()
    # ogu.header = map_msg.header
    # ogu.header.stamp = rospy.Time.now()
    # ogu.x = ogu.y = -MAP_SIZE/2 
    # ogu.width = map_msg.info.width
    # ogu.height = map_msg.info.height
    # ogu.data = map_msg.data

    map_msg.data.extend(occupancy_grid)

    # update_pub.publish(ogu)
    OG_publisher.publish(map_msg)
    