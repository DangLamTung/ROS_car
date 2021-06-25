# -*- coding: utf-8 -*-

import numpy as np

import rospy
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from bird_python import *
rospy.init_node("local_map")

map_topic = "map/local_map"
OG_publisher = rospy.Publisher(map_topic, OccupancyGrid, queue_size=5, latch=True)

MAP_RESOLUTION = 0.1    #Unit: Meter
MAP_RESOLUTION_H = 0.1    #Unit: Meter

MAP_SIZE       = 43     #Unit: Meter, Shape: Square with center "base_link"
MAP_SIZE_H       = 36     #Unit: Meter, Shape: Square with center "base_link"
# map_img = np.zeros(430,360,1], \
#                     dtype=np.uint8)

rate = rospy.Rate(10)      
while not rospy.is_shutdown():
    map_img = local_mapping(0)
    occupancy_grid = map_img.flatten()
    occupancy_grid = occupancy_grid.tolist()

    map_msg = OccupancyGrid()

    map_msg.header = Header()
    map_msg.header.frame_id = "base_link"
    map_msg.header.stamp    = rospy.Time.now()

    map_msg.info= MapMetaData()
    map_msg.info.map_load_time = rospy.Time.now()
    map_msg.info.height = 362       #Unit: Pixel
    map_msg.info.width  = 436     #Unit: Pixel
    map_msg.info.resolution = MAP_RESOLUTION

    map_msg.info.origin = Pose()
    map_msg.info.origin.position = Point()
    map_msg.info.origin.position.x = -MAP_SIZE/2      #Unit: Meter
    map_msg.info.origin.position.y = -MAP_SIZE/2      #Unit: Meter
    map_msg.info.origin.position.z = 0
    map_msg.info.origin.orientation = Quaternion()
    map_msg.info.origin.orientation.x = 0
    map_msg.info.origin.orientation.y = 0
    map_msg.info.origin.orientation.z = 0
    map_msg.info.origin.orientation.w = 1

    map_msg.data.extend(occupancy_grid)

    OG_publisher.publish(map_msg)
    
    rate.sleep()
