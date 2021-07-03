#!/usr/bin/env python

# Author: christoph.roesmann@tu-dortmund.de

import rospy, math
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import PolygonStamped, Point32
import numpy as np
pub_obs = rospy.Publisher('/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, queue_size=1)

def publish_obstacle_msg(mask):
  global pub_obs 
  # pub = rospy.Publisher('/test_optim_node/obstacles', ObstacleArrayMsg, queue_size=1)
  



  obstacle_msg = ObstacleArrayMsg() 
  obstacle_msg.header.stamp = rospy.Time.now()
  obstacle_msg.header.frame_id = "base_link" # CHANGE HERE: odom/map
  
 

  t = 0.0
  temp = ObstacleMsg()


  occupancy_map = np.zeros((8,16))
  for i in range(8):
    for j in range(16):
      if( (mask[i*20:(i*20+ 20),j*20:(j*20 + 20)] == 0).sum( ) <320): #$80% map
        temp = ObstacleMsg()        
        temp.id = i*8+j
        temp.polygon.points = [Point32()]
        temp.polygon.points[0].x = .3*i +1
        temp.polygon.points[0].y =  0.3*j -2
        temp.polygon.points[0].z = 0
        obstacle_msg.obstacles.append(temp)
        occupancy_map[i][j] = 1

    # Vary y component of the point obstacle
    # obstacle_msg.obstacles[0].polygon.points[0].y = 0.5*math.sin(t)
    # t = t + 0.1
  # print(len(obstacle_msg.obstacles))  
  # print(occupancy_map)
  pub_obs.publish(obstacle_msg)
    
    # r.sleep()



# if __name__ == '__main__': 
#   try:
#     publish_obstacle_msg()
#   except rospy.ROSInterruptException:
#     pass
