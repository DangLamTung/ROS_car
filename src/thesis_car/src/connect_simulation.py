#! /usr/bin/python3
import rospy

import math
from math import sin, cos, pi

from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

from zmq_ros import *
from occu_map import *
from visual_obstacles import *
v = 0
omega = 0


def callback(data):
    global v, omega
    # print(data.linear,data.angular)
    v = data.linear.x 

    if(abs(data.angular.z)< 25):
            
        omega = -data.angular.z*180/np.pi/25.0
    else:
        if(data.angular.z > 0):
            omega = -1
        else:
            omega = 1
    # send_uart(data.linear.x,data.angular.z)


def main():
    global v, omega
    rospy.init_node('simulation')
    # pub_imu = rospy.Publisher('imu', Imu, queue_size=10)
    rospy.Subscriber("cmd_vel", Twist, callback)
    rate = rospy.Rate(10) # 10hz

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()
    _,lat_origin,lon_origin,_, angle, _,x_origin,y_origin= zmq_connect(0,0)
    x_origin = float(x_origin)
    y_origin = float(y_origin)
    pre_x = pre_y = pre_angle= 0


    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()

    gps_pub = rospy.Publisher("/gps/fix", NavSatFix, queue_size=50)
    gps_pub_origin = rospy.Publisher("/gps/fix_origin", NavSatFix, queue_size=1)
    gps_msg_origin = NavSatFix()
    gps_msg_origin.header.stamp = current_time
    gps_msg_origin.header.frame_id = "base_link"

    gps_msg_origin.latitude = float(lat_origin)
    gps_msg_origin.longitude = float(lon_origin) + 100
    gps_msg_origin.altitude = 0
    gps_msg_origin.position_covariance = [1,0,0,0,2,0,0,0,1]
            
    
    while not rospy.is_shutdown():
        try:
            current_time = rospy.Time.now()
            occu,lat,lon,_, angle, _,x_map,y_map= zmq_connect(omega, v)
            x_map = float(x_map)
            y_map = float(y_map)

            x = x_map - x_origin
            y = y_map - y_origin

            print(omega*180/np.pi, v)
            # print(occu.shape)
            # x,y = ConvertGPStoUCS(float(lat_origin),float(lon_origin),float(lat), float(lon))
            angle = float(angle)
            # print(x,y, angle*180/np.pi)
                # since all odometry is 6DOF we'll need a quaternion created from yaw
            
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, -angle + np.pi/2) 
            # print(odom_quat)
            publish_map(occu,odom_quat,x,y)
            publish_obstacle_msg(occu)
                # first, we'll publish the transform over tf
            odom_broadcaster.sendTransform(
                (x, y, 0.),
                odom_quat,
                current_time,
                "base_link",
                "map"
            )

                # next, we'll publish the odometry message over ROS
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"

                # set the position
            odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

                # set the velocity
            vx = (x - pre_x)*10
            vy = (y - pre_y)*10
            vth = (angle - pre_angle)*10
            odom.child_frame_id = "base_link"
            odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0,0))
            pre_x,pre_y = x,y
            pre_angle = angle
                # publish the message

            gps_msg = NavSatFix()
            gps_msg.header.stamp = current_time
            gps_msg.header.frame_id = "base_link"

            gps_msg.latitude = float(lat)
            gps_msg.longitude = float(lon) + 100
            gps_msg.altitude = 0
            gps_msg.position_covariance = [1,0,0,0,2,0,0,0,1]
            
            gps_pub_origin.publish(gps_msg_origin)
            gps_pub.publish(gps_msg)
            odom_pub.publish(odom)
            last_time = current_time
        except Exception as e:
            print(e)
        rate.sleep()
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass