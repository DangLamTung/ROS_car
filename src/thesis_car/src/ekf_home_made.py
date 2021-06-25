#!/usr/bin/env python
"""
Extended kalman filter (EKF) localization sample
author: Atsushi Sakai (@Atsushi_twi)
"""

import math

import rospy
import numpy as np
# from scipy.spatial.transform import Rotation as Rot
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg  import NavSatFix
# Covariance for EKF simulation
Q = np.diag([
    0.1,  # variance of location on x-axis
    0.1,  # variance of location on y-axis
    np.deg2rad(1.0),  # variance of yaw angle
    1.0  # variance of velocity
]) ** 2  # predict state covariance
R = np.diag([1.0, 1.0]) ** 2  # Observation x,y position covariance
    # State Vector [x y yaw v]'
xEst = np.zeros((4, 1))

PEst = np.eye(4)

xDR = np.zeros((4, 1))  # Dead reckoning

Lon_Or = 106.65644
Lat_Or = 10.77654

LatOrigin = 10.773390000000006
LonOrigin = 106.65971499999999

gps_warmed = False
global_path = []
def FindMetersPerLat(lat): # Compute lengths of degrees
	
	m1 = 111132.92
	m2 = -559.82
	m3 = 1.175
	m4 = -0.0023
	p1 = 111412.84
	p2 = -93.5
	p3 = 0.118

	lat = np.deg2rad(lat)
	# Calculate the length of a degree of latitude and longitude in meters
	metersPerLat = m1 + (m2 * np.cos(2 * lat)) + (m3 * np.cos(4 * lat)) + (m4 * np.cos(6 * lat))
	metersPerLon = (p1 * np.cos(lat)) + (p2 * np.cos(3 * lat)) + (p3 * np.cos(5 * lat))
	return metersPerLat,metersPerLon
def ConvertGPStoUCS(_LatOrigin,_LonOrigin,Lat, Lon):
	metersPerLat,metersPerLon =  FindMetersPerLat(_LatOrigin)
	yPosition  = metersPerLat * (Lat - _LatOrigin)
	xPosition  = metersPerLon * (Lon - _LonOrigin)
	return xPosition, yPosition

def ConvertUCStoGPS(LatOrigin,LonOrigin,x, y):
	metersPerLat,metersPerLon =  FindMetersPerLat(LatOrigin)
	Lat_dis = ((LatOrigin + (y) / metersPerLat))
	Lon_dis = ((LonOrigin + (x) / metersPerLon))
	return Lat_dis,Lon_dis
 


def motion_model(x, u):
    F = np.array([[1.0, 0, 0, 0],
                  [0, 1.0, 0, 0],
                  [0, 0, 1.0, 0],
                  [0, 0, 0, 0]])

    B = np.array([[DT * math.cos(x[2, 0]), 0],
                  [DT * math.sin(x[2, 0]), 0],
                  [0.0, DT],
                  [1.0, 0.0]])

    x = np.dot(F, x) + np.dot(B, u)

    return x


def observation_model(x):
    H = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])

    z = np.dot(H,x)

    return z


def jacob_f(x, u):
    """
    Jacobian of Motion Model
    motion model
    x_{t+1} = x_t+v*dt*cos(yaw)
    y_{t+1} = y_t+v*dt*sin(yaw)
    yaw_{t+1} = yaw_t+omega*dt
    v_{t+1} = v{t}
    so
    dx/dyaw = -v*dt*sin(yaw)
    dx/dv = dt*cos(yaw)
    dy/dyaw = v*dt*cos(yaw)
    dy/dv = dt*sin(yaw)
    """
    yaw = x[2, 0]
    v = u[0, 0]
    jF = np.array([
        [1.0, 0.0, -DT * v * math.sin(yaw), DT * math.cos(yaw)],
        [0.0, 1.0, DT * v * math.cos(yaw), DT * math.sin(yaw)],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]])

    return jF


def jacob_h():
    # Jacobian of Observation Model
    jH = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])

    return jH
def ekf_odom_predict(xEst, PEst, u):
        #  Predict
    xPred = motion_model(xEst, u)
    jF = jacob_f(xEst, u)
    PPred = np.dot(np.dot(jF,PEst),jF.T) + Q
    return xEst, PEst

def ekf_gps_update(xEst, PEst, z):
 
    jH = jacob_h()
    zPred = observation_model(xEst)
    y = z - zPred
    S = np.dot(np.dot(jH,PEst),jH.T) + R
    K = np.dot(np.dot( PEst,jH.T), np.linalg.inv(S))
    xEst = xEst + np.dot(K , y)
    PEst = np.dot((np.eye(len(xEst)) - np.dot(K, jH)), PEst)
    return xEst, PEst

# def ekf_estimation(xEst, PEst, z, u):
#     #  Predict
#     xPred = motion_model(xEst, u)
#     jF = jacob_f(xEst, u)
#     PPred = jF @ PEst @ jF.T + Q

#     #  Update
#     jH = jacob_h()
#     zPred = observation_model(xPred)
#     y = z - zPred
#     S = jH @ PPred @ jH.T + R
#     K = PPred @ jH.T @ np.linalg.inv(S)
#     xEst = xPred + K @ y
#     PEst = (np.eye(len(xEst)) - K @ jH) @ PPred
#     return xEst, PEst


# def plot_covariance_ellipse(xEst, PEst):  # pragma: no cover
#     Pxy = PEst[0:2, 0:2]
#     eigval, eigvec = np.linalg.eig(Pxy)

#     if eigval[0] >= eigval[1]:
#         bigind = 0
#         smallind = 1
#     else:
#         bigind = 1
#         smallind = 0

#     t = np.arange(0, 2 * math.pi + 0.1, 0.1)
#     a = math.sqrt(eigval[bigind])
#     b = math.sqrt(eigval[smallind])
#     x = [a * math.cos(it) for it in t]
#     y = [b * math.sin(it) for it in t]
#     angle = math.atan2(eigvec[1, bigind], eigvec[0, bigind])
#     rot = Rot.from_euler('z', angle).as_matrix()[0:2, 0:2]
#     fx = rot @ (np.array([x, y]))
#     px = np.array(fx[0, :] + xEst[0, 0]).flatten()
#     py = np.array(fx[1, :] + xEst[1, 0]).flatten()
#     plt.plot(px, py, "--r")

def odom_callback(data):
    global xEst, PEst
    # xEst, PEst = ekf_odom_predict(xEst, PEst, z)
    return

def imu_callback(data):
    return

def gps_callback(data):
    global xEst, PEst, LatOrigin,LonOrigin,gps_warmed
    if(data.position_covariance[0] > 2.5):
        print("sensor warming")
    else:
        if(not gps_warmed):
            gps_warmed = True
            LatOrigin,LonOrigin = data.latitude,data.longitude
            print("sensor warmed, gps origin:", LatOrigin,LonOrigin)
        else:
            x, y = ConvertGPStoUCS(LatOrigin,LonOrigin,data.latitude,data.longitude)
            print("distance from origin:",x,y)
            xEst, PEst = ekf_gps_update(xEst, PEst, np.array([x,y]))
def main():
    rospy.init_node('ekf_home_made')
    rospy.Subscriber("odom", Odometry, odom_callback)
    rospy.Subscriber("imu", Imu, imu_callback)
    rospy.Subscriber("fix", NavSatFix, gps_callback)
    rate = rospy.Rate(10) # 10hz
    
    # odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    time = 0.0
    rospy.spin()




if __name__ == '__main__':
    main()
    