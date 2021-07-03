#! /usr/local/bin/python3
import zmq
import time
import random

import base64
import numpy as np
import cv2
from PIL import Image
from io import BytesIO
import _thread
from drive import *
from stanley_controller import *
import json


context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://localhost:12346")

TIMEOUT = 10000
def make_matrix(roll,pitch,heading,x0,y0,z0):
    a = np.radians(roll)
    b = np.radians(pitch)
    g = np.radians(heading)

    T = np.array([[ np.cos(b)*np.cos(g), (np.sin(a)*np.sin(b)*np.cos(g) + np.cos(a)*np.sin(g)), (np.sin(a)*np.sin(g) - np.cos(a)*np.sin(b)*np.cos(g)), x0],
    [-np.cos(b)*np.sin(g), (np.cos(a)*np.cos(g) - np.sin(a)*np.sin(b)*np.sin(g)), (np.sin(a)*np.cos(g) + np.cos(a)*np.sin(b)*np.sin(g)), y0],
    [ np.sin(b), -np.sin(a)*np.cos(b), np.cos(a)*np.cos(b), z0],
    [ 0, 0, 0, 1]])
    return T
def Rz(theta):
    return np.matrix([[ np.cos(theta), -np.sin(theta)],
                   [ np.sin(theta), np.cos(theta) ],
                  ])
def draw_map( map_png,image,pitch, Lat, Lon,global_path):
    global result
    
    demo = np.copy(map_png)
    x, y = int((Lon + 100 - Lon_Or)*map_png.shape[1]/(106.66226 - 106.65644)),int((Lat_Or - Lat)*map_png.shape[0]/(10.77654 - 10.77023))

    # map_png = cv2.circle(map_png, ( int((Lon - Lon_Or + 100)*map_png.shape[1]/(106.66226 - 106.65644)),int((Lat_Or - Lat)*map_png.shape[0]/(10.77654 - 10.77023)) ), radius=1, color=(0, 0, 255), thickness=-1)
    p1,p2,p3,p4 = get_map_pos(pitch, x, y)
    # p5, r_p5 = get_map_loc(pitch, x, y, 3, 14)
    # print(r_p5[0] - x, r_p5[1] - y)
    demo = cv2.circle(demo,p1, radius=1, color=(0, 0, 255), thickness=-1)
    demo = cv2.circle(demo,p2, radius=1, color=(0, 0, 255), thickness=-1)
    demo = cv2.circle(demo,(x,y), radius=1, color=(0, 0, 255), thickness=-1)
    # cv2.line(demo, (x,y), (100,100), (0, 255, 0), thickness=1)
    demo = cv2.circle(demo,p3, radius=1, color=(0, 0, 255), thickness=-1)
    demo = cv2.circle(demo,p4, radius=1, color=(0, 0, 255), thickness=-1)
    map_path_x = []
    map_path_y = []
    global_path_distance = []
    global_path_inframe = []
    # for p in global_path:
    #     # print(p)
    #     dis_to_point = ConvertGPStoUCS(Lat,Lon+100,p[0], p[1])
    #     global_path_distance.append(dis_to_point)
    #     (x_,y_) = global2pix(p[0],p[1]-100,Lat_Or,Lon_Or)

    #     if (dis_to_point[0] > -3 and dis_to_point[0] < 3):
    #         if (dis_to_point[1] > 0 and dis_to_point[0] < 3):
    #             global_path_inframe.append(dis_to_point)
    #     # print(x,y)
    #     map_path_x.append(x_)
    #     map_path_y.append(y_)
    #     demo = cv2.circle(demo,(x_,y_), radius=1, color=(0, 0, 255), thickness=-1)
    # print(global_path_distance)
    # state = State(x=x, y=y, yaw = pitch, v=0.0)
    # # # print(x,y)
    # target_idx, _ = calc_target_index(state, map_path_x, map_path_y)
    # if(target_idx < len(map_path_x)):
    #     global_angle = np.arctan((global_path[target_idx][0] - global_path[target_idx + 1][0])/(global_path[target_idx][1] - global_path[target_idx + 1][1] ))
    # print(target_idx)
    # print(global_angle*180/np.pi)
    # print(pitch - global_angle*180/np.pi)
    # demo = cv2.circle(demo,(map_path_x[target_idx],map_path_y[target_idx]), radius=3, color=(0, 255, 0), thickness=-1)
    # # demo = cv2.circle(demo,p5, radius=1, color=(0, 0, 255), thickness=-1)
    # # k,l = 0,0
    # # for i in range(p1[0],p4[0],1):
    # #     for j in range(p1[1],p4[1],1):
    # #         k += 1
    # #         l += 1

    # #         # demo[i][j] = result[k][l]
    resized = cv2.resize(demo[(y - 100):(y+100),(x-100):(x+100)], (400,400), interpolation = cv2.INTER_AREA)

    


    cv2.imshow("map1",resized)
    cv2.imshow("map",demo)
    
    # print("  Distance: ",  ConvertGPStoUCS(Lat_Or,Lon_Or,Lat, Lon+100) )

    # The black region in the mask has the value of 0,
    # so when multiplied with original image removes all non-blue regions
    
    # cv2.imshow("map_o",occupancy_map)
    # print(occupancy_map)
    # time.sleep(0.5)

    # cv2.imshow("im",image)

    # # warped_img = cv2.warpPerspective(image, M, (IMAGE_W, IMAGE_H)) # Image warping
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # # Threshold of blue in HSV space

    lower_blue = np.array([0, 0, 0])
    upper_blue = np.array([100, 100, 100])
 
    # preparing the mask to overlay
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    result_mask = cv2.bitwise_and(image, image, mask = mask)   
def occu_map(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # # Threshold of blue in HSV space

    lower_blue = np.array([0, 0, 0])
    upper_blue = np.array([100, 100, 100])
 
    # preparing the mask to overlay
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    mask = np.fliplr(mask)
    # mask = np.flipud(mask)
    # result_mask = cv2.bitwise_and(image, image, mask = mask)   
    return np.array(255 - mask,dtype=np.int8)
def send_control(raw,angle,velocity):
    raw.write(b's')
    raw.write(bytes([int(angle / 10)]))
    raw.write(bytes([angle % 10]))
    raw.write(bytes([velocity]))
    raw.write(bytes([(velocity + angle) % 37]))
    raw.write(b'e')    

connected = False
# raw.open()
class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s

    def readline(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        while True:
            i = max(1, min(2048, self.s.in_waiting))
            data = self.s.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)
map_png = cv2.imread("./map_png.PNG")
def zmq_connect(v, o):
    global socket,map_png
    arr = [0,0,0,0,0,0]
    socket.send_string(str(v) + ","+str(o))
    poller = zmq.Poller()
    poller.register(socket, zmq.POLLIN)
    evt = dict(poller.poll(TIMEOUT))
    if evt:
        if evt.get(socket) == zmq.POLLIN:
            response = socket.recv(zmq.NOBLOCK)
            arr = response.split()	
            image = Image.open(BytesIO(base64.b64decode(arr[0])))
            image = np.asarray(image)   

                # bird_image = Image.open(BytesIO(base64.b64decode(arr[6])))
                # bird_image = np.asarray(bird_image)   
            # cv2.imshow("im",image)
            occu = occu_map(image)
            
            # _thread.start_new_thread( draw_map, (map_png,image,float(arr[4])*180/np.pi, float(arr[1]), float(arr[2]),global_path) )
                #_thread.start_new_thread( draw_map, ("Thread-2", image,float(arr[1]), float(arr[2]),float(arr[4])*180/np.pi,global_path) )
            # print(arr[1], float(arr[2])+ 100, arr[3], arr[4], arr[5],arr[6], arr[7])
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break
                # print(response)
                
        # time.sleep(0.1)
    socket.close()

    socket = context.socket(zmq.REQ)
    socket.connect("tcp://localhost:12346")
    return occu, arr[1], arr[2], arr[3], arr[4], arr[5],arr[6], arr[7]


# while True:
#     zmq_connect(0,0)
#     # cv2.imshow("im",map_png)
    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break
                
                
    
#     socket.send_string("request")
#     poller = zmq.Poller()
#     poller.register(socket, zmq.POLLIN)
#     evt = dict(poller.poll(TIMEOUT))
#     if evt:
#         if evt.get(socket) == zmq.POLLIN:
#             response = socket.recv(zmq.NOBLOCK)
#             arr = response.split()	
#             image = Image.open(BytesIO(base64.b64decode(arr[0])))
#             image = np.asarray(image)   
#             # cv2.imshow("im",image)

#             _thread.start_new_thread( draw_map, ("Thread-1",float(arr[4]), float(arr[1]), float(arr[2]) ) )

#             print(arr[1], arr[2], arr[3])
#             # cv2.waitKey(0)
#            # print(response)
#             continue
#     time.sleep(0.5)
#     socket.close()
#     socket = context.socket(zmq.REQ)
#     socket.connect("tcp://localhost:12346")
