import cv2
import numpy as np

import jetson.inference
import jetson.utils

import argparse
import sys
import cv2
from segnet_utils import *
from drive import *
import time

import memcache

shared = memcache.Client(['127.0.0.1:11211'], debug=0)

FRAME_HEIGHT = 1.90
FRAME_WIDTH = 1.60
# parse the command line
parser = argparse.ArgumentParser(description="Segment a live camera stream using an semantic segmentation DNN.", 
                                 formatter_class=argparse.RawTextHelpFormatter, epilog=jetson.inference.segNet.Usage() +
                                 jetson.utils.videoSource.Usage() + jetson.utils.videoOutput.Usage() + jetson.utils.logUsage())

parser.add_argument("input_URI", type=str, default="", nargs='?', help="URI of the input stream")
parser.add_argument("output_URI", type=str, default="", nargs='?', help="URI of the output stream")
parser.add_argument("--network", type=str, default="fcn-resnet18-cityscapes", help="pre-trained model to load, see below for options")
parser.add_argument("--filter-mode", type=str, default="linear", choices=["point", "linear"], help="filtering mode used during visualization, options are:\n  'point' or 'linear' (default: 'linear')")
parser.add_argument("--visualize", type=str, default="overlay,mask", help="Visualization options (can be 'overlay' 'mask' 'overlay,mask'")
parser.add_argument("--ignore-class", type=str, default="void", help="optional name of class to ignore in the visualization results (default: 'void')")
parser.add_argument("--alpha", type=float, default=150.0, help="alpha blending value to use during overlay, between 0.0 and 255.0 (default: 150.0)")
parser.add_argument("--stats", action="store_true", help="compute statistics about segmentation mask class output")

is_headless = ["--headless"] if sys.argv[0].find('console.py') != -1 else [""]

try:
	opt = parser.parse_known_args()[0]
except:
	print("")
	parser.print_help()
	sys.exit(0)

# load the segmentation network
net = jetson.inference.segNet(opt.network, sys.argv)

# set the alpha blending value
net.SetOverlayAlpha(opt.alpha)

# create buffer manager
buffers = segmentationBuffers(net, opt)

# create video sources & outputs
#input = jetson.utils.videoSource(opt.input_URI, argv=sys.argv)
output = jetson.utils.videoOutput(opt.output_URI, argv=sys.argv+is_headless)
# out = cv2.VideoWriter("./Map_dorm1.avi",cv2.VideoWriter_fourcc('M','J','P','G'),10,(640,480))
# out1 = cv2.VideoWriter("./Map_dorm_bird1.avi",cv2.VideoWriter_fourcc('M','J','P','G'),10,(640,480))
transformMat = np.loadtxt("./transform_file.txt", dtype=np.float32)
transformMat  = np.reshape(transformMat ,(3,3))

r_crop = np.loadtxt("./crop_img.txt", dtype=np.float32)
r_crop = np.reshape(r_crop ,(4,1))

road_graph = SquareGrid(6,5)
def nothing(x):
    pass

# # Create a black image, a window
# img = np.zeros((300,512,3), np.uint8)
# cv2.namedWindow('image')

# # # # create trackbars for color change
# cv2.createTrackbar('r','image',0,255,nothing)
# cv2.createTrackbar('g','image',0,255,nothing)
# cv2.createTrackbar('b','image',0,255,nothing)
# cv2.createTrackbar('ru','image',0,255,nothing)
# cv2.createTrackbar('gu','image',0,255,nothing)
# cv2.createTrackbar('bu','image',0,255,nothing)
# # cv2.createTrackbar('f','image',0,2000,nothing)
# # cv2.createTrackbar('dist','image',0,2000,nothing)
# def mouseRGB(event,x,y,flags,param):
#   if event == cv2.EVENT_LBUTTONDOWN: #checks mouse left button down condition
#     colorsB = frame[y,x,0]
#     colorsG = frame[y,x,1]
#     colorsR = frame[y,x,2]
#     colors = frame[y,x]
#     print("Red: ",colorsR)
#     print("Green: ",colorsG)
#     print("Blue: ",colorsB)
#     print("BRG Format: ",colors)
#     print("Coordinates of pixel: X: ",x,"Y: ",y)


cap = cv2.VideoCapture(opt.input_URI)
# process frames until user exits
def local_mapping(heading):
  # while True:
    # capture the next image
    #img_input = input.Capture()
  ret, frame = cap.read()
      # print (shared.get('lat'))
      # print (shared.get('lon'))    r = cv2.getTrackbarPos('r','image')
  # g = cv2.getTrackbarPos('g','image')
  # b = cv2.getTrackbarPos('b','image')
  #     # f = cv2.getTrackbarPos('f','image')
  # r_u = cv2.getTrackbarPos('ru','image')
  # g_u = cv2.getTrackbarPos('gu','image')
  # b_u = cv2.getTrackbarPos('bu','image')
  #     # #print(heading.get())
  # r = cv2.getTrackbarPos('r','image')
  # g = cv2.getTrackbarPos('g','image')
  # b = cv2.getTrackbarPos('b','image')
  #     # f = cv2.getTrackbarPos('f','image')
  # r_u = cv2.getTrackbarPos('ru','image')
  # g_u = cv2.getTrackbarPos('gu','image')
  # b_u = cv2.getTrackbarPos('bu','image')
      # dist = cv2.getTrackbarPos('dist','image')

      # a =(a -90) * np.pi/180
      # beta =(g -90) * np.pi/180
      # gamma =(b -90) * np.pi/180
      # focalLength =  f


      # w = frame.shape[0]
      # h = frame.shape[1]
      # A1 = np.array([[1, 0, -w/2],[0, 1, -h/2],[0,0,0],[0,0,1]])
      # RX = np.array([[1, 0, 0, 0],[0, np.cos(a), -np.sin(a), 0],[0, np.sin(a), np.cos(a), 0],[0, 0, 0, 1 ]])
      # RY = np.array([ [np.cos(beta), 0, -np.sin(beta), 0],[0, 1, 0, 0],[np.sin(beta), 0, np.cos(beta), 0],[0, 0, 0, 1]]	)
      # RZ = np.array([[np.cos(gamma), -np.sin(gamma), 0, 0],[np.sin(gamma), np.cos(gamma), 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]])
      # R = RX.dot(RY.dot(RZ))

      # T = np.array([[1, 0, 0, 0], [0, 1, 0, 0],[0, 0, 1, dist],[0, 0, 0, 1]]); 

      # K = np.array([[focalLength, 0, w/2, 0],[0, focalLength, h/2, 0],[0, 0, 1, 0]]); 


      # transformationMat = K.dot(T.dot( (R.dot(A1))))

  warped = cv2.warpPerspective(frame, transformMat , (frame.shape[0]+200, frame.shape[1]))

#   cv2.imshow('bird', warped)
  img_input = jetson.utils.cudaFromNumpy(frame)
      # allocate buffers for this size image
  buffers.Alloc(img_input.shape, img_input.format)

      # process the segmentation network
  net.Process(img_input, ignore_class=opt.ignore_class)

      # generate the overlay
  if buffers.overlay:
    net.Overlay(buffers.overlay, filter_mode=opt.filter_mode)

      # generate the mask
  if buffers.mask:
    net.Mask(buffers.mask, filter_mode=opt.filter_mode)
  buffers.ComputeStats()
      # print(buffers.class_mask_np)
      # mask_class = jetson.utils.cudaToNumpy(buffers.class_mask_np)

      # print(buffers.class_mask_np.shape)
      # composite the images
      #if buffers.composite:
      #	jetson.utils.cudaOverlay(buffers.overlay, buffers.composite, 0, 0)
      #	jetson.utils.cudaOverlay(buffers.mask, buffers.composite, buffers.overlay.width, 0)
  mask = jetson.utils.cudaToNumpy(buffers.mask)
      # 	# render the output image
  output.Render(buffers.mask)

	   # update the title bar
  output.SetStatus("{:s} | Network {:.0f} FPS".format(opt.network, net.GetNetworkFPS()))

  resized = cv2.resize(mask, (frame.shape[1],frame.shape[0]), interpolation = cv2.INTER_AREA)
      #mask_demo = jetson.utils.cudaToNumpy(buffers.overlay)
#   cv2.imshow('mask visualize', resized)
  warped_mask = cv2.warpPerspective(resized, transformMat,(frame.shape[0]+200, frame.shape[1]),flags = cv2.INTER_LINEAR) # Image warping
      #cv2.imshow('mask visualize birded', warped_mask )
      #warped_img = cv2.warpPerspective(mask, M,(mask.shape[1],mask.shape[0]),flags = cv2.INTER_NEAREST) # Image warping
      # print(mask)

  # lower_blue = np.array([r, g, b])
  # upper_blue = np.array([r_u, g_u, b_u])


  lower_blue = np.array([0, 10, 0])
  upper_blue = np.array([0, 255, 0])
      # # preparing the mask to overlay
  mask = cv2.inRange(warped_mask, lower_blue, upper_blue)
#   result_mask = cv2.bitwise_and(warped_mask , warped_mask , mask = mask)   
  # result_mask = result_mask    
  result_mask = 255 - mask[int(r_crop[1]):int(r_crop[1]+r_crop[3]), int(r_crop[0]):int(r_crop[0]+r_crop[2])]
  result_mask = np.fliplr(result_mask)
#   print(result_mask.shape)
  # print(result_mask.max())
  # print(result_mask.min())
     # out.write(result_mask)
     # out1.write(resized)
  # occupancy_map = np.zeros((6,5))
  # for i in range(6):
  #   for j in range(5):
  #       if( (mask[i*60:(i*60+ 86),j*86:(j*86 + 86)] == 255).sum( ) > 3000): #$80% map

  #         occupancy_map[i][j] = 1
                

  #         cv2.line(result_mask, (0, (i+1)*60), (430, (i+1)*60), (255, 255,255), thickness=1)
  #         cv2.line(result_mask, (j*86, 0), (j*86, 360), (255, 255,255), thickness=1)
  #     # print(occupancy_map)
  #   np.flipud(occupancy_map)
  #   np.fliplr(occupancy_map)
    # shared.set('occu',occupancy_map)
    # dis_map = np.ones((6,5))*1000
    #   # map_pos = []
    # wall = []
    # for i in range(6):
    #   for j in range(5):
    #     if(occupancy_map[i][j] == 0):
    #           wall.append((j,i))
    #     else:
    #       dis = np.hypot(10 - j,10 - i )
 
    #       dis_map[i][j] = dis
    # road_graph.walls = wall
      
    # print(diagram4.cost(start,goal))
    # start, goal =(2,5),(2,0)
    # found_way = 0
    # came_from, cost_so_far = a_star_search(road_graph, start, goal)
      
    #   # print(warped_img.shape)
      
      
    # try:
    #   draw_grid(road_graph,point_to = came_from,start = start,goal = goal)
    #   draw_grid(road_graph,path = reconstruct_path(came_from, start=start, goal=goal))
    # # print(diagram4.weights)
    # except Exception as e:
    #   print(e)
            # point = (int( (index_pos_list[len(index_pos_list)-1]- index_pos_list[0] )*10+5),i*10+5) 
      # render the output image
      #output.Render(buffers.output)

      # update the title bar
      #output.SetStatus("{:s} | Network {:.0f} FPS".format(opt.network, net.GetNetworkFPS()))
  # cv2.imshow('1', result_mask)  

      # print out performance info
  jetson.utils.cudaDeviceSynchronize()
      # net.PrintProfilerTimes()
      # time.sleep(0.1)
  # cv2.imshow("Result", warped)
        #  operations on the frame come here
  # cv2.imshow('frame', frame)
  key = cv2.waitKey(3) & 0xFF
  if key == ord('q'):  # Quit
    np.savetxt('map.txt',  mask[int(r_crop[1]):int(r_crop[1]+r_crop[3]), int(r_crop[0]):int(r_crop[0]+r_crop[2])], fmt='%d')
    # break
  return result_mask.astype(np.int8)
#local_mapping(0, 0)
