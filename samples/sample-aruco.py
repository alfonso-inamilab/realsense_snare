#############################################################
##      Virtual snare with real sense depth camera         ##
##    this examle uses aruco markers to detect the snare   ## 
##      or cymbals orientation. Elimnates noise problem    ## 
#############################################################

# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
# Used to calculate the velocity
import time
# Aruco support in Opencv
import cv2.aruco as aruco
# Mat is for math 
import math
# USED ONLY FOR DEBUG
import csv   # USED ONLY FOR DEBUG
from matplotlib import pyplot as plt   # USED ONLY FOR DEBUG

# GLOBAL PARAMETERS 
resX = 640 #camera and final result resolution in X
resY = 480 #camera and final result resolution in Y

# Create a pipeline
pipeline = rs.pipeline()

#Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()
config.enable_stream(rs.stream.depth, resX, resY, rs.format.z16, 30)  #GETS BETTER DEPTH READINGS !!
config.enable_stream(rs.stream.color, resX, resY, rs.format.bgr8, 30)  #GETS BETTER DEPTH READINGS !!
# config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
# config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

# Variables for drawing the rectangles 
AVGS = 20   # Number of average distances
TRIGGER_VAL = 2.0   #treshold for the detection mask
rects_start = []   #rectangles origin point (down clicks)
rects_end = []     #rectangles end points  (up clicks)
contacts = []     #array with rectangle contact flags
planes = []    #saves the normal contact plane of every rectangle
distance_offset = 25  #depth avobe the average of the active area (in RS units, snare skin) 
green = (0,255,0) #color bgr  CONTACT
red = (0,0,255) #color bgr    NO CONTACT

# ARUCO variables 
global color_image 
global depth_image
color_image = None
depth_image = None
ARUCO_PARAMETERS = aruco.DetectorParameters_create()
ARUCO_DICT = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)   #aruco dictionary  (3.5 x 2.5 cm   and 4 by 4 bits)

# Save click events (setups the active areas by user's clicks)
def click_callback(event,x,y,flags,param):  
    if(event == cv2.EVENT_LBUTTONDOWN):  
        rects_start.append([x,y]) 
    elif(event == cv2.EVENT_LBUTTONUP):  
        plane  = getContactPlane(rects_start[-1][0],rects_start[-1][1], x, y ) 
        if plane is not None:
            rects_end.append([x,y])
            contacts.append(False)
            rects_contact = False
            planes.append(plane)
        else:
            rects_start.pop(-1)

cv2.namedWindow('Real Sense Snare', cv2.WINDOW_AUTOSIZE)  #OpenCV window obj
cv2.setMouseCallback('Real Sense Snare', click_callback)  #link window for mouse click 

# Draw the active detection areas with red rectangles
def draw_rectangles(img, contacts):
    for i,r_end in enumerate(rects_end):
        cx = rects_start[i][0] - int ( (rects_start[i][0] - r_end[0]) / 2 ) 
        cy = rects_start[i][1] - int ( (rects_start[i][1] - r_end[1]) / 2 ) 

        if contacts[i] == False:
            cv2.rectangle(img,(rects_start[i][0],rects_start[i][1]),(r_end[0],r_end[1]),red,2) #draw red rectangle
            cv2.circle(img, (cx,cy), radius=0, color=red, thickness=-1)   #point on the center (vel point)
        else: 
            cv2.rectangle(img,(rects_start[i][0],rects_start[i][1]),(r_end[0],r_end[1]),green,2) #draw red rectangle
            cv2.circle(img, (cx,cy), radius=0, color=green, thickness=-1)   #point on the center (vel point)

# Search for an aruco marker in the rectangle area and creates a virtual depth plane using the aruco corners and depth data
# Returns None if nothing is found
def getContactPlane(x1, y1, x2, y2):
    global color_image
    global depth_image
    
    # Detect the aruco markers present in the color camera
    gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)  # color image to grayscale
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS) # search for all the arucos

    if ids is not None:
        #find the nearest aruco from the rectangle start point.
        ar_indx = 0
        min_dist = resX * resY
        for i, cor in enumerate (corners):
            dist =  ((cor[0][0][0] - x1) ** 2) + ((cor[0][0][1] - y1) ** 2)  
            if dist < min_dist:
                min_dist = dist
                ar_indx = i
        
        # color_image = aruco.drawDetectedMarkers(color_image, corners, borderColor=(0, 0, 255))     #DEBUG ONLY
        # color_image = cv2.line(color_image, ( int(corners[ar_indx][0][0][0]), int(corners[ar_indx][0][0][1])), (x1,y1), red, 2)  #DEBUG ONLY
        # cv2.imwrite("debug.png", color_image)  #DEBUG ONLY 
        # print (corners[ar_indx][0][0][0])  # DEBUG
        # print (corners[ar_indx][0][0][1])  # DEBUG
        # print ( depth_image[int(corners[ar_indx][0][0][1]), int(corners[ar_indx][0][0][0]) ] )   #DEBUG

        # get the plane equation from the nearest aruco 
        c1 = np.array( [corners[ar_indx][0][0][0], corners[ar_indx][0][0][1],  depth_image[int(corners[ar_indx][0][0][1]), int(corners[ar_indx][0][0][0]) ]  ] )
        c2 = np.array( [corners[ar_indx][0][1][0], corners[ar_indx][0][1][1],  depth_image[int(corners[ar_indx][0][1][1]), int(corners[ar_indx][0][1][0]) ]  ] )
        c3 = np.array( [corners[ar_indx][0][2][0], corners[ar_indx][0][2][1],  depth_image[int(corners[ar_indx][0][2][1]), int(corners[ar_indx][0][2][0]) ]  ] )
        plane_eq = getPlaneEquation( c1, c2, c3 )
        
        # Full screen size plane 
        plane = np.zeros( (resY, resX), dtype=np.float32)
        for y in range(0, resY):
            for x in range(0, resX):
                plane[y,x] = (plane_eq[3] - (plane_eq[0]*x) - (plane_eq[1]*y))  / plane_eq[2]
            
        # plt.imshow(plane)  # DEBUG 
        # plt.show()         # DEBUG
        return plane
       
    else:
        print ("ERROR 1: No aruco found inside the rectangle.")
        return None

# Gets the plane equation using 3 points 
def getPlaneEquation(p1, p2, p3):
    # TODO ensure that any of the 3 ponts has dept value zero 
    print (p1)
    print (p2)
    print (p3)
    # These two vectors are in the plane
    v1 = p3 - p1
    v2 = p2 - p1

    # the cross product is a vector normal to the plane
    cp = np.cross(v1, v2)
    a, b, c = cp

    # This evaluates a * x3 + b * y3 + c * z3 which equals d
    d = np.dot(cp, p3)

    return (a,b,c,d)

# Uses the depth mask to check for contacts inside the rectangle
# returns an array with boolean flags. 
def contactCheck(mask, trigger):
    for i, r_end in enumerate(rects_end):
        contacts[i] = False
        mean = np.mean (mask[ rects_start[i][1] : r_end[1],  rects_start[i][0] : r_end[0]])
        if mean > trigger:
            contacts[i] = True

# Streaming loop
try:
    while True:
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            continue

        #Gets data from Real Sense
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

    
        # Apply the mask to only the active region
        rect_color = red
        # For each normal plate (which has been selected with the mouse)
        for i, aruco_plane in enumerate(planes):
            above_frame = aruco_plane - distance_offset  # virtual frame - distance offset
            above =  np.where( (depth_image > above_frame), 255, 0) #above the active area (distance longer to the above frame )  >
            below = np.where( (depth_image < aruco_plane), 255, 0) #below the active area (distance shorter to the average frame ) <
            detection_mask = cv2.bitwise_and(above, below)  #AND betwen the areas  (distance between the above_frame and avg_frame)
            
            # VISUAL DEBUG ONLY
            detection_mask = np.array(detection_mask, dtype=np.uint8)   # DEBUG
            above = np.array(above, dtype=np.uint8)  # DEBUG
            below = np.array(below, dtype=np.uint8)  # DEBUG
            cv2.imshow('above', above)    #DEBUG
            cv2.imshow('below', below)    #DEBUG
            cv2.imshow('Detection Mask', detection_mask)    #DEBUG

            contactCheck(detection_mask, TRIGGER_VAL)
            
        # Apply color map to depth image 
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Draw rectangles in the contact areas (green if contact, red if not)
        draw_rectangles(depth_colormap, contacts)
        cv2.imshow('Real Sense Snare', depth_colormap)
        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
finally:
    pipeline.stop() 