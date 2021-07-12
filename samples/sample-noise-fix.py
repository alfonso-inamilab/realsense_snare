#############################################################
##      Virtual snare with real sense depth camera         ##
##      example thar works with a noisy surface            ##
#############################################################

# Usage click on rectangular surface on the margins of the drum, 
# then place the stick on the drum (for triggering calibration) until the depthmask appears
# Then test by hitting the drum. (green activated, red not)

# (Functionality is not so good, use sample-aruco.py instead)

# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
# Used to calculate the velocity
import time
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
depth_image = None
rects_start = []   #rectangles origin point (down clicks)
rects_end = []     #rectangles end points  (up clicks)
contacts = []     #array with rectangle contact flags
planes = []    #saves the normal contact plane of every rectangle
t_calib = []   #saves a calibration trigger avg value for reflective surfaces 
green = (0,255,0) #color bgr  CONTACT
red = (0,0,255) #color bgr    NO CONTACT

distance_offset = 25  #depth avobe the average of the active area (in RS units, snare skin)
frame_counter = 0
avg_frame = None  # depth image average distance to the snake skin
above_frame = None  # depth image above the average frame 
 
# Save click events (setups the active areas by user's clicks)
def click_callback(event,x,y,flags,param):  
    if(event == cv2.EVENT_LBUTTONDOWN):  
        rects_start.append([x,y]) 
    elif(event == cv2.EVENT_LBUTTONUP):  
        rects_end.append([x,y])
        contacts.append(False)
        planes.append(getContactPlane(rects_start[-1][0], rects_start[-1][1], x, y ) )
        t_calib.append( [0] * (AVGS+1) )
        rects_contact = False

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


# Uses the depth mask to check for contacts inside the rectangle
# returns an array with boolean flags. 
def contactCheck(mask, trigger):
    for i, r_end in enumerate(rects_end):
        contacts[i] = False
        mean = np.mean (mask[ rects_start[i][1] : r_end[1],  rects_start[i][0] : r_end[0]])
        # tt = mean - t_error 
        print (str(int(mean)) + " , " + str(int(trigger)) )
        if mean > (trigger * 2):
            contacts[i] = True

# Gets the plane equation using 3 points 
def getPlaneEquation(p1, p2, p3):
    # TODO ensure that any of the 3 ponts has a p3 on zero
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

    # print('The equation is {0}x + {1}y + {2}z = {3}'.format(a, b, c, d))
    return (a,b,c,d)

# Returns a depth defined by 3 corners of a rectangle 
# (solution to the noise problem)
def getContactPlane(x1, y1, x2, y2):
    # plane_eq = getPlaneEquation( np.array( [x1,y1, aligned_depth_frame.get_distance(x1,y1) ] ), np.array( [x2,y2, aligned_depth_frame.get_distance(x2,y2)] ), np.array( [x2,y1, aligned_depth_frame.get_distance(x2,y1) ] ) )
    plane_eq = getPlaneEquation( np.array( [x1,y1, depth_image[y1,x1] ] ), np.array( [x2,y2, depth_image[y2,x2] ] ), np.array( [x2,y1, depth_image[y1,x2] ] ) )
    
    # Rectangle size plane   # ANOTHER TYPE OF ACTIVE AREA
    # plane = np.zeros( (y2-y1, x2-x1, 1), dtype=np.float32)
    # for y in range(0, y2-y1):
    #     for x in range(0, x2-x1):
    #         xx = x + x1
    #         yy = y + y1
    #         plane[y,x] = (plane_eq[3] - (plane_eq[0]*xx) - (plane_eq[1]*yy))  / plane_eq[2]

    # Full screen size plane 
    plane = np.zeros( (resY, resX), dtype=np.float32)
    for y in range(0, resY):
        for x in range(0, resX):
            plane[y,x] = (plane_eq[3] - (plane_eq[0]*x) - (plane_eq[1]*y))  / plane_eq[2]
            
    # plt.imshow(plane)  # DEBUG 
    # plt.show()         # DEBUG
    return plane 


# Saves the distances of each rectangle and returns the average 
def triggerCalibration(mask, index):
    avg_index = t_calib[index][0] # The first place of the array saves the current average index

    #increase the value of the average index
    avg_index = avg_index + 1
    if (avg_index >= AVGS):  # loop the index
        avg_index = 1

    avg =  np.mean ( mask[ rects_start[index][1] : rects_end[index][1],  rects_start[index][0] : rects_end[index][0]])
    t_calib[index][avg_index] = avg  #saves the new value in place
    t_calib[index][0] = avg_index + 1 # save index

    #calculate and return the averaga value 
    avg = 0
    for i in range(1,AVGS):
        avg = avg + t_calib[index][i]
    return (avg/AVGS)


t_value = 0
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
        for i, normal_plane in enumerate(planes):
            above_frame = normal_plane - distance_offset  # virtual frame - distance offset
            above =  np.where( (depth_image > above_frame), 255, 0) #above the active area (distance longer to the above frame )
            below = np.where( (depth_image < normal_plane), 255, 0) #below the active area (distance shorter to the average frame )
            detection_mask = cv2.bitwise_and(above, below)  #AND betwen the areas  (distance between the above_frame and avg_frame)
            
            detection_mask = np.array(detection_mask, dtype=np.uint8)   # VISUAL DEBUG ONLY
            cv2.imshow('Detection Mask', detection_mask)    #DEBUG

            # For reflective surfaces average the error to calibrate the trigger
            if (frame_counter < 200):
                frame_counter = frame_counter + 1
                t_value =  triggerCalibration(detection_mask, i) 
            else: 
                contactCheck(detection_mask, t_value )

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