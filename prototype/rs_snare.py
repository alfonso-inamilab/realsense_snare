#############################################################
##       Virtual snare with real sense depth camera           ##
##############################################################
# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
# Used to calculate the velocity
import time
# USED ONLY FOR DEBUG
import csv

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
rects_start = []   #rectangles origin point (down clicks)
rects_end = []     #rectangles end points  (up clicks)
contacts = []     #array with rectangle contact flags
green = (0,255,0) #color bgr  CONTACT
red = (0,0,255) #color bgr    NO CONTACT

distance_offset = 20  #depth avobe the average of the active area (in RS units, snare skin)
frame_counter = 0
TRIGGER_VAL = 1.0   #treshold for the detection mask
avg_frame = None  # depth image average distance to the snake skin
above_frame = None  # depth image above the average frame 
 
# Save click events (setups the active areas by user's clicks)
def click_callback(event,x,y,flags,param):  
    if(event == cv2.EVENT_LBUTTONDOWN):  
        rects_start.append([x,y]) 
    elif(event == cv2.EVENT_LBUTTONUP):  
        rects_end.append([x,y])
        contacts.append(False)
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
        if np.mean ( mask[ rects_start[i][1] : r_end[1],  rects_start[i][0] : r_end[0]]) > trigger:
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

        # Get reference image after 20 frames
        # The reference image is the snare skin distance. 
        if (frame_counter == 0):
            avg_frame = np.zeros(depth_image.shape, dtype=np.uint32)
            frame_counter = frame_counter + 1
        elif(frame_counter < 200):  #average the image
            avg_frame = np.add(avg_frame, depth_image)
            frame_counter = frame_counter + 1
        elif (frame_counter == 200):
            avg_frame = avg_frame / frame_counter
            above_frame = avg_frame - distance_offset  # average frame + distance offset
            frame_counter = frame_counter + 1
        
        # Apply the mask to only the active region
        rect_color = red
        if above_frame is not None:
            above = np.where( (depth_image > above_frame), 255, 0)  #above the active area (distance longer to the above frame )
            below = np.where( (depth_image < avg_frame), 255, 0)  #below the active area (distance shorter to the average frame )
            detection_mask = cv2.bitwise_and(above, below)   #AND betwen the areas  (distance between the above_frame and avg_frame)
            detection_mask = np.array(detection_mask, dtype=np.uint8)  # VISUAL DEBUG ONLY
            
            # checks for contacts in the active areas selected by the user
            contactCheck(detection_mask, TRIGGER_VAL)
            
        # Apply color map to depth image 
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Draw rectangles in the contact areas (green if contact, red if not)
        draw_rectangles(depth_colormap, contacts)

        cv2.imshow('Real Sense Snare', depth_colormap)
        if above_frame is not None:
            cv2.imshow('Detection Mask', detection_mask)    #DEBUG
        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
finally:
    pipeline.stop()