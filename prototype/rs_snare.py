##########################################################################
##        Virtual snare with real sense depth camera                      ##
##         virtual plane is set over the selected are                     ## 
##       stick hit should trigger a vel 255 midi message                  ##
##         (aruco is only used for user id and VR snare tracking          ## 
##########################################################################

# TODO. Test better on reflective surfaces
# TODO. Use chacuro to detect VR snare movement
# TODO. GET VELOCITY FROM MOTION TRACKING 
# TODO. GET POSITION FROM DETECTION MASK

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
# FOR MIDI MESSAGES CREATION AND CONTROL
import mido

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

# Variables for contact control and rectangles
AVGS = 20   # Number of average distances
TRIGGER_VAL = 2.0   #treshold for the detection mask
GREEN = (0,255,0) #color bgr  CONTACT
RED = (0,0,255) #color bgr    NO CONTACT
WHITE = (255,255,255)  #line beteen aruco and rectangle

PLANE_RISE_DIST = 3   # Interval used to raise the virtual plain until above the surface (RS unities)
DIST_OFFSETS = [0,1.0]  # [3.8,4] ORIGINAL  #distance from aruco to the drum skin (INSERT DISTANCE IN cm)
rects_start = []   # rectangles origin point (down clicks)
rects_end = []     # rectangles end points  (up clicks)
contacts = []     # array with rectangle contact flags
planes = []    # saves the normal contact plane of every rectangle
aruco_corners = []  # saves relation bewtween aruco and rectangle

# ARUCO variables 
global color_image   # saves RS color image
global depth_image   # saves RS depth image
color_image = None
depth_image = None
ARUCO_PARAMETERS = aruco.DetectorParameters_create()            
ARUCO_DICT = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)  # aruco dictionary  (3.5 x 2.5 cm   and 4 by 4 bits)

# MIDI variables 
INSTRUMENTS = [1, 37]  # instrument program numbers (asigned by order)  12 Vibrophone, 34 Slap Bass
# OPEN mido port 
print ( mido.get_output_names() )  # Prints MIDI ports
port = mido.open_output('loopPort 1')

# Scale DIST_OFFSETS from cm to RealSense units, using depth_scale
for i in range(len(DIST_OFFSETS)):
    DIST_OFFSETS[i] = (DIST_OFFSETS[i] / 100.0) / depth_scale  #cm to meter and then to RS units

# Save click events (setups the active areas by user's clicks)
def click_callback(event,x,y,flags,param):  
    if(event == cv2.EVENT_LBUTTONDOWN):    #  STARTS RECTANGLE CLICK
        rects_start.append([x,y]) 
    elif(event == cv2.EVENT_LBUTTONUP):    #  RECTANGLE RELEASE CLICK
        plane  = getContactPlane(rects_start[-1][0],rects_start[-1][1], x, y ) 
        if plane is not None:
            rects_end.append([x,y])
            contacts.append(False)
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
            cv2.rectangle(img,(rects_start[i][0],rects_start[i][1]),(r_end[0],r_end[1]),RED,2) #draw red rectangle
            cv2.circle(img, (cx,cy), radius=0, color=RED, thickness=-1)   #point on the center (vel point)
            img = cv2.line(img, aruco_corners[i], (rects_start[i][0],rects_start[i][1]), WHITE, 2)  # Draw line between rectangle start and its correspondant aruco
        else: 
            cv2.rectangle(img,(rects_start[i][0],rects_start[i][1]),(r_end[0],r_end[1]),GREEN,2) #draw red rectangle
            cv2.circle(img, (cx,cy), radius=0, color=GREEN, thickness=-1)   #point on the center (vel point)  
            img = cv2.line(img, aruco_corners[i], (rects_start[i][0],rects_start[i][1]), WHITE, 2)  # Draw line between rectangle start and its correspondant aruco


# Returns True if the given plane is completely above the given plane img
def isPlaneAbove(plane, x1, y1, x2, y2):
    global depth_image
    above =  np.where( (plane[y1:y2, x1:x2] > depth_image[y1:y2, x1:x2]), 255, 0)
    if (np.mean(above) > 0):
        return False
    return True

# Creates a virtual depth plane using the aruco marker corners and the depth information
# Returns the plane depth image or none if arucos are not found
def getContactPlane(x1, y1, x2, y2):
    global color_image
    global depth_image
    
    # Detect the aruco markers present in the color camera
    gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)  # color image to grayscale
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS) # search for all the arucos

    if ids is not None:
        # Find the nearest aruco from the rectangle start point.
        # CHANGE. Charuco is used for user identification and movement
        # TODO. Use chacuro to detect VR snare movement
        ar_indx = 0
        min_dist = resX * resY
        for i, cor in enumerate (corners):
            dist =  ((cor[0][0][0] - x1) ** 2) + ((cor[0][0][1] - y1) ** 2)  
            if dist < min_dist:
                min_dist = dist
                ar_indx = i

        # CALCULATE PLANE USING THE RECTANGLE DATA AND CORNERS DEPTH 
        c1 = np.array( [ x1, y1, depth_image[y1,x1] ] )
        c2 = np.array( [ x2, y2, depth_image[y2,x2] ] )
        c3 = np.array( [ x2, y1, depth_image[y1,x2] ] )
        if (depth_image[y1,x1] <= 0 and depth_image[y2,x2] <= 0 and depth_image[y1,x2] <= 0 ):  # If depth cero cancel.
            print ("ERROR : Depth in a corner is cero.")
            return None
        plane_eq = getPlaneEquation( c1, c2, c3 )

        # Full screen size plane 
        plane = np.zeros( (resY, resX), dtype=np.float32)
        for y in range(0, resY):
            for x in range(0, resX):
                plane[y,x] = (plane_eq[3] - (plane_eq[0]*x) - (plane_eq[1]*y))  / plane_eq[2]
                    
        i = PLANE_RISE_DIST
        while (isPlaneAbove(plane, x1, y1, x2, y2) == False):
            c1 = np.array( [ x1, y1, depth_image[y1,x1] - i ] )
            c2 = np.array( [ x2, y2, depth_image[y2,x2] - i ] )
            c3 = np.array( [ x2, y1, depth_image[y1,x2] - i ] )
            plane_eq = getPlaneEquation( c1, c2, c3 )

            plane = np.zeros( (resY, resX), dtype=np.float32)
            for y in range(0, resY):
                for x in range(0, resX):
                    plane[y,x] = (plane_eq[3] - (plane_eq[0]*x) - (plane_eq[1]*y))  / plane_eq[2]
            i = i + PLANE_RISE_DIST

            # DEBUG USING 3D using Matplot lib
            # x = np.linspace(x1, x2, x2-x1)    # DEBUG
            # y = np.linspace(y1, y2, y2-y1)    # DEBUG
            # X, Y = np.meshgrid(x, y)  # DEBUG
            # Z = depth_image[y1 : y2, x1 : x2 ]    # DEBUG
            # Z2 = plane[y1 : y2, x1 : x2 ]     # DEBUG
            # fig = plt.figure()    # DEBUG
            # ax = plt.axes(projection='3d')    # DEBUG
            # ax.contour3D(X, Y, Z, 50, cmap='binary')      # DEBUG
            # ax.contour3D(X, Y, Z2, 50, cmap='binary')     # DEBUG
            # ax.set_xlabel('x')    # DEBUG
            # ax.set_ylabel('y')    # DEBUG
            # ax.set_zlabel('z')    # DEBUG
            # plt.show()    # DEBUG

        # plt.imshow(plane)    # DEBUG
        # plt.show()           # DEBUG
        aruco_corners.append( ( int(corners[ar_indx][0][0][0]), int(corners[ar_indx][0][0][1]) ) ) 
        return plane
       
    else:
        print ("ERROR : No aruco found.")
        return None

# Gets the plane equation using 3 points 
def getPlaneEquation(p1, p2, p3):
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
def contactCheck(mask, index, trigger):
    mean = np.mean (mask[ rects_start[index][1] : rects_end[index][1],  rects_start[index][0] : rects_end[index][0]])
    if mean > trigger:
        contacts[i] = True
    else:
        contacts[i] = False

# Triggers the given note with the given velocity when contact has contrast
# Midi_ON when prev_contact==True and current_contact=False
# Midi_OFF when prev_contact==False and current_contact=True
def midiTrigger(prev_contact, current_contact, instrument, note, velocity):
    if prev_contact == False and current_contact == True: # MIDI ON
        pc = mido.Message('program_change', program=instrument, channel=1, time=0 ) # Change instrument 
        port.send(pc)
        msg = mido.Message('note_on', note=note, velocity=velocity, channel=1, time=0)
        port.send(msg)       
    if prev_contact == True and current_contact == False:  # MIDI OFF
        pc = mido.Message('program_change', program=instrument, channel=1, time=0 ) # Change instrument 
        port.send(pc)
        msg = mido.Message('note_off', note=note, velocity=velocity, channel=1, time=0)
        port.send(msg)


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
    
        # For each normal plate (which has been selected with the mouse)
        prev_contacts = contacts.copy()  # Contrast between False and True triggers the MIDI message
        for i, aruco_plane in enumerate(planes):
            # NEW VERSION ARUCO ATTACHED ON THE SURFACE
            over = aruco_plane - DIST_OFFSETS[0] 
            over_over = aruco_plane - DIST_OFFSETS[1]
            above = np.where( (depth_image > over_over ), 255, 0)
            below = np.where( (depth_image < over ), 255, 0) 
            
            detection_mask = cv2.bitwise_and(above, below)

            # VISUAL DEBUG ONLY
            if (i == 0):
                debug_mask = np.array(detection_mask, dtype=np.uint8)   # DEBUG
                above = np.array(above, dtype=np.uint8)  # DEBUG
                below = np.array(below, dtype=np.uint8)  # DEBUG
                cv2.imshow('above', above[rects_start[i][1] : rects_end[i][1], rects_start[i][0] : rects_end[i][0] ])    #DEBUG
                cv2.imshow('below', below[rects_start[i][1] : rects_end[i][1], rects_start[i][0] : rects_end[i][0] ])    #DEBUG
                cv2.imshow('Detection Mask', debug_mask[rects_start[i][1] : rects_end[i][1], rects_start[i][0] : rects_end[i][0] ] ) 
                
                # TODO. GET VELOCITY FROM MOTION TRACKING 
                # TODO. GET POSITION FROM DETECTION MASK
                
            contactCheck(detection_mask, i, TRIGGER_VAL) # Check if detection mask mean goes beyond the threshold
            
            midiTrigger(prev_contacts[i], contacts[i], INSTRUMENTS[i], 60, 125) #Triggers and stops MIDI sound when contacts have activation or deactivation
        
        # Apply color map to depth image 
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)  # FOR DEBUG ONLY

        # Draw rectangles in the contact areas (green if contact, red if not)
        draw_rectangles(color_image, contacts)
        cv2.imshow('Real Sense Snare', color_image)         # color image DEBUG ONLY
        cv2.imshow('Real Sense Depth', depth_colormap)  #show depth image with color  # FOR DEBUG ONLY
        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
finally:
    pipeline.stop() 