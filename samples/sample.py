#############################################################
##       Virtual snare with real sense depth camera           ##
##############################################################
# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2

# Create a pipeline
pipeline = rs.pipeline()

#Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  #GETS BETTER DEPTH READINGS !!
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  #GETS BETTER DEPTH READINGS !!
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

# Variables for drawing the rectangles and debug points in the image
active_area_top = (250,150)   #active area top coordenate (rectangle start)
active_area_down = (400,300)  #active area width and length from the top coordinate  (rectangle lenght and width)
green = (0,255,0) #color bgr  CONTACT
red = (0,0,255) #color bgr    NO CONTACT

distance_offset = 20  #depth avobe the average of the active area (in RS units, snare skin)
frame_counter = 0
trigger_val = 1.0   #treshold for the detection mask
avg_frame = None  # depth image average distance to the snake skin
above_frame = None  # depth image above the average frame 
 
cv2.namedWindow('Real Sense Snare Example', cv2.WINDOW_AUTOSIZE)  #OpenCV window obj

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

        # Take active area and apply filter to it 
        active_area = depth_image[ active_area_top[1] : active_area_down[1] , active_area_top[0] : active_area_down[0] ]

        # Get reference image after 20 frames
        # The reference image is the snare skin distance. 
        if (frame_counter == 0):
            avg_frame = np.zeros(active_area.shape, dtype=np.uint32)
            frame_counter = frame_counter + 1
        elif(frame_counter < 200):  #average the image
            avg_frame = np.add(avg_frame, active_area)
            frame_counter = frame_counter + 1
        elif (frame_counter == 200):
            avg_frame = avg_frame / frame_counter
            above_frame = avg_frame - distance_offset  # average frame + distance offset
            frame_counter = frame_counter + 1
        
        # Apply the mask to only the active region
        rect_color = red
        if above_frame is not None:
            # OLD VERSION (DO NOT TOUCH)
            above = np.where( (active_area > above_frame), 255, 0)  #above the active area (distance longer to the above frame )
            below = np.where( (active_area < avg_frame), 255, 0)  #below the active area (distance shorter to the average frame )
            detection_mask = cv2.bitwise_and(above, below)   #AND betwen the areas  (distance between the above_frame and avg_frame)
            detection_mask = np.array(detection_mask, dtype=np.uint8)  # VISUAL DEBUG ONLY

            # Check if there is someting in between the avg_frame and the above_frame (TRIGGER UPDATED)
            det_mean = np.mean(detection_mask)
            if ( (det_mean > (255 - np.mean(above))) and (det_mean > trigger_val) ):
                rect_color = green  #If so, paint the active area rectangle green

        # Apply color map to depth image 
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Draw rectangle (green if contact, red if not)
        cv2.rectangle(depth_colormap,active_area_top,active_area_down,rect_color,3)
        
        cv2.imshow('Real Sense Snare Example', depth_colormap)
        if above_frame is not None:
            above = np.array(above, dtype=np.uint8)  # VISUAL DEBUG ONLY
            below = np.array(below, dtype=np.uint8)  # VISUAL DEBUG ONLY
            cv2.imshow('Above', above)    #DEBUG
            cv2.imshow('Below', below)    #DEBUG
            cv2.imshow('Detection Mask', detection_mask)    #DEBUG
        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
finally:
    pipeline.stop()