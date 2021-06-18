## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#####################################################
##              Align Depth to Color               ##
#####################################################

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

# We will be removing the background of objects more than
#  clipping_distance_in_meters meters away
clipping_distance_in_meters = 1 #1 meter
clipping_distance = clipping_distance_in_meters / depth_scale

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

# Variables for drawing rectangles and debug points in the image
active_area_top = (250,50)   #active area top coordenate
active_area_down = (500,400)  #active area width and length from the top coordinate 
green = (0,255,0) #color bgr
red = (0,0,255) #color bgr

active_height = 20  #depth avobe the average of the active area (in RS units, snare skin)
frame_counter = 0
trigger_val = 1.0   #treshold for the detection mask
avg_frame = None
ref_frame = None


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

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Calculate the average area in the snare  
        active_area = depth_image[ active_area_top[1] : active_area_down[1] , active_area_top[0] : active_area_down[0] ]

        # Get reference image after 20 frames
        if (frame_counter == 0):
            avg_frame = np.zeros(active_area.shape, dtype=np.uint32)
            frame_counter = frame_counter + 1
        elif(frame_counter < 200):  #average the image
            avg_frame = np.add(avg_frame, active_area)
            frame_counter = frame_counter + 1
        elif (frame_counter == 200):
            avg_frame = avg_frame / frame_counter
            ref_frame = avg_frame - active_height
            frame_counter = frame_counter + 1
        
        # Apply the mas to only the active region
        rect_color = red
        if ref_frame is not None:
            above = np.where( (active_area > ref_frame), 255, 0)  #above the active area
            below = np.where( (active_area < avg_frame), 255, 0)  #below the active area
            detection_mask = cv2.bitwise_and(above, below)   #AND betwen the areas 
            detection_mask = np.array(detection_mask, dtype=np.uint8)  # VISUAL DEBUG ONLY
            
            # Check the mask to trigger events
            if (np.mean(detection_mask) > trigger_val ):  
                rect_color = green
        
        
        
        

        
        # Remove background - Set pixels further than clipping_distance to grey
        grey_color = 153
        depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
        bg_removed = np.where( (depth_image_3d > clipping_distance) | (depth_image_3d <= 0) , grey_color, color_image)

        # Draw rectangle
        cv2.rectangle(bg_removed,active_area_top,active_area_down,rect_color,3)

        # Render images
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        images = np.hstack((bg_removed, depth_colormap))
        cv2.namedWindow('Align Example', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Align Example', images)
        if ref_frame is not None:
            cv2.imshow('Detection Mask', detection_mask)    #DEBUG
        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
finally:
    pipeline.stop()