import cv2 as cv
import numpy as np

# 7x7cm 
# Load the predefined dictionary
# dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)  

# # Generate the marker 
# markerImage = np.zeros((200, 200), dtype=np.uint8)   
# markerImage = cv.aruco.drawMarker(dictionary, 33, 200, markerImage, 1);

# 3.5 x 2.5 cm   and 4 by 4 bits
# Load the predefined dictionary
dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_250)  

# Generate the marker 3.5x3.5cm 
for i in range(33):
    markerImage = np.zeros((200, 200), dtype=np.uint8)   
    markerImage = cv.aruco.drawMarker(dictionary, i, 200, markerImage, 1);

    cv.imwrite("markers/marker"+str(i)+".png", markerImage);
