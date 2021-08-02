import cv2
import time
import math
import mediapipe as mp
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

# For webcam input:
cap = cv2.VideoCapture(0)
now = time.time_ns()
past = 0
first = True
p_left = [None, None, None]
p_right = [None, None, None]
v_left = [None, None, None]
v_right = [None, None, None]
with mp_pose.Pose(
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5) as pose:
  while cap.isOpened():
    success, image = cap.read()
    if not success:
      print("Ignoring empty camera frame.")
      # If loading a video, use 'break' instead of 'continue'.
      continue

    # Flip the image horizontally for a later selfie-view display, and convert
    # the BGR image to RGB.
    image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
    past = now
    now = int( round(time.time() * 1000 ))   # ORIGINAL int( round(time.time() * 1000))
    # To improve performance, optionally mark the image as not writeable to
    # pass by reference.
    image.flags.writeable = False
    results = pose.process(image)

    # Draw the pose annotation on the image.
    image.flags.writeable = True
    annotated_image = image.copy()
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    mp_drawing.draw_landmarks(
        image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

    # Calculate the hands velocity and height 
    # NOTE pose_landmarks are less noisy than pose_world_landmarks
    if results.pose_landmarks and first:
      # left wrist position
      p_left[0] = results.pose_landmarks.landmark[16].x 
      p_left[1] = results.pose_landmarks.landmark[16].y 
      p_left[2] = results.pose_landmarks.landmark[16].z 
      # right wrist position
      p_right[0] = results.pose_landmarks.landmark[15].x 
      p_right[1] = results.pose_landmarks.landmark[15].y 
      p_right[2] = results.pose_landmarks.landmark[15].z 
      # get the time 
      first = False
    elif results.pose_landmarks and not first:
      # get time 
      dt = now - past 
      # print (dt)
      # left wrist velocity 
      v_left[0] = (p_left[0] - results.pose_landmarks.landmark[16].x) / dt
      v_left[1] = (p_left[1] - results.pose_landmarks.landmark[16].y) / dt 
      v_left[2] = (p_left[2] - results.pose_landmarks.landmark[16].z) / dt 
      # right wrist velocity 
      v_right[0] = (p_right[0] - results.pose_landmarks.landmark[15].x) / dt 
      v_right[1] = (p_right[1] - results.pose_landmarks.landmark[15].y) / dt  
      v_right[2] = (p_right[2] - results.pose_landmarks.landmark[15].z) / dt  

      # left wrist position
      p_left[0] = results.pose_landmarks.landmark[16].x 
      p_left[1] = results.pose_landmarks.landmark[16].y 
      p_left[2] = results.pose_landmarks.landmark[16].z 
      # right wrist position
      p_right[0] = results.pose_landmarks.landmark[15].x 
      p_right[1] = results.pose_landmarks.landmark[15].y 
      p_right[2] = results.pose_landmarks.landmark[15].z 

      # PRINT VELOCITY VALUES 
      # print (  "{:.5f}".format(v_left[0]) +","+  "{:.5f}".format(v_left[1]) +"," + "{:.5f}".format(v_left[2]) + "," , end='' )
      # print (  "{:.5f}".format(v_right[0]) +","+  "{:.5f}".format(v_right[1]) +"," + "{:.5f}".format(v_right[2]) + ","  )
      
      # Velocity MIDI Normalization (0-255)
      MAX_VEL = 1/255  # Pose landmaks go from 0 to 1
      v_left[0] = (v_left[0]*255)/MAX_VEL;  v_left[1] = (v_left[1]*255)/MAX_VEL;    v_left[2] = (v_left[2]*255)/MAX_VEL
      v_right[0] = (v_right[0]*255)/MAX_VEL;  v_right[1] = (v_right[1]*255)/MAX_VEL;    v_right[2] = (v_right[2]*255)/MAX_VEL

      # PRINT NORMALIZED VELOCITY 
      print (  "{:.5f}".format(v_left[0]) +","+  "{:.5f}".format(v_left[1]) +"," + "{:.5f}".format(v_left[2]) + "," , end='' )
      print (  "{:.5f}".format(v_right[0]) +","+  "{:.5f}".format(v_right[1]) +"," + "{:.5f}".format(v_right[2]) + ","  )


      # FOR DEBUG DRAW THE POSE IN THE SCREEN
      mp_drawing.draw_landmarks(
        annotated_image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
      cv2.imwrite('pinchi.png', annotated_image)

    cv2.imshow('MediaPipe Pose', image)
    if cv2.waitKey(5) & 0xFF == 27:
      break
cap.release()