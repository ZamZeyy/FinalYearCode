import numpy as np
import math
from controller import Robot, Accelerometer
from vehicle import Driver
from collections import deque
import time
import cv2
import os
from tensorflow.keras.models import load_model
import tensorflow as tf
import imghdr
from matplotlib import pyplot as plt
import sqlite3
KC = 0.277778
D = 2.35
class RASRobot(object):
    def __init__(self):
        self.__robot = Driver()
        self.__timestep = int(self.__robot.getBasicTimeStep())

        self.__camera = self.__robot.getDevice("camera")
        self.__camera.enable(self.__timestep)

        self.__accelerometer = self.__robot.getDevice("accelerometer")
        self.__accelerometer.enable(self.__timestep)

        self.__gps = self.__robot.getDevice("gps")
        self.__gps.enable(self.__timestep)
        
        self.start_time = time.time()

        self.dq = deque(maxlen=10000)
        self.last_z_acc = 0.0
        self.z_acc_change = 0.0
        self.frames_since_bump = 0
        self.gui_window_name = "RAS Robot GUI"
        self.gui_text = ""
        self.cam_frames = deque(maxlen=10000)
        self.bump_frames_dir = None
        
        self.speed = 0
    
    def detect_lane_lines(self, image):
        # Convert the image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur to the grayscale image
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        # Use the Canny edge detector to find edges in the image
        edges = cv2.Canny(blur, 50, 150)

        # Define a region of interest (ROI) for lane detection
        height, width = edges.shape
        mask = np.zeros_like(edges)
        roi_vertices = np.array([[(0, height), (width / 2, height / 2), (width, height)]], dtype=np.int32)
        cv2.fillPoly(mask, roi_vertices, 255)
        masked_edges = cv2.bitwise_and(edges, mask)

        # Use the Hough transform to detect lines in the image
        lines = cv2.HoughLinesP(masked_edges, 1, np.pi / 180, 100, minLineLength=100, maxLineGap=50)

        return lines
        
    def get_camera_image(self):
        return np.frombuffer(self.__camera.getImage(), np.uint8).reshape((200, 200, 4))

    def set_steering_angle(self, angle):
        self.__robot.setSteeringAngle(angle)

    def set_speed(self, speed):
        self.__robot.setCruisingSpeed(speed)

 #   def update_gui(self, gps_coords, pothole):
  #      gui_img = np.zeros((200, 200, 3), np.uint8)
   #     # Add the red bounding box
    #    cv2.rectangle(gui_img, (0, 50), (200, 150), (0, 0, 255), 2)
#
 #       cv2.imshow(self.gui_window_name, gui_img)
  #      cv2.waitKey(1)
   #     if pothole:
    #        cv2.putText(gui_img, "Bump Detected", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
     #   cv2.putText(gui_img, "GPS Coordinates:", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
      #  cv2.putText(gui_img, str(gps_coords[0]), (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
       # cv2.putText(gui_img, str(gps_coords[1]), (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        #cv2.putText(gui_img, str(gps_coords[2]), (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    
        

        
        
    

 
    def tick(self):
        

        if self.__robot.step() == -1:
            return False
        self.speed = self.__robot.getCurrentSpeed()
        image = self.get_camera_image()
        acc = self.__accelerometer.getValues()
        self.z_acc_change = acc[2] - self.last_z_acc
        self.last_z_acc = acc[2]
        frames_to_go_back = 0
        if not math.isnan(self.speed):
            speed = self.speed * KC
            time_to_travel_back = D / speed
            frames_to_go_back = round(time_to_travel_back / (self.__timestep / 1000))

             # Store acceleration value before bump
            acc_before_bump = acc
    # Add current camera frame to cam_frames deque
        self.cam_frames.append((image, time.time(), acc))
        
        current_time = time.time()
       
            
        
        
        
        # Ignore bumps for the first 2 seconds
       # if current_time - self.start_time > 2:
        if self.z_acc_change > 4 or self.z_acc_change < -4:
            if self.frames_since_bump == 0:
                print("Bump detected!")
                #rFrame = Frames
                # Find the perfect frame
                if frames_to_go_back < len(self.cam_frames):
                    perfect_frame = self.cam_frames[-frames_to_go_back]
                    # You can now use perfect_frame for further processin
                gps_coords = self.__gps.getValues()
                #print("GPS Coordinates:", gps_coords)
                #print("Delta Z:", self.z_acc_change)
                #self.update_gui(gps_coords)
                
                if -29.6 <= gps_coords[0] <= 59.26 and -32.6 <= gps_coords[1] <= 81.7:
                    self.bump_frames_dir = os.path.join(os.getcwd(), "area1")
                    os.makedirs(self.bump_frames_dir, exist_ok=True)
                    AREA = "area1"
                    MODEL = "A13E01.h5"
                    
                elif -29.6 <= gps_coords[0] <= 59.26 and -126.24 <= gps_coords[1] <= -32.6:
                    self.bump_frames_dir = os.path.join(os.getcwd(), "area2")
                    os.makedirs(self.bump_frames_dir, exist_ok=True)
                    AREA = "area2"
                    MODEL = "A21E11.h5"
                    
                elif -127.65 <= gps_coords[0] <= -29.6 and -126.24 <= gps_coords[1] <= -32.6:
                    self.bump_frames_dir = os.path.join(os.getcwd(), "area3")
                    os.makedirs(self.bump_frames_dir, exist_ok=True)
                    AREA = "area3"
                    MODEL = "A31E33.h5"
                    
                elif -127.65 <= gps_coords[0] <= -29.6 and -32.6 <= gps_coords[1] <= 92.92:
                    self.bump_frames_dir = os.path.join(os.getcwd(), "area4")
                    os.makedirs(self.bump_frames_dir, exist_ok=True)
                    AREA = "area4"
                    MODEL = "A42E248.h5"
                
                

            # Save the last 10 frames to bump_frames directory
                #for i, (image, timestamp, acc) in enumerate(list(reversed(self.cam_frames))[rFrames:rFrames+1]):
                
            
            #    for i, (image, timestamp, acc) in enumerate(list(reversed(self.cam_frames))[1:15]):
             #       filename = os.path.join(self.bump_frames_dir, f"{gps_coords[0]}_{gps_coords[1]}_{gps_coords[2]}_frame_{i}.jpg")
              #      cv2.imwrite(filename, image)
               #     print(f"Saved image to {filename}")
                if frames_to_go_back < len(self.cam_frames):
                    perfect_frame = self.cam_frames[-frames_to_go_back]
                    filename = os.path.join(self.bump_frames_dir, f"{gps_coords[0]}_{gps_coords[1]}_{gps_coords[2]}_perfect_frame.jpg")
                    cv2.imwrite(filename, perfect_frame[0])
                    print(f"Saved perfect frame image to {filename}")    
                conn = sqlite3.connect('Area1.db')
                conn.execute('''
                    CREATE TABLE IF NOT EXISTS images (
                        image BLOB,
                        gps TEXT,
                        Bump BOOLEAN,
                        area TEXT
                    );
                ''')
               #new_model = load_model(os.path.join(os.getcwd(), 'models', "rl_simtest.h5"))
                new_model = load_model(os.path.join(os.getcwd(), 'models', MODEL))
                # Specify the directory path
                directory_path = os.path.join(os.getcwd(), AREA)
                for i, image_name in enumerate(os.listdir(directory_path)):
                    if image_name.endswith(".jpg"):
                        image_path = os.path.join(directory_path, image_name)
                        img = cv2.imread(image_path)
                        resize = tf.image.resize(img, (256,256))
                        xhat = new_model.predict(np.expand_dims(resize/255, 0))
                        pothole = bool(xhat > 0.5)
                        
                        with open(image_path, 'rb') as f:
                            image_data = f.read()
                    gps_coords = os.path.splitext(image_name)[0] # remove the ".jpg" extension
                    conn.execute('INSERT INTO images (image, gps, Bump, area) VALUES (?, ?, ?, ?)', (image_data, gps_coords, pothole, AREA))
                    print(f"Inserted image {i+1} into database with Bump={pothole}")
                    #self.update_gui(gps_coords, pothole)
                # Delete the file from the directory
                    os.remove(image_path)
        # Commit the changes and close the connection
                conn.commit()
                conn.close()

            # Empty cam_frames deque
                self.cam_frames.clear()

                self.frames_since_bump = 20
    
        elif self.frames_since_bump > 0:
            self.frames_since_bump -= 1
        
        
        
        
        #print(self.speed)
        return True
