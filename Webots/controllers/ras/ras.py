from rasrobot import RASRobot

import numpy as np
import time

import cv2

class MyRobot(RASRobot):
  def __init__(self):
    super(MyRobot, self).__init__()

    cv2.namedWindow("output", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("output", 128*4, 64*4)

    self.set_speed(30)

  def get_road(self):
      image = self.get_camera_image()
      hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
      lower = np.array([0, 0, 0], np.uint8)
      upper = np.array([255, 29, 78], np.uint8)
      mask = cv2.inRange(hsv, lower, upper).astype(bool)
      result = np.zeros(image.shape)
      result[mask] = 255
      return cv2.morphologyEx(result, cv2.MORPH_OPEN, (3,3))

  def run(self):
    while self.tick():
      road = self.get_road()
      rows = road.shape[0]
      cols = road.shape[1]
      left_side  = road[:,:cols//2]
      right_side = road[:,cols//2:]
      left_is_dominant = np.sum(left_side)-np.sum(right_side) > 0

      #if left_is_dominant:
     #   self.set_steering_angle(-0.2)
     # else:
     #   self.set_steering_angle(0.2)
      

      cv2.imshow('output', road)
      cv2.waitKey(1)


robot = MyRobot()
robot.run()


