from GUI import GUI
from HAL import HAL
import cv2
import numpy as np
import math
import random
import time
# Enter sequential code!

WIDTH = HAL.getImage().shape[1]
THRESHOLD_SIZE = 5

lower = np.array([0,0,150], dtype="uint8")
upper = np.array([80, 80, 255], dtype="uint8")

last_middle_point = int(WIDTH / 2)
def get_middle_point2(img):
    for i in range(20, int(img.shape[0]/2)):
        whites = np.array(np.where(img[-i] > 0))
        if whites is not None and len(whites[0]) > 1:
            first = whites[0][0]
            last = whites[0][-1]
            if last - first < THRESHOLD_SIZE*2:
                continue
            
            return int((first + last) / 2)
        
    return last_middle_point

A = 0
last_err = 0
last_time = time.time()
def get_control(error):
    global A, last_time
    kp, ki, kd = 0.0060, 0.002600, 0.00090
    
    dt = time.time() - last_time
    last_time = time.time()
    A += dt * (error + last_err) / 2
    
    P = kp * error
    I = ki * A
    D = kd * (error - last_err) / dt
    
    return (P + I + D) * 0.65

def main():
    img = HAL.getImage()
    
    # mask red area
    mask = cv2.inRange(img, lower, upper)
    filtered_img = cv2.bitwise_and(img, img, mask=mask)
    filtered_img = cv2.cvtColor(filtered_img, cv2.COLOR_BGR2GRAY)
    
    # blur image
    filtered_img = cv2.GaussianBlur(filtered_img, (7,7), 0)

    # adaptive threshold
    th = cv2.adaptiveThreshold(filtered_img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, THRESHOLD_SIZE, 2)
    
    # get the middle point
    middlePoint = (get_middle_point2(th), img.shape[0]-20)
    cv2.circle(img, middlePoint, radius=5, color=(255, 0, 0), thickness=-1)
    cv2.circle(th, middlePoint, radius=5, color=255, thickness=-1)
    
    # add lines to img
    # img = add_lines(img, lines)
    
    # get error
    global last_middle_point
    error = int(img.shape[1]/2) - middlePoint[0]
    last_middle_point = middlePoint[0]
    
    # get control
    global last_err
    w = get_control(error)
    last_err = error
    
    HAL.setV(3)
    HAL.setW(w)
    GUI.showImage(img)
    
while True:
    # Enter iterative code!
    main()