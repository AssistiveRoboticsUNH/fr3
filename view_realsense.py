import scipy.io as sio
 
import numpy as np
import cv2 
import pyrealsense2 as rs
from threading import Thread
import time


from camera_utils import MyRealSense

r1 = MyRealSense()
# tb1=Thread(target=r1.run)
# tb1.start()

for i in range(100):
    # frame = r1.current_frame
    frame = r1.get_current_frame()
    # if frame==None: continue
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    cv2.imshow("preview", frame) 
    key = cv2.waitKey(20)
    if key == 27: # exit on ESC
        break
