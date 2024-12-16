import scipy.io as sio
 
import numpy as np
import cv2 
import pyrealsense2 as rs
from threading import Thread
import time


class MyRealSense:
    def __init__(self):
        self.pipe = rs.pipeline()
        self.profile = self.pipe.start()
        self.isalive=True
        self.current_frame=None 

    def get_current_frame(self, scale=0.5):
        frames = self.pipe.wait_for_frames()
        color_frame = frames.get_color_frame() 

        image=np.asanyarray(color_frame.get_data())
        image= cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        
        image = cv2.resize(image, (int(image.shape[1]*scale), int(image.shape[0]*scale) ) , interpolation = cv2.INTER_AREA)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) 
        return image 
    
    def run(self):
        while self.isalive:
            try:
                self.current_frame = self.get_current_frame() 
            except:
                self.isalive=False
                break 

    def close(self):
        self.isalive=False
        self.pipe.stop()

    def stop(self):
        self.close()

class CVCamera:
    def __init__(self, camera_id=0):
        self.camera_id=camera_id
        self.cap = cv2.VideoCapture(camera_id)
        self.isalive=True
        self.current_frame=None
        
    def get_current_frame(self, scale=0.5):
        ret, image = self.cap.read()
        image = cv2.resize(image, (int(image.shape[1]*scale), int(image.shape[0]*scale) ) , interpolation = cv2.INTER_AREA)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) 
        return image 

    def run(self):
        while self.isalive:
            try:
                self.current_frame = self.get_current_frame() 
            except:
                self.isalive=False
                break 

    def close(self):
        self.isalive=False
        self.cap.release()

    def stop(self):
        self.close()
