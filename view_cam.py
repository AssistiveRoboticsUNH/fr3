import scipy.io as sio
 
import numpy as np
import cv2 
import pyrealsense2 as rs
from threading import Thread
import time
import argparse
 
class CVCamera:
    def __init__(self, camera_id=0):
        self.camera_id=camera_id
        self.cap = cv2.VideoCapture(camera_id)
        self.isalive=True
        self.current_frame=None
        
    def get_current_frame(self, scale=0.5):
        ret, image = self.cap.read()
        # image = cv2.resize(image, (int(image.shape[1]*scale), int(image.shape[0]*scale) ) , interpolation = cv2.INTER_AREA)
        # image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) 
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

def main(camera_id):
    print(f"opening camera id {camera_id}")
    print('-----------press esc to close---------\n')
    cam=CVCamera(camera_id)
    while cam.isalive:
        frame=cam.get_current_frame()
        if frame==None: continue
        cv2.imshow("preview", frame) 
        key = cv2.waitKey(20)
        if key == 27: # exit on ESC
            break

    cv2.destroyWindow("preview")
    cam.close()


if __name__=='__main__': 
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--camera', default=0)
    args=parser.parse_args()
    main(args.camera)


# v4l2-ctl --list-devices
