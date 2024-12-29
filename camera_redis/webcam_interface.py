
import numpy as np
from PIL import Image
 
from matplotlib import pyplot as plt
 
import scipy.io as sio
 
import numpy as np
import cv2  
from threading import Thread
import time



class CVCamera:
    def __init__(self, camera_id=0):
        self.camera_id=camera_id

        self.cap = cv2.VideoCapture(camera_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        r, frame = self.cap.read()
        # print('Resolution: ' + str(frame.shape[0]) + ' x ' + str(frame.shape[1]))

        self.isalive=True
        self.current_frame=None
        
    def get_current_frame(self, scale=None):
        ret, image = self.cap.read()
        if scale!=None:
            image = cv2.resize(image, (int(image.shape[1]*scale), int(image.shape[0]*scale) ) , interpolation = cv2.INTER_AREA)
        return image 

    def run(self):
        print('camera running...')
        while self.isalive:
            ret, frame = self.cap.read()
            self.current_frame=frame

    def start(self):
        t1 = Thread(target=self.run)
        t1.start()

    def close(self):
        self.isalive=False
        self.cap.release()


def main():
    cam = CVCamera(0)
    for i in range(100):
        frame = cam.get_current_frame()
        cv2.imshow("frame", frame)
        if cv2.waitKey(1)=='27':
            break
    
    cv2.destroyAllWindows()  
    cam.close()
    

if __name__=='__main__':
    main()
