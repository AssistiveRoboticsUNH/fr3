
import argparse
import json
import os
import struct
import time

import cv2
# import init_path
import numpy as np
import redis
from easydict import EasyDict
from deoxys_vision.networking.camera_redis_interface import CameraRedisSubInterface

def main():
    print('view camera image from redis')

    camera_id = 0

    host = "127.0.0.1"
    port = 6379

    # camera_info = EasyDict(camera_id=camera_id, camera_name='webcam', camera_type='webcam')
    camera_info = EasyDict(camera_id=camera_id, camera_name='camera_rs_0', camera_type='rs')
    camera2redis_sub_interface = CameraRedisSubInterface(
        redis_host=host, redis_port=port, camera_info=camera_info
    ) 

    camera2redis_sub_interface.start()

    for i in range(1000):
        img_info = camera2redis_sub_interface.get_img()
        frame = img_info['color']
        # if frame==None:
        #     continue

        cv2.imshow('redis-frame', frame)
        if cv2.waitKey(1)=='27':
            break
    
    cv2.destroyAllWindows()  
    camera2redis_sub_interface.stop()

if __name__=='__main__':
    main()

 