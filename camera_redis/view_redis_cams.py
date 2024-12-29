import argparse

import cv2
# import init_path
import numpy as np
import redis
from easydict import EasyDict
from deoxys_vision.networking.camera_redis_interface import CameraRedisSubInterface

def main(camera_ids_color, camera_ids_depth):
    print('view camera image from redis')

    camera_ids_color = [int(item) for item in camera_ids_color]
    camera_ids_depth = [int(item) for item in camera_ids_depth]

   

    print('camera_ids_color:', camera_ids_color)
    print('camera_ids_depth:', camera_ids_depth, type(camera_ids_depth))

    cr_interfaces = {}
    for camera_id in camera_ids_color:
        use_depth = True if camera_id in camera_ids_depth else False 

        print(f'connecting camera_id {camera_id} use_depth={use_depth}')

        camera_info = EasyDict(camera_id=camera_id, camera_name=f'camera_rs_{camera_id}')

        cr_interface = CameraRedisSubInterface(camera_info=camera_info, use_depth=use_depth , redis_host='127.0.0.1')
        cr_interface.start()
        cr_interfaces[camera_id] = cr_interface


    while True:
        for camera_id in camera_ids_color:
            img_info = cr_interfaces[camera_id].get_img_info()
            imgs = cr_interfaces[camera_id].get_img()
            # print('keys: ', imgs.keys())
            color_img = imgs["color"][..., ::-1]
            depth_img = imgs["depth"]

            # print('check: ', np.min(color_img), np.min(depth_img), np.max(color_img), np.max(color_img))

            cv2.imshow(f"camera_{camera_id}_color", color_img) 
            # if depth_img.all()==None: 
            if len(camera_ids_depth):
                cv2.imshow(f"camera_{camera_id}_depth", depth_img * 0.001) 

            if cv2.waitKey(1)=='27':
                break
        
    cv2.destroyAllWindows()  

    for camera_id in camera_ids_color:
        cr_interfaces[camera_id].stop()



if __name__=='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera-ids-color", type=list, default=[0])
    parser.add_argument("--camera-ids-depth", type=list, default=[])
    args = parser.parse_args()

    print('args: ', args)

    main(args.camera_ids_color, args.camera_ids_depth)

 

#  python camera_redis/view_redis_cams.py --camera-ids-color 0 --camera-ids-depth 0
#  python camera_redis/view_redis_cams.py --camera-ids-color 01 --camera-ids-depth 01

#  python camera_redis/view_redis_cams.py --camera-ids-color 01 
