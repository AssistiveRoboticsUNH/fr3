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


# from deoxys_vision.networking.camera_redis_interface import CameraRedisPubInterface
# from deoxys_vision.camera.k4a_interface import K4aInterface
from deoxys_vision.camera.rs_interface import RSInterface
from deoxys_vision.utils.img_utils import preprocess_color, preprocess_depth
from deoxys_vision.utils.camera_utils import assert_camera_ref_convention, get_camera_info
import pyrealsense2 as rs

#based on deoxys_camera_node.py

color_distortion1 = np.array([[-3.21808020e+01],
                    [ 1.56948008e+02],
                    [ 1.08836334e-04],
                    [-3.35339398e-03],
                    [ 2.35932470e+03],
                    [-3.23246223e+01],
                    [ 1.62487586e+02],
                    [ 2.30373935e+03],
                    [ 0.00000000e+00],
                    [ 0.00000000e+00],
                    [ 0.00000000e+00],
                    [ 0.00000000e+00],
                    [ 0.00000000e+00],
                    [ 0.00000000e+00]])

color_distortion2 = np.array(
                            [[-1.03494286e+01],
                            [ 1.81229044e+02],
                            [-1.33669038e-03],
                            [ 3.86838065e-03],
                            [ 6.03400600e+02],
                            [-1.04039164e+01],
                            [ 1.82251593e+02],
                            [ 5.75642409e+02],
                            [ 0.00000000e+00],
                            [ 0.00000000e+00],
                            [ 0.00000000e+00],
                            [ 0.00000000e+00],
                            [ 0.00000000e+00],
                            [ 0.00000000e+00]])   

 
def get_current_snap(camera_interface, node_config, camera_config, camera_info, camera_id):
    capture = camera_interface.get_last_obs() 
    if capture is None:
        return False, None 
    t = time.time_ns()
    if capture is None:
        return False, None
        
    imgs = {} 
    if node_config.use_color:
        color_img = preprocess_color(capture["color"], flip_channel=camera_config.rgb_convention == "rgb")
        intrinsics_matrix = camera_interface.get_color_intrinsics(mode="matrix")
        color_distortion = camera_interface.get_color_distortion()  
        if camera_config.use_rec:
            if camera_info.camera_type == "rs":
                if camera_id == 0:
                    color_distortion = color_distortion1
                else:
                    color_disotortion = color_distortion2
            imgs["color"] = cv2.undistort(color_img, intrinsics_matrix, color_distortion, None)
        else:
            imgs["color"] = color_img

    if node_config.use_depth:
        depth_img = preprocess_depth(capture["depth"])  
        imgs["depth"] = depth_img

    return True, imgs


def init_camera(camera_ref, use_rgb, use_depth, use_rec, rgb_convention, visualization, depth_visualization):
    assert_camera_ref_convention(camera_ref)
    camera_info = get_camera_info(camera_ref)
    # print information about the cameras to run

    print('--------------')
    print(camera_info)
    print('---------------')

    camera_config = EasyDict(
        camera_type=camera_info.camera_type,
        camera_id=camera_info.camera_id,
        use_rgb=use_rgb,
        use_depth=use_depth,
        use_rec=use_rec,
        rgb_convention=rgb_convention,
    )
    print(f"This node runs with the camera {camera_config.camera_type} with id {camera_config.camera_id}")
    print("The node will publish the following data:")
    if use_rgb:
        print("- Color image")
    if use_depth:
        print("- Depth image")
    if use_rec:
        print("Note that Images are rectified with undistortion")

    camera_id = camera_info.camera_id
    camera_interface = None

    node_config = EasyDict(use_color=True, use_depth=True)
    if not use_rgb:
        node_config.use_color = False

    if not use_depth:
        node_config.use_depth = False

    color_cfg = EasyDict(
        enabled=node_config.use_color, img_w=640, img_h=480, img_format=rs.format.bgr8, fps=30
    )

    # if args.use_depth:
    depth_cfg = EasyDict(
        enabled=node_config.use_depth, img_w=640, img_h=480, img_format=rs.format.z16, fps=30
    )
    # else:
    #     depth_cfg = None

    pc_cfg = EasyDict(enabled=False)
    camera_interface = RSInterface(
        device_id=camera_id, color_cfg=color_cfg, depth_cfg=depth_cfg, pc_cfg=pc_cfg
    )
    return camera_interface, node_config, camera_config, camera_info, camera_id

def main():

    camera_ref0 = "rs_0"
    camera_ref1 = "rs_1"
    
    use_rgb, use_depth, use_rec = True, True, True

    rgb_convention = "rgb" 
    visualization , depth_visualization = True, True

    camera_interface, node_config, camera_config, camera_info, camera_id=init_camera(
        camera_ref0,
        use_rgb,
        use_depth,
        use_rec,
        rgb_convention,
        visualization,
        depth_visualization
    )  
    camera_interface2, node_config2, camera_config2, camera_info2, camera_id2=init_camera(
        camera_ref1,
        use_rgb,
        use_depth,
        use_rec,
        rgb_convention,
        visualization,
        depth_visualization
    )

    camera_interface.start()
    camera_interface2.start()


    t = time.time()

    freq = 30.0

    while True:

        try:
            start_time = time.time_ns() 

            s, imgs = get_current_snap(camera_interface, node_config, camera_config, camera_info, camera_id)
            s2, imgs2 = get_current_snap(camera_interface2, node_config2, camera_config2, camera_info2, camera_id2)

            if not s: continue  
            if not s2: continue

            if visualization:
                if rgb_convention == "rgb":
                    cv2.imshow("", imgs["color"][..., ::-1])
                    cv2.imshow("2", imgs2["color"][..., ::-1])
                else:
                    cv2.imshow("", imgs["color"])
                    cv2.imshow("2", imgs2["color"])
                if depth_visualization:
                    cv2.imshow("depth", imgs["depth"] * 0.001)
                    cv2.imshow("depth2", imgs2["depth"] * 0.001)

                cv2.waitKey(10)

            end_time = time.time_ns()

            time_interval = (end_time - start_time) / (10 ** 9)
            # print(f"dt=", time_interval)
            if time_interval < 1.0 / freq:
                time.sleep(1.0 / freq - time_interval)

        except KeyboardInterrupt: 
            print('keyboard interrupt-----------------')
            break 

        camera_interface.close()
        camera_interface2.close()



if __name__ == "__main__":
    main()

# python deoxys_realsense2.py
