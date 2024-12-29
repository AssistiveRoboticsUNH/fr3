# %%
import os
import json
import h5py
import numpy as np
import imageio
import cv2 
import argparse
from pathlib import Path

def render_video(f, demo_name, savepath):
    demo=f['data'][demo_name]
    # print(demo['obs'].keys())
    imgs1=demo['obs']['agentview_rgb']
    imgs2=demo['obs']['eye_in_hand_rgb']


    # imgs1=demo['obs']['agentview_rgb_depth'][:]*0.001
    # imgs2=demo['obs']['eye_in_hand_rgb_depth'][:]*0.001

    imgs=np.concatenate([imgs1, imgs2], axis=2)

    savepath=f"{savepath}/{demo_name}.mp4"

    writer = imageio.get_writer(savepath, fps=20)
    for img in imgs:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) 
        writer.append_data(img)
    writer.close()

 

def main(args): 
    dataset_path = args.file
    print(f"opening file {dataset_path}")
    
    f = h5py.File(dataset_path, "r")
    demos = list(f["data"].keys())


    lengths=[]
    for demo_name in demos:
        demo=f['data'][demo_name]
        # num_samples=demo.attrs['num_samples']
        num_samples = demo['actions'].shape[0]
        lengths.append(num_samples)

    lengths=np.array(lengths)

    print('Number of demos: ', len(demos))
    print('Max length: ', np.max(lengths))
    print('Min length: ', np.min(lengths))
    print('Mean length: ', np.mean(lengths))


    savepath = str(dataset_path).replace(".hdf5","")
    savepath = savepath+"_videos"
    Path(savepath).mkdir(parents=True, exist_ok=True)

    print('saving to ', savepath)
    for demo_name in demos:
        print('processing', demo_name)
        render_video(f, demo_name, savepath)

    print('-----------------done--------------')

if __name__ == "__main__":
    parser = argparse.ArgumentParser() 
    parser.add_argument("--file", type=Path)

    args = parser.parse_args()
    main(args)

# python3 hdf5_2video.py --file /home/franka_deoxys/data_franka/imgs_demo/demo.hdf5

# python3 hdf5_2video.py --file /home/franka_deoxys/data_franka/imgsd_demo/demo.hdf5


