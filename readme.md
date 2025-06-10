### Step 1: init franka hardware

* Turn on the robot
* Turn on the NUC
* From desktop https://172.16.0.2/desk/
* unlock motors
* enable FCI


 
### Step 2: init franka control (use this)
```bash
ssh carl@172.16.0.3

(optional) # check performance mode
cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

./franka_init.sh        #give password when asked
```

### Step 3: Teleoperation check (from desktop)

Reset
```
cd fr3
python3 reset_robot_joints.py 

```

Spacemouse
```
cd fr3
python3 run_deoxys_with_space_mouse.py --product-id 50746
```
to use wireless space mpuse 
 python3 run_deoxys_with_space_mouse.py --product-id 50770
### Step 4: Init cameras
```
in fr3 

# with rgb only
./run_cams.sh 

#or with rgb+depth
./run_cams.sh  --use-depth


#view cameras (info only)
python camera_redis/view_redis_cams.py --camera-ids-color 01 --camera-ids-depth 01
python camera_redis/view_redis_cams.py --camera-ids-color 01


#info only: view topics
redis-cli keys '*'


```

### Step 5: collect data
```
conda deactivate
python3 collect_data_with_space_mouse.py

python3 fr3/collect_data_with_space_mouse.py --use-depth
```

see the data here: /home/franka_deoxys/data_franka/imgsd_demo/


### Step 6: data post process
```
# convert to hdf5 format
python3 create_dataset_example22.py --folder /home/franka_deoxys/data_franka/imgsd_demo

# make video
python3 data_ipynb/hdf5_2video.py --file /home/franka_deoxys/data_franka/imgsd_demo/demo.hdf5
```


--------------rest of them -----no need now---------

### Step 4: Data collection

collect image data
```
python3 examples/demo_collection/data_collection2_imgs.py
python3 examples/reset_robot_joints.py 
#not python3 examples/demo_collection/create_dataset2.py --folder /home/franka_deoxys/data_franka/lift_blue

# python3 examples/create_dataset_example2.py --folder /home/franka_deoxys/data_franka/imgs_demo/
franka_deoxys@carl-rog:~/deoxys_control/data_ipynb$ python3 hdf5_2video.py --file /home/franka_deoxys/data_franka/imgs_demo/demo.hdf5

```
 
sudo apt install usb-creator-gtk

### collect image data v2 (realsense images)
```
view cameras: python3 deoxys_realsense2.py

python3 demo_collection/data_collection2_imgs_rs.py


python3 create_dataset_example2.py --folder /home/franka_deoxys/data_franka/imgs_demo/

```

### Step 2: init franka control (no need, commands kept for info only)

```bash
ssh carl@172.16.0.3
# check performance mode
cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# run arm and gripper
cd ~/deoxys_control/deoxys
./auto_scripts/auto_gripper.sh config/charmander.yml

#another terminal
sudo su
./auto_scripts/auto_arm.sh config/charmander.yml
```

------camera-------------
```bash
#start redis server:  
redis-server

#start camera1:       
cd fr3
python deoxys_camera_node_org.py --camera-ref rs_0 --use-rgb --use-depth --eval --use-rec
python deoxys_camera_node_org.py --camera-ref rs_0 --use-rgb  --eval --use-rec

#start camera2: 
python deoxys_camera_node_org.py --camera-ref rs_1 --use-rgb --use-depth --eval --use-rec
python deoxys_camera_node_org.py --camera-ref rs_1 --use-rgb --eval --use-rec

```
