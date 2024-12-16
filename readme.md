### Step 1: Turn on the robot
### Step 2: Turn on the NUC
### Step 3: From desktop https://172.16.0.2/desk/  then enable FCI, unlock
### Step 4: 

```
ssh carl@172.16.0.3
```

check performance mode
```bash
cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
```

run arm and gripper
```bash
cd ~/deoxys_control/deoxys

./auto_scripts/auto_gripper.sh config/charmander.yml

#another terminal
sudo su
./auto_scripts/auto_arm.sh config/charmander.yml
```



### Desktop

Reset
```
~/deoxys_control/deoxys$ python3 examples/reset_robot_joints.py 

```

Spacemouse
```
~/deoxys_control/deoxys$ python3 examples/run_deoxys_with_space_mouse.py
```

collect image data
```
python3 examples/demo_collection/data_collection2_imgs.py
python3 examples/reset_robot_joints.py 
#not python3 examples/demo_collection/create_dataset2.py --folder /home/franka_deoxys/data_franka/lift_blue

# python3 examples/create_dataset_example2.py --folder /home/franka_deoxys/data_franka/imgs_demo/
franka_deoxys@carl-rog:~/deoxys_control/data_ipynb$ python3 hdf5_2video.py --file /home/franka_deoxys/data_franka/imgs_demo/demo.hdf5

```


