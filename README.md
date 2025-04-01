# CBSea-Software

### Microfred Joystick Teleop Controls

On a laptop running ROS2 foxy, plug in the gamepad controller.

Run joy_node
- Open new terminal (ctl+alt+t on Linux machine)
- ```source /opt/ros/foxy/setup.bash```
- ```ros2 run joy joy_node```
This will run the controller node only, if you want to also run RviZ to see the robot visualization you can use the launch file to launch both the controller and visualization:
- ```ros2 launch robot_description display_and_cmd.launch.py```

On the robot, source ROS setups and start nodes:
   Open New Terminal
- ```source /opt/ros/foxy/setup.bash && source ros2_ws/install/setup.bash```

- ```ros2 launch launch/robot.launch.py```

Troubleshooting:

If the controller isn't detected, it may need to be set up:

- ```ls /dev/input/```

- ```sudo jstest /dev/input/jsX```

- ```ls -l /dev/input/jsX```

- ```sudo chmod a+rw /dev/input/js0```

## Autonomous mode

To put the robot into autonomous GPS waypoint mode from manual control, press the RB button on the controller. The robot will start moving towards the first GPS waypoint in the list of waypoints, or if none have been added it will wait.  

To add a waypoint, run this ROS command in a terminal on the robot:
- ```ros2 service call /add_waypoint waypoint_nav_interfaces/srv/AddWaypoint "{latitude: -0.846, longitude: -90.569}"```  
Replace the floats after ```latitude:``` and ```longitude:``` with your desired coordinates

To add a series of waypoints from a CSV file exported from Google maps, run:  
- ```ros2 run waypoint_loader waypoint_loader --ros-args -p csv_path:=/path/to/waypoints.csv```  
This will also publish GeoJSON polygons to Foxglove to visualize the path between waypoints. 

### Visualizing data with Foxglove

Run the display launch file, this will spin up the rosbridge node for Foxglove to communicate with the robot:
- ```ros2 launch robot_description display.launch.py```
- or you an use ```display_and_cmd.launch.py``` for controlling and visualizing. 
Start Foxglove. Click `Open connection...` and select Rosbridge. The WebSocket URL is `ws://localhost:9090`. Click Open and you should see data streaming from the robot. You may need to add panes for the map and plots for data like the IMU.

### Logging robot speed
TODO: Commit this node to git  
On the controller laptop, run this node to publish to `/speed` and log speed messages:
- ```ros2 run speed_logger speed_logger```


## Machine vision instructions

Follow the instructions below to train object detection and instance segmentation models on the FloW dataset.

Inputs: dataset (provided)

Outputs: ```.pt``` files for object detection and instance segmentation models

### Set up

Download the FloW folder from [here](https://drive.google.com/drive/u/2/folders/1ZFSAsUG19YPjx-GEFRZE5SJNu9cg3KS6)

Edit ```FloW.yaml```:
- make ```src_path``` the path of the FloW dataset you downloaded
- make ```path``` the path you want to use for this project

Run ```format_dataset.py``` to format the dataset you downloaded in your new project folder

### Training models

```models.ipynb``` is where all of the training happens

The general flow of the process goes like this:
- train detection model on object detection labels from dataset
- generate instance segmentation labels using your trained detection model and a generalized segmentation model
- train segmentation model on instance segmentation labels from previous step

The notebook will tell you which cells to run depending on your situation


microfred_gazebo.urdf.xacro

### Machine Vision Demo

To run the NVidia object detection demo with our stereo camera setup:
Set the resolution of the Jetson Nano desktop GUI to 720p.  
Download the model:  
- ```cd jetson-inference/tools```
- ```./download_models.sh```  
  Select the model(s) you want to use, then hit `Okay` to download.  
  Run the demo with this command (first time running a particular model will take a while, but the second time should be much faster):  
- ```detectnet --image-flip=clockwise --image-width=2000 --image-height=520```  
Add the ```--network=<your network>``` flag to specify a different model.  

### Microfred Simulation

To simulate the vehicle, please refer to the UUV Simulator repository and follow the installation instructions of the package. Then you can clone this package in the src folder of you catkin workspace
- ```git clone https://github.com/uuvsimulator/uuv_simulator.git```
- ```git clone https://github.com/ClearBlueSea/CBSea-Software.git```
- ```cd ~/catkin_ws/src```
- ```catkin make```
  
To run a demonstration with the vehicle with teleoperation, you must first open the world file provided by uuv simulation
- ```roslaunch uuv_gazebo_worlds empty_underwater_world.launch```
  
Upload Microfred Model onto the world file
-  ```roslaunch microfred_platform_description upload.launch```

  
### Microfred Controller (Simulation)
This controller file launches the velocity controller as well as thruster managaer. 

- ```roslaunch microfred_platform_control start_velocity_controller.launch```

### Microfred Controls Keyboard Teleop (Simulation)

Once you have microfred platform uploaded onto the world file, you can ron teleoperation controls. In order to that just run:
- ```roslaunch microfred_platform_control start_teleoperation_controller.launch teleop_on:=true joy_id:=0```
  
Follow the instructions on your terminal in order to move microfred



### Microfred Hardware Keyboard Telelop
These instructions allow for hardware controls for Micro Fred Platform
- ```rosrun teleop_twist_keyboard teleop_twist_keyboard.py```
- ```rosrun rosserial_python serial_node.py port:=/dev/ttyACM0```
  
  
Follow the instructions on your terminal in order to move microfred

### TODO: Add section about what ROS2 packages to install
- like ros-foxy-robot-localization
