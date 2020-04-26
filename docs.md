# Documentation

This is a list of features that we implemented along with a brief description on how to test them.

<hr>

## Interupting a program with Ctrl+C
The user can interrupt program execution with Ctrl+C
```
./roshell
sleep 10000
```

```sleep``` will be sent a SIGINT, and will exit. However, ```roshell``` does **not** exit.

<hr>

## Variable Handling

A variable can be assigned using '=' and retrieved with '$'
```
var=1
echo $var
1
```
## Source Command

```source``` command reads a text file with multiple lines of commands.
To test the command, source "test.sh" script file.
```
source test.sh
```

<hr>

# Robotic Features
Before running these features, you will need to build the ROS workspace.
```
cd ros_ws
catkin_make && source devel/setup.bash
```
Then, you can run the ROS nodes provided inside the packages in the `ros_ws/src` directory.

## Point Cloud Visualization

To visualize point clouds inside the terminal, you will need to launch the ROS Node `pcl2_visualizer_node` and play the rosbag provided [here](https://drive.google.com/open?id=1z4M2eawrsd_YgwQ4UPVxoBvqgmICQmMB). Download it to a location and navigate there.

First, play the rosbag
```
rosbag play test_bag_filtered.bag
```

In a different terminal, start the visualizer

```
roslaunch roshell_graphics pcl2_visualizer
```
This should now start visualizing the point clouds that are streamed from the rosbag over the default topic : `/simulator/lidar`. The window should look something like this:
![](images/pcl2_visualizer.gif)

This node allows for some parameters to be changed as needed. See the `ros_ws/src/roshell_graphics/launch/pcl2_visualizer.launch` file for more details. For example, to change the camera focal distance (which controls the zoom level) and input topic, you should launch the node like this 
```
roslaunch roshell_graphics pcl2_visualizer cam_focal_distance:=500 in_topic:=/lidar
```
