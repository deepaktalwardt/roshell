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
## Auto Complete

Pressing ```tab``` button auto-completes the input file name only when one match occurs.
When multiple matches occur, pressing ```tab``` second time, all matching files are listed.
For the first word in command line, it searches the matching files from ```$PATH```.
For the other words in command line, it searches files the current directory only.

<hr>

# Robotic Features
Before running these features, you will need to build the ROS workspace.
```
cd ros_ws
catkin_make && source devel/setup.bash
```
Then, you can run the ROS nodes provided inside the packages in the `ros_ws/src` directory.

## Point Cloud Visualization
### From PCD file
To visualize a point cloud stored in the PCD file, you need to run the `pcd_visualizer_node`. First, download a test PCD file provided [here](https://drive.google.com/open?id=1HfrEJ8wTFe-DFC0YWpUx6X5AZ5MJgBFG) and place it in `ros_ws/src/roshell_graphics/test/` directory. Then, use rosrun
```
rosrun roshell_graphics pcd_visualizer_node
```

### From ROS bags
To visualize point clouds streaming from a rosbag inside the terminal, you will need to launch the ROS Node `pcl2_visualizer_node` and play the rosbag provided [here](https://drive.google.com/open?id=1z4M2eawrsd_YgwQ4UPVxoBvqgmICQmMB). Download it to a location and navigate there.

First, make sure that `roscore` is running, then play the rosbag
```
rosbag play test_bag_filtered.bag
```

In a different terminal, start the visualizer

```
roslaunch roshell_graphics pcl2_visualizer.launch
```
This should now start visualizing the point clouds that are streamed from the rosbag over the default topic : `/simulator/lidar`. The window should look something like this:
![](images/pcl2_visualizer.gif)

This node allows for some parameters to be changed as needed. See the `ros_ws/src/roshell_graphics/launch/pcl2_visualizer.launch` file for more details. For example, to change the camera focal distance (which controls the zoom level) and input topic, you should launch the node like this
```
roslaunch roshell_graphics pcl2_visualizer cam_focal_distance:=500 in_topic:=/lidar
```

## Line Plots

To create line plots, we first need to have a publisher node that will publish random values to be plot on the line plot. To do that, launch the following
```
roslaunch roshell_graphics float_visualizer.launch
```
This will start both a floating point publisher and a subscriber and start printing values to the scren.

## Images

To test image functionality:

Do the standard setup:
```bash
cd roshell/ros_ws
catkin_make
source devel/setup.bash
```

Get some rosbags with image data to test. For example: [http://infinity.csail.mit.edu/data/2011/2011-04-06-06-38-27.bag](http://infinity.csail.mit.edu/data/2011/2011-04-06-06-38-27.bag).

Play your rosbag:
``` bash
rosbag play 2011-04-06-06-38-27.bag -l -r 0.5
```

Run the node, passing the topic as an argument:
```bash
rosrun roshell_graphics image_viewer_node _in_topic:=/wide_stereo/right/image_raw
```
