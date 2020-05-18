# roshell
roshell - An interactive shell for robotics applications


# Quickstart:
```shell
git clone https://github.com/deepaktalwardt/roshell.git
cd roshell
make
```

You should see a bunch of environment variables, and then a prompt:

```shell
$: ./roshell
user@computer $:
```

Sourcing ```build.sh``` script file builds both ROS file (catkin_make) and roshell (make), launches separate terminals for roscore and rosbag, and starts ```roshell``` executable.  From ```roshell``` command, you can run roslaunch to see the visualizer.

```
$: cd ros_ws
$: roslaunch roshell_graphics pcl2_visualizer.launch
```


You should be running the following commands:
```
$: ./roshell
user@computer $:ls /
```

```
user@computer $: ssh 01234@coe-hpc1.sjsu.edu
Last login: Sat Mar  7 10:57:06 2020 from 10.40.42.167
```
(you will need a working ssh server for this of course)


```
user@computer $:whoami
tomek
```
