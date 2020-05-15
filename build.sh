cd ros_ws
catkin_make
source devel/setup.bash
gnome-terminal -x "roscore"
gnome-terminal -e "rosbag play test_bag_filtered.bag -l"
cd ..
make
./roshell
