cd ros_ws
catkin_make
source devel/setup.bash
gnome-terminal -x "roscore"
cd ..
make
./roshell
