# source ros2 setup file
source /opt/ros/humble/setup.bash || source /opt/ros/foxy/setup.bash

# enable colcon argcomplete
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/$ROS_DISTRO/

# export environmental variables
export ROS_DOMAIN_ID=4
LC_NUMERIC="en_US.UTF-8"
