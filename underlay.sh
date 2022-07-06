# source ros2 setup file
{
  {
    {
      echo -n "Setting up humble version... " &&
      source /opt/ros/humble/setup.bash
    } ||
    {
      echo -n -e "\nSetting up foxy version instead... " &&
      source /opt/ros/foxy/setup.bash
    }
  } &&
  echo "Done"
}

# enable colcon argcomplete
{
  echo -n "Eabling colcon auto-complete... " &&
  source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash &&
  source /usr/share/colcon_cd/function/colcon_cd.sh &&
  export _colcon_cd_root=/opt/ros/$ROS_DISTRO/ &&
  echo "Done"
}

# export environmental variables
{
  echo -n "Exporting environmental variables... " &&
  export ROS_DOMAIN_ID=4 &&
  export LC_NUMERIC="en_US.UTF-8" &&
  echo "Done"
}
