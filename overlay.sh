# source custom packages
{
  echo -n "Setting up overlay packages... " &&
  source install/local_setup.bash &&
  echo "Done"

  echo -n "Exporting Gazebo plugins path... " &&
  export GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins/
  echo "Done"
}
