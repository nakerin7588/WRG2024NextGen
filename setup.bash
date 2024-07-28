#! /bin/bash

__setup_path=(/opt/ros/humble/setup.bash $PWD/src/install/setup.bash /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash /usr/share/gazebo/setup.bash)
__setup_path_name=("Humble setup" "Project setup" "colcon-argcomplete setup" "gazebo setup")

for i in $(seq 0 3);
do
  if [ ! -f ${__setup_path[i]} ]; then
    echo "${__setup_path_name[i]} file not found at ${__setup_path[i]}"
    exit 1
  fi
done

for i in $(seq 0 3);
do
  source ${__setup_path[i]}
done

export TURTLEBOT3_MODEL=waffle
export rmw_implementtion=rmw_cyclnedds_cpp

echo "source successfully"
