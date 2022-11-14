#!/bin/bash

script_dir="$(realpath -e -- "$(dirname -- ${BASH_SOURCE[0]})")"

path_to_driver="$script_dir/../ur_driver/Universal_Robots_ROS2_Driver/ur_robot_driver/resources/ursim_driver"

pushd $path_to_driver  &>/dev/null
ROS2_DISTRO=foxy BUILD_MOVEIT2=0 docker-compose up
popd &>/dev/null
