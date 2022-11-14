#!/bin/bash

script_dir="$(realpath -e -- "$(dirname -- ${BASH_SOURCE[0]})")"

source $script_dir/../../install/local_setup.bash

ros2 launch urscript_bridge launch.py
