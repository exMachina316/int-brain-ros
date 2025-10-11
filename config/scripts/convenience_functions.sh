#!/bin/bash

# source workspace alias
alias wss="source $WORKSPACE/install/setup.bash && echo \"Sourced workspace\""

# build workspace alias
alias wsb="cd $WORKSPACE && colcon build --symlink-install && wss"

# god forsaken rosdep install alias
## if environment variable PLATFORM is "sbc", skip the "int_brain_gazebo" package
if [ "$PLATFORM" == "sbc" ]; then
    alias rdi="sudo rosdep install -y --from-paths $WORKSPACE/src --ignore-src --rosdistro $ROS_DISTRO --skip-keys=\"ros_gz_sim rviz2\""
else
    alias rdi="sudo rosdep install -y --from-paths $WORKSPACE/src --ignore-src --rosdistro $ROS_DISTRO"
fi

# enable colcon argument completion
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash