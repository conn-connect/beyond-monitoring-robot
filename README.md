# Monitoring robot prototype
Beyond Robotics Co.,ltd

## To install
source /opt/ros/foxy/setup.bash
cd ~/robot_ws/
colcon build --symlink-install
source ~/robot_ws/install/setup.bash

## Alias settings
alias eb='gedit ~/.bashrc'
alias sb='source ~/.bashrc'

alias vino='/usr/lib/vino/vino-server &'

alias cba='colcon build --symlink-install'
alias cbp='colcon build --symlink-install --packages-select'
alias gh='greenhouse-simulation'
alias dd='beyond-diffdrive'
alias killg='killall -9 gzserver && killall -9 gzclient && killall -9 rosmaster'

alias srf='source /opt/ros/foxy/setup.bash'
alias source_robot='source ~/robot_ws/install/setup.bash'
alias rosfoxy='source /opt/ros/foxy/setup.bash && source ~/robot_ws/install/local_setup.bash'

alias ros2launchgh='ros2 launch greenhouse_simulation greenhouse_simulation.launch.py'
alias ros2rundd='ros2 run beyond_robotics_control beyond_diffdrive'
alias ros2runld='ros2 run beyond_robotics_control lane_detection'
alias ros2runkc='ros2 run beyond_robotics_control keyboard_control'
alias ros2runteleop='ros2 run teleop_twist_keyboard teleop_twist_keyboard'

// add aliases to ~./bashrc and use alias

## To launch
ros2rundd # motor drive
ros2runld # lane detection
ros2runkc # keyboard operator with lane detection
ros2runteleop # teleop for move robot
