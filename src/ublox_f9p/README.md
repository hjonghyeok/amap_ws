#GPS관련 alias 설정
alias cgw='cd ~/gps_ws'
alias cgs='cd ~/gps_ws/src'
alias cgm='cd ~/gps_ws && catkin_make'
alias cgd='cd ~/gps_ws && source devel/setup.bash'
alias gps='cd ~/gps_ws && roslaunch ublox_gps ublox_device.launch'
alias ntrip='cd ~/gps_ws && roslaunch ntrip_ros ntrip_ros.launch'
alias utm_convert='cd ~/gps_ws && roslaunch utm_lla coordinate_convertion.launch'
alias utm_distance='cd ~/gps_ws && rosrun utm_gps_distance gps_node'

alias acm0_chmod='sudo chmod 777 /dev/ttyACM0'


# ROS driver for ublox ZED-F9P receiver

This is for the ArduSimple ZED-F9P boards. (Mainly affects which port on the board is being used.) I am connecting the F9P board to a Linux laptop with USB cable. (MovingBaseline requires connecting a single wire between two receiver boards.)

I copied the ublox ROS driver from https://github.com/bao-eng/ublox

Looking at my initial commit comments I really didn't change anything to get it to work.

I added ROS topic RTCM correction support so the board produces an RTK fix. (The correction data is sent through the ROS driver node.) The RTCM server is here: https://github.com/ros-agriculture/ntrip_ros A similar server (may be better than mine) is here: https://github.com/dayjaby/ntrip_ros

I added launch files and yaml files to allow running two F9P boards at the same time. This allows operation in MovingBaseline mode. 

**I need to add the uBlox config files which get manually loaded on to the boards through uBlox u-center software.**

Below is original README from bao-eng that I started with:

=======================================================================

# ROS driver for ublox ZED-F9P receiver

Just quick hardcode to publish some UBX messages to ROS.
Disabled configuration of the receiver via yaml. Reciever should be configured via u-center software.
Hardcoded to work as HPG Rover device.
NavRELPOSNED.msg updated to match u-blox 9 protocol version 27.1

## RTCM Messages
It may also require this package: https://github.com/tilk/rtcm_msgs

## Options

zed-f9p.yaml (only for seting up device connection and published messages)

## Launch

```roslaunch ublox_gps ublox_zed-f9p.launch```

