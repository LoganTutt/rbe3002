#!/bin/bash

ip=`ifconfig  | grep 'inet addr:'| grep -v '127.0.0.1' | cut -d: -f2 | awk '{ print $1}'`
#echo $ip
if [[ $ip == *192.168* ]];
then
    #echo "on turtlebot"
    export ROS_MASTER_URI=http://192.168.1.101:11511
    export ROS_HOSTNAME=$ip
    export ROS_IP=$ip
else
    #echo "in sim"
    export ROS_MASTER_URI=http://localhost:11511
    export ROS_HOSTNAME=localhost
    export ROS_IP=localhost
fi
