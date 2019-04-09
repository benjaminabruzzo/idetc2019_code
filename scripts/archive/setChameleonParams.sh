#!/bin/bash

rosrun dynamic_reconfigure dynparam load /left/camera/left_nodelet $HOME/ros/src/hast/cam_info/pgr.yaml
rosrun dynamic_reconfigure dynparam load /right/camera/right_nodelet $HOME/ros/src/hast/cam_info/pgr.yaml
