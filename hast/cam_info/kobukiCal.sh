#!/bin/bash
matlab -nojvm -nodesktop -r "cd('/home/benjamin/ros/src/hast/cam_info');kobukiCalChameleonPGR('linux')" 

roscd hast/cam_info && cp 16369047.yaml ~/.ros/camera_info && cp 16306423.yaml ~/.ros/camera_info