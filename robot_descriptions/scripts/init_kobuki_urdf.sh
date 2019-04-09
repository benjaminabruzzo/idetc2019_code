#!/bin/bash
# if [ -d ~/.gazebo/models/hast_kobuki ]; then
#     rm -rf ~/.gazebo/models/hast_kobuki
# fi

# mkdir -p ~/.gazebo/models/hast_kobuki/urdf/
# cp $(rospack find hast_kobuki_description)/urdf/hast_kobuki.urdf ~/.gazebo/models/hast_kobuki/urdf/hast_kobuki.urdf
# cp $(rospack find hast_kobuki_description)/urdf/hast_kobuki.config ~/.gazebo/models/hast_kobuki/model.config
# # . init_kobuki_urdf.sh && roslaunch hast_kobuki_description hast_kobuki.launch


if [ -d ~/.gazebo/models/test_camera ]; then
    rm -rf ~/.gazebo/models/test_camera
fi
mkdir -p ~/.gazebo/models/test_camera/urdf/
rosrun xacro xacro --inorder $(rospack find hast_kobuki_description)/urdf/test_camera.urdf.xacro > $(rospack find hast_kobuki_description)/urdf/test_camera.urdf
cp $(rospack find hast_kobuki_description)/urdf/test_camera.urdf ~/.gazebo/models/test_camera/urdf/test_camera.urdf
cp $(rospack find hast_kobuki_description)/urdf/test_camera.config ~/.gazebo/models/test_camera/model.config
# . init_kobuki_urdf.sh && roslaunch hast_kobuki_description test_camera.launch

if [ -d ~/.gazebo/models/hast_kobuki_xacro ]; then
    rm -rf ~/.gazebo/models/hast_kobuki_xacro
fi
mkdir -p ~/.gazebo/models/hast_kobuki_xacro/urdf/
rosrun xacro xacro --inorder $(rospack find hast_kobuki_description)/urdf/hast_kobuki_xacro.urdf.xacro > $(rospack find hast_kobuki_description)/urdf/hast_kobuki_xacro.urdf
cp $(rospack find hast_kobuki_description)/urdf/hast_kobuki_xacro.urdf ~/.gazebo/models/hast_kobuki_xacro/urdf/hast_kobuki_xacro.urdf
cp $(rospack find hast_kobuki_description)/urdf/hast_kobuki_xacro.config ~/.gazebo/models/hast_kobuki_xacro/model.config
# . init_kobuki_urdf.sh && roslaunch hast_kobuki_description hast_kobuki_xacro.launch

if [ -d ~/.gazebo/models/hast_kobuki_simple ]; then
    rm -rf ~/.gazebo/models/hast_kobuki_simple
fi
mkdir -p ~/.gazebo/models/hast_kobuki_simple/urdf/
rosrun xacro xacro --inorder $(rospack find hast_kobuki_description)/urdf/hast_kobuki_simple.urdf.xacro > $(rospack find hast_kobuki_description)/urdf/hast_kobuki_simple.urdf
cp $(rospack find hast_kobuki_description)/urdf/hast_kobuki_simple.urdf ~/.gazebo/models/hast_kobuki_simple/urdf/hast_kobuki_simple.urdf
cp $(rospack find hast_kobuki_description)/urdf/hast_kobuki_simple.config ~/.gazebo/models/hast_kobuki_simple/model.config
# . init_kobuki_urdf.sh && roslaunch hast_kobuki_description hast_kobuki_simple.launch
