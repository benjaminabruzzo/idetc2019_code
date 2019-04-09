# set MTU from command line:
sudo ifconfig eth0 mtu 9014
sudo ifconfig enp2s0 mtu 9014
sudo ifconfig enp3s0 mtu 9014

# add the following to /etc/sysctl.conf
###################################################################
# Memory buffer issues with pointgrey cameras:
net.core.rmem_max = 16777215
net.core.wmem_max = 16777215
###################################################################
# call using sudo sysctl -p
sudo sysctl -p

# Also, set GigE Packet delay to max
# gpio voltage register: 19d0
# set LEAST significant bit to 1 to turn on
# 

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~``
# TRENDNET_CREATE admin password: 93CY865&

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~``
# List detected cameras:
rosrun pointgrey_camera_driver list_cameras

# mars cameras
# [0]Serial: 16335922, Model: Blackfly BFLY-PGE-13H2C, Vendor: Point Grey Research, Sensor: Sharp RJ33J3CA0DT (1/3" Color CCD), Resolution: 1288x964, Color: true, Firmware Version: 1.48.3.0
# [1]Serial: 17180307, Model: Blackfly BFLY-PGE-13H2C, Vendor: Point Grey Research, Sensor: Sharp RJ33J3CA0DT (1/3" Color CCD), Resolution: 1288x964, Color: true, Firmware Version: 1.62.3.0

# create cameras
# [0]Serial: 16452463, Model: Blackfly BFLY-PGE-09S2C, Vendor: Point Grey Research, Sensor: Sony ICX692 (1/3" Color CCD), Resolution: 1288x728, Color: true, Firmware Version: 1.46.3.0
# [1]Serial: 17094423, Model: Blackfly BFLY-PGE-09S2C, Vendor: Point Grey Research, Sensor: Sony ICX692 (1/3" Color CCD), Resolution: 1288x728, Color: true, Firmware Version: 1.46.3.0

#########################################################################
# ~~~~~~~~~~~~~ changes to /etc/netowrk/interfaces ~~~~~~~~~~~~~~~~~~~~ #
# before changing the ethernet options define the camera properties:
# open flycap and slect a camera
# change register 0014 (gigE Vision) to:
00 00 00 07 
00000000 00000000 00000000 00000111
# this changes the camera to persistent ip address, then
# change register 064c (ip address) to: 192.168.128.136
C0 A8 80 88 #(hex)
# change register 065c (subnet) to: 255.255.0.0
FF FF 00 00 #(hex)
# change register 066c (gateway) to: 0.0.0.0
00 00 00 00 #(hex)


#########################################################################
# ~~~~~~~~~~~~~ changes to /etc/netowrk/interfaces ~~~~~~~~~~~~~~~~~~~~ #
# interfaces(5) file used by ifup(8) and ifdown(8)
auto lo
iface lo inet loopback

## create uppermost port (right camera)
auto eth0
iface eth0 inet static
address 192.168.128.1
netmask 255.255.0.0

## create lowermost port
auto eth1
iface eth1 inet static
address 192.168.129.1
netmask 255.255.0.0




#########################################################################
# ~~~~~~~~~~~~~ reference websites ~~~~~~~~~~~~~~~~~~~~ #

https://www.ptgrey.com/KB/10849