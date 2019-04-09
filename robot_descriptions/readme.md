UGV description:

`
<include file="$(find robot_descriptions)/UGV2/launch/ugv2_$(arg vicon_date)_$(arg vicon_run).launch" />
`


`
<include file="$(find robot_descriptions)/UGV2/launch/spawn_ugv.launch" >
	<arg name="model" 				value="$(find robot_descriptions)/UGV2/urdf/hast_ugv2_kobuki_stereo.urdf.xacro"/> 
	<arg name="robot_ns" 			value="/gazebo/kobuki"/> 
	<arg name="raw_wheel_cmd_topic" value="/gazebo/kobuki/cmd_vel_raw" />
	<arg name="x"	value="$(arg map_x)"/>
	<arg name="y"	value="$(arg map_y)"/>
	<arg name="z"   value="$(arg map_z)"/>
	<arg name="Y"	value="$(arg map_Y)"/>
</include>
`


