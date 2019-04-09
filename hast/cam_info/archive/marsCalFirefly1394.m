function Calibration = marsCal()
% matlab -nojvm -nodesktop -r "marsCal();quit" 

HOSTID = 'mars';
cal.camXOffset = 0.0;
cal.camYOffset = 0.0;
cal.camZOffset = 0.0;
Left.GUID  = '00b09d0100af04f7';
Right.GUID = '00b09d0100af0503';
%% cal file name
cal.date = '20170620';
% cal.filename = 'create_balckfly_20170620';
% cal.filename = 'create_chameleon_20170711';
cal.filename = 'create_chameleon_20170712';
cal.yamlfile = [cal.filename '.yaml'];

disp(' ');disp(' ');
cal.path = ['/home/benjamin/ros/src/hast/cam_info/'];
cal.matpath = ['/home/benjamin/ros/data/calibrations/' cal.filename '/']; 
cal.launchpath = ['/home/benjamin/ros/src/hast/launch/'];

try mkdir(cal.matpath); catch; end;

%% Read in data from calibration file
disp(['loading calibration file:' cal.path cal.yamlfile]); disp(' ')
calID = fopen([cal.path cal.yamlfile]);
for s = 1:24
	rawCal(s).line(1,:) = fgets(calID);
end
clear s calID
%% Convert ascii text into relevant matrices
% ~~~~~~~~~~~~~~~~~~~~~~ Left Camera Matrices ~~~~~~~~~~~~~~~~~~~~~~

Left.Image.Width = eval(rawCal(21).line(1,:));
Left.Image.Height = eval(rawCal(24).line(1,:));

Left.DistortionMatrix = eval(rawCal(2).line(1,10:(end-2)));

A = eval(rawCal(3).line(1,10:(end-2)));
Left.CameraMatrix = reshape(A,[3,3])';
clear A

A = eval(rawCal(4).line(1,10:(end-2)));
Left.RectificationMatrix = reshape(A,[3,3])';
clear A

A = eval(rawCal(5).line(1,10:(end-2)));
Left.ProjectionMatrix = reshape(A,[4,3])';
clear A
% ~~~~~~~~~~~~~~~~~~~~~~ Right Camera Matrices ~~~~~~~~~~~~~~~~~~~~~~
Right.Image.Width = eval(rawCal(21).line(1,:));
Right.Image.Height = eval(rawCal(24).line(1,:));

Right.DistortionMatrix = eval(rawCal(8).line(1,10:(end-2)));

A = eval(rawCal(9).line(1,10:(end-2)));
Right.Camera.Matrix = reshape(A,[3,3])';
clear A

A = eval(rawCal(10).line(1,10:(end-2)));
Right.RectificationMatrix = reshape(A,[3,3])';
clear A

A = eval(rawCal(11).line(1,10:(end-2)));
Right.ProjectionMatrix = reshape(A,[4,3])';
clear A
%% ~~~~~~~~~~~~~~~~~~~~~~ Compute Calibration Parameters ~~~~~~~~~~~~~~~~~
Right.Camera.Inverse = inv(Right.Camera.Matrix);
M = Right.Camera.Inverse*Right.ProjectionMatrix;
Baseline = abs(M(1,4));
Cx = Right.ProjectionMatrix(1,3);
Cy = Right.ProjectionMatrix(2,3);
FocalLength = Right.ProjectionMatrix(1,1);

disp('M = ')
disp(M)

disp(['Baseline[m] = ' num2str(Baseline)])
disp(['FocalLength[px] = ' num2str(FocalLength)])
disp(['Cx[px] = ' num2str(Cx)])
disp(['Cy[px] = ' num2str(Cy)])
%% ~~~~~~~~~~~~~~~~~~~~~~ Save Calibration to mat files ~~~~~~~~~~~~~~~~~~
Calibration.Left = Left;
Calibration.Right = Right;

disp([[cal.matpath cal.filename '.mat saved']])
save([cal.matpath cal.filename '.mat'], 'Calibration');
%% ~~~~~~~~~~~~~~~~~~~~~~ Write  cam_HOSTID.launch ~~~~~~~~~~~~~~~~~~~~~~
Launch.filename = [cal.launchpath HOSTID '_firefly.launch'];
Launch.fileID = fopen(Launch.filename,'w');
fprintf(Launch.fileID, '<!-- -*- mode: XML -*- -->\n');
fprintf(Launch.fileID, '<!-- launch file auto-generated by: %s -->\n', cal.yamlfile);
fprintf(Launch.fileID, '<!-- launch file auto-generated on: %s -->\n\n', cal.date);
fprintf(Launch.fileID, '<launch>\n');

fprintf(Launch.fileID, '<node pkg="tf" type="static_transform_publisher" name="mars2left" args="0 0 0 0 0 0 /tb_mars /left 100"/> \n');
fprintf(Launch.fileID, '<arg name="%s_camera_guid" default="%s" />\n', 'left', Left.GUID);



% ----------- param group -----------
fprintf(Launch.fileID, '<!-- load stereo parameters -->\n');
fprintf(Launch.fileID, '<group ns="/hast/stereo" >\n');
fprintf(Launch.fileID, '\t<rosparam file="$(env HOME)/ros/src/hast/cam_info/%s_firefly_params.yaml" command="load"/>\n', HOSTID);
fprintf(Launch.fileID, '</group>\n\n');


fprintf(Launch.fileID, '<group ns="stereo">\n');

% ----------- left group -----------
fprintf(Launch.fileID, '<!-- left camera -->\n');


fprintf(Launch.fileID, '<group ns="%s" >\n', 'stereo');
fprintf(Launch.fileID, '\t<group ns="%s" >\n', 'left');
fprintf(Launch.fileID, '\t\t<node pkg="nodelet" type="nodelet" name="%s_nodelet_manager" args="manager" />\n', 'left');
fprintf(Launch.fileID, '\t\t<node pkg="nodelet" type="nodelet" name="%s_nodelet" args="load pointgrey_camera_driver/PointGreyCameraNodelet %s_nodelet_manager">\n', 'left', 'left');    
fprintf(Launch.fileID, '\t\t\t<param name="frame_id" value="%s" />\n', 'left');
fprintf(Launch.fileID, '\t\t\t<param name="serial" value="$(arg left_camera_serial)" />\n');
fprintf(Launch.fileID, '\t\t\t<rosparam file="/home/$(env USER)/ros/src/hast/cam_info/$(arg left_camera_serial).yaml" />\n');
fprintf(Launch.fileID, '\t\t</node>\n');
% fprintf(Launch.fileID, '\t\t<node pkg="nodelet" type="nodelet" name="image_proc_debayer" args="load image_proc/debayer %s_nodelet_manager"/>\n', 'left');
fprintf(Launch.fileID, '\t\t<node pkg="image_proc" type="image_proc" name="image_proc" />\n');
fprintf(Launch.fileID, '\t\t<node pkg="image_view" type="image_view" name="image_view" >\n');
fprintf(Launch.fileID, '\t\t\t<remap from="image" to="image_rect_color" />\n', 'left');
fprintf(Launch.fileID, '\t\t</node>\n');
fprintf(Launch.fileID, '\t</group>\n');
fprintf(Launch.fileID, '</group>\n\n');

% ----------- right group -----------
fprintf(Launch.fileID, '<!-- launch right camera -->\n');
fprintf(Launch.fileID, '<arg name="%s_camera_serial" default="%s" />\n', 'right', Right.SERIAL);
% fprintf(Launch.fileID, '<arg name="%s_camera_calibrated" default="1" />\n', 'right');
fprintf(Launch.fileID, '<group ns="%s" >\n', 'stereo');
fprintf(Launch.fileID, '\t<group ns="%s" >\n', 'right');
fprintf(Launch.fileID, '\t\t<node pkg="nodelet" type="nodelet" name="%s_nodelet_manager" args="manager" />\n', 'right');
fprintf(Launch.fileID, '\t\t<node pkg="nodelet" type="nodelet" name="%s_nodelet" args="load pointgrey_camera_driver/PointGreyCameraNodelet %s_nodelet_manager">\n', 'right', 'right');    
fprintf(Launch.fileID, '\t\t\t<param name="frame_id" value="%s" />\n', 'right');
fprintf(Launch.fileID, '\t\t\t<param name="serial" value="$(arg right_camera_serial)" />\n');
fprintf(Launch.fileID, '\t\t\t<rosparam file="/home/$(env USER)/ros/src/hast/cam_info/$(arg right_camera_serial).yaml" />\n');
fprintf(Launch.fileID, '\t\t</node>\n');
% fprintf(Launch.fileID, '\t\t<node pkg="nodelet" type="nodelet" name="image_proc_debayer" args="load image_proc/debayer %s_nodelet_manager"/>\n', 'right');
fprintf(Launch.fileID, '\t\t<node pkg="image_proc" type="image_proc" name="image_proc" />\n');
fprintf(Launch.fileID, '\t\t<node pkg="image_view" type="image_view" name="image_view" >\n');
fprintf(Launch.fileID, '\t\t\t<remap from="image" to="image_rect_color" />\n', 'right');
fprintf(Launch.fileID, '\t\t</node>\n');
fprintf(Launch.fileID, '\t</group>\n');
fprintf(Launch.fileID, '</group>\n\n');

% ----------- stereo image_proc nodelet -----------
fprintf(Launch.fileID, '<!-- <node pkg="stereo_image_proc" type="stereo_image_proc" name="stereo_image_proc"> -->\n');
fprintf(Launch.fileID, '\t<!-- <param name="approximate_sync" value="true"/> -->\n');
fprintf(Launch.fileID, '<!-- </node> -->\n');

fprintf(Launch.fileID, '<node ns="/stereo" pkg="stereo_image_proc" type="stereo_image_proc" name="stereo_image_proc" />\n');


% ----------- setchameleon -----------
% fprintf(Launch.fileID,'\n\n<!-- <node pkg="hast" type="setchameleon" name="setchameleon" output="screen"/> -->\n');
fprintf(Launch.fileID,'\n\n</launch>');
disp([Launch.filename ' created'])
%% ~~~~~~~~~~~~~~~~~~~~~~ Write  leftcam.yaml ~~~~~~~~~~~~~~~~~~~~~~
Left.ID_filename = [cal.path Left.SERIAL '.yaml'];
Left.yamlID = fopen(Left.ID_filename,'w');
fprintf(Left.yamlID,'# %s stereo-left camera calibration parameters\n', HOSTID);
fprintf(Left.yamlID,'# [narrow_stereo/left]\n');
fprintf(Left.yamlID,'# SERIAL %s\n', Left.SERIAL);
fprintf(Left.yamlID,'# yaml file auto-generated by : %s\n', cal.yamlfile);
fprintf(Left.yamlID,'# yaml file auto-generated on : %s\n', cal.date);
fprintf(Left.yamlID,'#### TAB CANNOT BE USED #####\n\n');

fprintf(Left.yamlID,'image_width: %i\n', Left.Image.Width);
fprintf(Left.yamlID,'image_height: %i\n\n', Left.Image.Height);

fprintf(Left.yamlID,'camera_matrix:\n');
fprintf(Left.yamlID,'  rows: %i\n', size(Left.CameraMatrix,1));
fprintf(Left.yamlID,'  cols: %i\n', size(Left.CameraMatrix,2));
fprintf(Left.yamlID,'  data: [');
	A = Left.CameraMatrix';
	for idx = 1:(numel(Left.CameraMatrix)-1)
		fprintf(Left.yamlID,'%6.16f, ',A(idx));
	end
	fprintf(Left.yamlID,'%6.16f]\n\n', A(end));
	clear A;

fprintf(Left.yamlID,'distortion_coefficients:\n');
fprintf(Left.yamlID,'  rows: %i\n',size(Left.DistortionMatrix,1));
fprintf(Left.yamlID,'  cols: %i\n',size(Left.DistortionMatrix,2));
fprintf(Left.yamlID,'  data: [');
	A = Left.DistortionMatrix';
	for idx = 1:(numel(Left.DistortionMatrix)-1)
		fprintf(Left.yamlID,'%6.16f, ',A(idx));
	end
	fprintf(Left.yamlID,'%6.16f]\n\n', A(end));
	clear A;

fprintf(Left.yamlID,'rectification_matrix:\n');
fprintf(Left.yamlID,'  rows: %i\n',size(Left.RectificationMatrix,1));
fprintf(Left.yamlID,'  cols: %i\n',size(Left.RectificationMatrix,2));
fprintf(Left.yamlID,'  data: [');
	A = Left.RectificationMatrix';
	for idx = 1:(numel(Left.RectificationMatrix)-1)
		fprintf(Left.yamlID,'%6.16f, ',A(idx));
	end
	fprintf(Left.yamlID,'%6.16f]\n\n', A(end));
	clear A;

fprintf(Left.yamlID,'projection_matrix:\n');
fprintf(Left.yamlID,'  rows: %i\n',size(Left.ProjectionMatrix,1));
fprintf(Left.yamlID,'  cols: %i\n',size(Left.ProjectionMatrix,2));
fprintf(Left.yamlID,'  data: [');
	A = Left.ProjectionMatrix';
	for idx = 1:(numel(Left.ProjectionMatrix)-1)
		fprintf(Left.yamlID,'%6.16f, ',A(idx));
	end
	fprintf(Left.yamlID,'%6.16f]\n\n', A(end));
	clear A;

fprintf(Left.yamlID,'# camera_info_url: file:///home/$(env USER)/ros/src/hast/cam_info/${NAME}.yaml\n\n');
fprintf(Left.yamlID,'camera_info_url: file:///home/benjamin/ros/src/hast/cam_info/${NAME}.yaml\n\n');

fprintf(Left.yamlID,'#Auto-Control States\n');
fprintf(Left.yamlID,'#Off (0): turns the feature off\n');
fprintf(Left.yamlID,'#Query (1): returns the current mode and (if possible) the value of the feature, with no change to the device\n');
fprintf(Left.yamlID,'#Auto (2): the camera sets the value continuously\n');
fprintf(Left.yamlID,'#Manual (3): sets a specific value from the corresponding parameter\n');
fprintf(Left.yamlID,'#OnePush (4): the camera sets the value once, then holds it constant\n');
fprintf(Left.yamlID,'#None (5): the camera does not provide this feature \n\n');

fprintf(Left.yamlID,'auto_white_balance: false\n');
fprintf(Left.yamlID,'white_balance_blue: 1000\n');
fprintf(Left.yamlID,'white_balance_red: 400\n');
fprintf(Left.yamlID,'enable_trigger: false\n');
fprintf(Left.yamlID,'# trigger_mode: mode14\n');
fprintf(Left.yamlID,'# trigger_delay: \n');
fprintf(Left.yamlID,'# trigger_parameter: \n');
fprintf(Left.yamlID,'# trigger_polarity: \n');
fprintf(Left.yamlID,'# trigger_source: \n');
fprintf(Left.yamlID,'auto_gain: false\n');
fprintf(Left.yamlID,'gain: 8.375\n');
fprintf(Left.yamlID,'# gamma: \n');
fprintf(Left.yamlID,'auto_exposure: false\n');
fprintf(Left.yamlID,'exposure: 0.5\n');
fprintf(Left.yamlID,'auto_shutter: false\n');
fprintf(Left.yamlID,'shutter_speed: 0.002 # seconds\n');

fprintf(Left.yamlID,'# auto_brightness: 3\n');
fprintf(Left.yamlID,'# auto_gamma: 3\n');
fprintf(Left.yamlID,'# auto_hue: 3\n');
fprintf(Left.yamlID,'# auto_saturation: 3\n');
fprintf(Left.yamlID,'# auto_sharpness: 3\n');
fprintf(Left.yamlID,'# auto_white_balance: 3\n');
fprintf(Left.yamlID,'# auto_iris: 3\n');
fprintf(Left.yamlID,'# auto_focus: 3\n');
fprintf(Left.yamlID,'# auto_zoom: 3\n\n');
fprintf(Left.yamlID,'# bayer_pattern: rggb\n');
fprintf(Left.yamlID,'# brightness: 170.0\n');
fprintf(Left.yamlID,'# frame_id: /camera\n');
fprintf(Left.yamlID,'# frame_rate: 60.0\n');
fprintf(Left.yamlID,'# gamma: 0\n');
fprintf(Left.yamlID,'# hue: 0.0\n');
fprintf(Left.yamlID,'# iris: 8.0\n');
fprintf(Left.yamlID,'# iso_speed: 400 #Fixed, the cameras dont operate on other ISO\n');
fprintf(Left.yamlID,'# reset_on_open: false\n');
fprintf(Left.yamlID,'# use_ros_time: true\n');
fprintf(Left.yamlID,'# video_mode: 640x480_mono8\n');
fprintf(Left.yamlID,['# bayer_method: '''' #default method, uses ros image_proc\n']);
fprintf(Left.yamlID,'# binning_x: 0\n');
fprintf(Left.yamlID,'# binning_y: 0\n');
fprintf(Left.yamlID,'# focus: 0.0\n');
fprintf(Left.yamlID,'# format7_color_coding: mono16\n');
fprintf(Left.yamlID,'# format7_packet_size: 0\n');
fprintf(Left.yamlID,'# roi_height: 0\n');
fprintf(Left.yamlID,'# roi_width: 0\n');
fprintf(Left.yamlID,'# saturation: 1.0\n');
fprintf(Left.yamlID,'# sharpness: 1.0\n');
fprintf(Left.yamlID,'# x_offset: 0\n');
fprintf(Left.yamlID,'# y_offset: 0\n');
fprintf(Left.yamlID,'# zoom: 0.0\n');
disp([Left.ID_filename ' created']);
%% ~~~~~~~~~~~~~~~~~~~~~~ Write rightcam.yaml ~~~~~~~~~~~~~~~~~~~~~~
Right.ID_filename = [cal.path Right.SERIAL '.yaml'];
Right.yamlID = fopen(Right.ID_filename,'w');
fprintf(Right.yamlID,'# %s stereo-right camera calibration parameters\n', HOSTID);
fprintf(Right.yamlID,'# [narrow_stereo/right]\n');
fprintf(Right.yamlID,'# SERIAL %s\n', Right.SERIAL);
fprintf(Right.yamlID,'# yaml file auto-generated by : %s\n', cal.yamlfile);
fprintf(Right.yamlID,'# yaml file auto-generated on : %s\n', cal.date);
fprintf(Right.yamlID,'#### TAB CANNOT BE USED #####\n\n');

fprintf(Right.yamlID,'image_width: %i\n', Right.Image.Width);
fprintf(Right.yamlID,'image_height: %i\n\n', Right.Image.Height);

fprintf(Right.yamlID,'camera_matrix:\n');
fprintf(Right.yamlID,'  rows: %i\n', size(Right.Camera.Matrix,1));
fprintf(Right.yamlID,'  cols: %i\n', size(Right.Camera.Matrix,2));
fprintf(Right.yamlID,'  data: [');
	A = Right.Camera.Matrix';
	for idx = 1:(numel(Right.Camera.Matrix)-1)
		fprintf(Right.yamlID,'%6.16f, ',A(idx));
	end
	fprintf(Right.yamlID,'%6.16f]\n\n', A(end));
	clear A;

fprintf(Right.yamlID,'distortion_coefficients:\n');
fprintf(Right.yamlID,'  rows: %i\n',size(Right.DistortionMatrix,1));
fprintf(Right.yamlID,'  cols: %i\n',size(Right.DistortionMatrix,2));
fprintf(Right.yamlID,'  data: [');
	A = Right.DistortionMatrix';
	for idx = 1:(numel(Right.DistortionMatrix)-1)
		fprintf(Right.yamlID,'%6.16f, ',A(idx));
	end
	fprintf(Right.yamlID,'%6.16f]\n\n', A(end));
	clear A;

fprintf(Right.yamlID,'rectification_matrix:\n');
fprintf(Right.yamlID,'  rows: %i\n',size(Right.RectificationMatrix,1));
fprintf(Right.yamlID,'  cols: %i\n',size(Right.RectificationMatrix,2));
fprintf(Right.yamlID,'  data: [');
	A = Right.RectificationMatrix';
	for idx = 1:(numel(Right.RectificationMatrix)-1)
		fprintf(Right.yamlID,'%6.16f, ',A(idx));
	end
	fprintf(Right.yamlID,'%6.16f]\n\n', A(end));
	clear A;

fprintf(Right.yamlID,'projection_matrix:\n');
fprintf(Right.yamlID,'  rows: %i\n',size(Right.ProjectionMatrix,1));
fprintf(Right.yamlID,'  cols: %i\n',size(Right.ProjectionMatrix,2));
fprintf(Right.yamlID,'  data: [');
	A = Right.ProjectionMatrix';
	for idx = 1:(numel(Right.ProjectionMatrix)-1)
		fprintf(Right.yamlID,'%6.16f, ',A(idx));
	end
	fprintf(Right.yamlID,'%6.16f]\n\n', A(end));
	clear A;

fprintf(Right.yamlID,'# camera_info_url: file:///home/$(env USER)/ros/src/hast/cam_info/${NAME}.yaml\n\n');
fprintf(Right.yamlID,'camera_info_url: file:///home/benjamin/ros/src/hast/cam_info/${NAME}.yaml\n\n');

fprintf(Right.yamlID,'#Auto-Control States\n');
fprintf(Right.yamlID,'#Off (0): turns the feature off\n');
fprintf(Right.yamlID,'#Query (1): returns the current mode and (if possible) the value of the feature, with no change to the device\n');
fprintf(Right.yamlID,'#Auto (2): the camera sets the value continuously\n');
fprintf(Right.yamlID,'#Manual (3): sets a specific value from the corresponding parameter\n');
fprintf(Right.yamlID,'#OnePush (4): the camera sets the value once, then holds it constant\n');
fprintf(Right.yamlID,'#None (5): the camera does not provide this feature \n\n');

fprintf(Right.yamlID,'auto_white_balance: false\n');
fprintf(Right.yamlID,'white_balance_blue: 1000\n');
fprintf(Right.yamlID,'white_balance_red: 400\n');
fprintf(Right.yamlID,'enable_trigger: false\n');
fprintf(Right.yamlID,'# trigger_mode: mode14\n');
fprintf(Right.yamlID,'# trigger_delay: \n');
fprintf(Right.yamlID,'# trigger_parameter: \n');
fprintf(Right.yamlID,'# trigger_polarity: \n');
fprintf(Right.yamlID,'# trigger_source: \n');
fprintf(Right.yamlID,'auto_gain: false\n');
fprintf(Right.yamlID,'gain: 8.375\n');
fprintf(Right.yamlID,'# gamma: \n');
fprintf(Right.yamlID,'auto_exposure: false\n');
fprintf(Right.yamlID,'exposure: 0.5\n');
fprintf(Right.yamlID,'auto_shutter: false\n');
fprintf(Right.yamlID,'shutter_speed: 0.002 # seconds\n');

fprintf(Right.yamlID,'# auto_brightness: 3\n');
fprintf(Right.yamlID,'# auto_gamma: 3\n');
fprintf(Right.yamlID,'# auto_hue: 3\n');
fprintf(Right.yamlID,'# auto_saturation: 3\n');
fprintf(Right.yamlID,'# auto_sharpness: 3\n');
fprintf(Right.yamlID,'# auto_white_balance: 3\n');
fprintf(Right.yamlID,'# auto_iris: 3\n');
fprintf(Right.yamlID,'# auto_focus: 3\n');
fprintf(Right.yamlID,'# auto_zoom: 3\n\n');
fprintf(Right.yamlID,'# bayer_pattern: rggb\n');
fprintf(Right.yamlID,'# brightness: 170.0\n');
fprintf(Right.yamlID,'# frame_id: /camera\n');
fprintf(Right.yamlID,'# frame_rate: 60.0\n');
fprintf(Right.yamlID,'# gamma: 0\n');
fprintf(Right.yamlID,'# hue: 0.0\n');
fprintf(Right.yamlID,'# iris: 8.0\n');
fprintf(Right.yamlID,'# iso_speed: 400 #Fixed, the cameras dont operate on other ISO\n');
fprintf(Right.yamlID,'# reset_on_open: false\n');
fprintf(Right.yamlID,'# use_ros_time: true\n');
fprintf(Right.yamlID,'# video_mode: 640x480_mono8\n');
fprintf(Right.yamlID,['# bayer_method: '''' #default method, uses ros image_proc\n']);
fprintf(Right.yamlID,'# binning_x: 0\n');
fprintf(Right.yamlID,'# binning_y: 0\n');
fprintf(Right.yamlID,'# focus: 0.0\n');
fprintf(Right.yamlID,'# format7_color_coding: mono16\n');
fprintf(Right.yamlID,'# format7_packet_size: 0\n');
fprintf(Right.yamlID,'# roi_height: 0\n');
fprintf(Right.yamlID,'# roi_width: 0\n');
fprintf(Right.yamlID,'# saturation: 1.0\n');
fprintf(Right.yamlID,'# sharpness: 1.0\n');
fprintf(Right.yamlID,'# x_offset: 0\n');
fprintf(Right.yamlID,'# y_offset: 0\n');
fprintf(Right.yamlID,'# zoom: 0.0\n');
disp([Right.ID_filename ' created']);
%% ~~~~~~~~~~~~~~~~~~~~~~ Write HOSTID_stereo_params.yaml ~~~~~~~~~~~~~~~~~~~~~~
Stereo.param_filename = [cal.path HOSTID '_pgr_params.yaml']; 

Stereo.yamlID = fopen(Stereo.param_filename,'w');
fprintf(Stereo.yamlID,'# %s stereo camera calibration parameters\n', HOSTID);
fprintf(Stereo.yamlID,'# Right SERIAL %s\n', Right.SERIAL);
fprintf(Stereo.yamlID,'# Left SERIAL %s\n', Left.SERIAL);
fprintf(Stereo.yamlID,'# yaml file auto-generated by : %s\n', cal.yamlfile);
fprintf(Stereo.yamlID,'# yaml file auto-generated on : %s\n', cal.date);
fprintf(Stereo.yamlID,'#### TAB CANNOT BE USED #####\n\n');

fprintf(Stereo.yamlID,'FocalLength: %6.16f\n',FocalLength);
fprintf(Stereo.yamlID,'Baseline: %6.16f\n',Baseline);
fprintf(Stereo.yamlID,'RightOffset: %6.16f\n',Right.ProjectionMatrix(1,4));
% fprintf(Stereo.yamlID,'cal.camXOffset: %6.6f\n',cal.camXOffset);
% fprintf(Stereo.yamlID,'cal.camYOffset: %6.6f\n',cal.camYOffset);
% fprintf(Stereo.yamlID,'cal.camZOffset: %6.6f\n',cal.camZOffset);
fprintf(Stereo.yamlID,'camXOffset: %6.6f\n',cal.camXOffset);
fprintf(Stereo.yamlID,'camYOffset: %6.6f\n',cal.camYOffset);
fprintf(Stereo.yamlID,'camZOffset: %6.6f\n',cal.camZOffset);
fprintf(Stereo.yamlID,'Cx: %6.16f\n',Cx);
fprintf(Stereo.yamlID,'Cy: %6.16f\n',Cy);
fprintf(Stereo.yamlID,'dRx: 0.00\n');
fprintf(Stereo.yamlID,'dRz: 0.00');

disp([Stereo.param_filename ' created']);
disp(' ');disp(' ');

end



% // <!-- -*- mode: XML -*- -->

% // <launch>
% // <node pkg="tf" type="static_transform_publisher" name="mars2left" args="0 0 0 0 0 0 /tb_mars /left 100"/> 
% // <!-- <group ns="/hast/stereo" > -->
% // 	<!-- <rosparam file="$(env HOME)/ros/src/hast/cam_info/mars_pgr_params.yaml" command="load"/> -->
% // <!-- </group> -->
% // <group ns="stereo">
% // <!-- left camera -->
% // 	<node pkg="nodelet" type="nodelet" name="left_manager" args="manager" />
% // 	<node pkg="nodelet" type="nodelet" name="left_1394" args="load camera1394/driver left_manager" >
% // 		<rosparam file="$(env HOME)/.ros/camera_info/00b09d0100af04f7.yaml" /> 
% // 		<remap from="camera" to="left" />
% // 		<param name="bayer_pattern" value="rggb" />
% // 		<param name="guid" value="00b09d0100af04f7" />
% // 	</node>
% // 	<node  ns="left" pkg="image_proc" type="image_proc" name="image_proc" />

% // <!-- right camera -->
% // 	<node pkg="nodelet" type="nodelet" name="right_manager" args="manager" />
% // 	<node pkg="nodelet" type="nodelet" name="right_1394" args="load camera1394/driver right_manager" >
% // 		<rosparam file="$(env HOME)/.ros/camera_info/00b09d0100af0503.yaml" /> 
% // 		<remap from="camera" to="right" />
% // 		<param name="bayer_pattern" value="rggb" />
% // 		<param name="guid" value="00b09d0100af0503" />
% // 	</node>
% // 	<node  ns="right" pkg="image_proc" type="image_proc" name="image_proc" />

% // 	<node pkg="stereo_image_proc" type="stereo_image_proc" name="stereo_image_proc" />

% // </group>
% // <!-- 	<node pkg="viso2_ros" type="stereo_odometer" name="stereo_odometer">
% // 		<remap from="stereo/left/" to="/left/camera/"/> 
% // 		<remap from="stereo/right/" to="/right/camera/"/> 
% // 		<remap from="image" to="image_rect"/>
% // 		<param name="base_link_frame_id" value="/map"/>
% // 	</node>
% //  -->

% // </launch>