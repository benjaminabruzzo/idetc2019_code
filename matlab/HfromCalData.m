%% Compute ugv and gator baord transforms at the time of there stereo raw images
% Spline UGV data
[data.caldata.ugvk.q_x.spline, data.caldata.ugvk.q_x.atRawtime]  = splineAndInterp(data.vicon.ugvk.time, data.vicon.ugvk.q_i(:,1), data.ugvStereo.leftraw.time);
[data.caldata.ugvk.q_y.spline, data.caldata.ugvk.q_y.atRawtime]  = splineAndInterp(data.vicon.ugvk.time, data.vicon.ugvk.q_i(:,2), data.ugvStereo.leftraw.time);
[data.caldata.ugvk.q_z.spline, data.caldata.ugvk.q_z.atRawtime]  = splineAndInterp(data.vicon.ugvk.time, data.vicon.ugvk.q_i(:,3), data.ugvStereo.leftraw.time);
[data.caldata.ugvk.q_w.spline, data.caldata.ugvk.q_w.atRawtime]  = splineAndInterp(data.vicon.ugvk.time, data.vicon.ugvk.q_i(:,4), data.ugvStereo.leftraw.time);
 data.caldata.ugvk.q.atRawtime = [data.caldata.ugvk.q_x.atRawtime data.caldata.ugvk.q_y.atRawtime data.caldata.ugvk.q_z.atRawtime data.caldata.ugvk.q_w.atRawtime];

[data.caldata.ugvk.P_x.spline, data.caldata.ugvk.P_x.atRawtime]  = splineAndInterp(data.vicon.ugvk.time, data.vicon.ugvk.P.vicon(:,1), data.ugvStereo.leftraw.time);
[data.caldata.ugvk.P_y.spline, data.caldata.ugvk.P_y.atRawtime]  = splineAndInterp(data.vicon.ugvk.time, data.vicon.ugvk.P.vicon(:,2), data.ugvStereo.leftraw.time);
[data.caldata.ugvk.P_z.spline, data.caldata.ugvk.P_z.atRawtime]  = splineAndInterp(data.vicon.ugvk.time, data.vicon.ugvk.P.vicon(:,3), data.ugvStereo.leftraw.time);
 data.caldata.ugvk.P.atRawtime = [data.caldata.ugvk.P_x.atRawtime data.caldata.ugvk.P_y.atRawtime data.caldata.ugvk.P_z.atRawtime];

% Spline Gator data
[data.caldata.gator_.q_x.spline, data.caldata.gator_.q_x.atRawtime]  = splineAndInterp(data.vicon.gator_.time, data.vicon.gator_.q_i(:,1), data.ugvStereo.leftraw.time);
[data.caldata.gator_.q_y.spline, data.caldata.gator_.q_y.atRawtime]  = splineAndInterp(data.vicon.gator_.time, data.vicon.gator_.q_i(:,2), data.ugvStereo.leftraw.time);
[data.caldata.gator_.q_z.spline, data.caldata.gator_.q_z.atRawtime]  = splineAndInterp(data.vicon.gator_.time, data.vicon.gator_.q_i(:,3), data.ugvStereo.leftraw.time);
[data.caldata.gator_.q_w.spline, data.caldata.gator_.q_w.atRawtime]  = splineAndInterp(data.vicon.gator_.time, data.vicon.gator_.q_i(:,4), data.ugvStereo.leftraw.time);
 data.caldata.gator_.q.atRawtime = [data.caldata.gator_.q_x.atRawtime data.caldata.gator_.q_y.atRawtime data.caldata.gator_.q_z.atRawtime data.caldata.gator_.q_w.atRawtime];
 
[data.caldata.gator_.P_x.spline, data.caldata.gator_.P_x.atRawtime]  = splineAndInterp(data.vicon.gator_.time, data.vicon.gator_.P.vicon(:,1), data.ugvStereo.leftraw.time);
[data.caldata.gator_.P_y.spline, data.caldata.gator_.P_y.atRawtime]  = splineAndInterp(data.vicon.gator_.time, data.vicon.gator_.P.vicon(:,2), data.ugvStereo.leftraw.time);
[data.caldata.gator_.P_z.spline, data.caldata.gator_.P_z.atRawtime]  = splineAndInterp(data.vicon.gator_.time, data.vicon.gator_.P.vicon(:,3), data.ugvStereo.leftraw.time);
 data.caldata.gator_.P.atRawtime = [data.caldata.gator_.P_x.atRawtime data.caldata.gator_.P_y.atRawtime data.caldata.gator_.P_z.atRawtime];

for i = 1:length(data.ugvStereo.leftraw.time)
    % compute camera frame to gator frame
    Rl = data.caldata.leftposes.R.(matlab.lang.makeValidName(['R' num2str(i)]))';
    tl = data.caldata.leftposes.t.(matlab.lang.makeValidName(['t' num2str(i)]));
    Rr = data.caldata.rightposes.R.(matlab.lang.makeValidName(['R' num2str(i)]))';
    tr = data.caldata.rightposes.t.(matlab.lang.makeValidName(['t' num2str(i)]));
    
    data.caldata.Hlcam2gator(:,:,i) = Ryh(pi)*wrapH(Rl, tl);
    data.caldata.Hrcam2gator(:,:,i) = Ryh(pi)*wrapH(Rr, tr);

    % compute gator frame to ugv frame
    ugv_q = data.caldata.ugvk.q.atRawtime(i,:); 
    data.caldata.ugvk.dcm(:,:,i) = QwedgeDCM(ugv_q);
    data.caldata.ugvk.H_vic2lo(:,:,i) = wrapH(data.caldata.ugvk.dcm(:,:,i), data.caldata.ugvk.P.atRawtime(i,:)');
    
    gator_q = data.caldata.gator_.q.atRawtime(i,:);
    data.caldata.gator_.dcm(:,:,i) = QwedgeDCM(gator_q);
    data.caldata.gator_.H_vic2lo(:,:,i) = wrapH(data.caldata.gator_.dcm(:,:,i), data.caldata.gator_.P.atRawtime(i,:)');
    
    data.caldata.Hgator2ugv(:,:,i) = data.caldata.ugvk.H_vic2lo(:,:,i) * invertH(data.caldata.gator_.H_vic2lo(:,:,i));
    data.caldata.Hugv2gator(:,:,i) = invertH(data.caldata.Hgator2ugv(:,:,i));
    
	delta(i,:) = data.caldata.ugvk.P.atRawtime(i,:) - data.caldata.gator_.P.atRawtime(i,:);
    rms_delta(i,1) = sqrt(delta(i,:) * delta(i,:)');

    
    % combine into camera to ugv
    data.caldata.Hlcam2ugv(:,:,i) = data.caldata.Hgator2ugv(:,:,i) * data.caldata.Hlcam2gator(:,:,i)*Rzh(pi);
    data.caldata.Hrcam2ugv(:,:,i) = data.caldata.Hgator2ugv(:,:,i) * data.caldata.Hrcam2gator(:,:,i)*Rzh(pi);
    
    rms_H(i,1) = sqrt(data.caldata.Hlcam2ugv(1:3,4,i)'*data.caldata.Hlcam2ugv(1:3,4,i));
    rms_H(i,2) = sqrt(data.caldata.Hrcam2ugv(1:3,4,i)'*data.caldata.Hlcam2ugv(1:3,4,i));
    
    
    data.caldata.Hl_t(:,i) = data.caldata.Hlcam2ugv(1:3,4,i);
    data.caldata.Hr_t(:,i) = data.caldata.Hrcam2ugv(1:3,4,i);
end
% Rz(-(90)*pi/180)*Rx(-(90-20)*pi/180)

% figure(5000); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
set(0, 'DefaultFigureVisible', 'on');
figHandles = findall(0, 'Type', 'figure');
set(figHandles(:), 'visible', 'on');
clear figHandles

figure(5000); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
    for i = 1:length(data.ugvStereo.leftraw.time)
        xaxis = [data.caldata.ugvk.P.atRawtime(i,:)' data.caldata.ugvk.P.atRawtime(i,:)'+0.25*data.caldata.ugvk.dcm(1,:,i)'];
        yaxis = [data.caldata.ugvk.P.atRawtime(i,:)' data.caldata.ugvk.P.atRawtime(i,:)'+0.25*data.caldata.ugvk.dcm(2,:,i)'];
        h1 = plot(xaxis(2,:), xaxis(1,:), 'b', 'LineWidth',4,'displayname', 'ugv x axis');
        h2 = plot(yaxis(2,:), yaxis(1,:), 'r', 'LineWidth',4,'displayname', 'ugv y axis');
    end
    
    xaxis = [data.caldata.gator_.P.atRawtime(end,:)' data.caldata.gator_.P.atRawtime(end,:)'+0.25*data.caldata.gator_.dcm(1,:,end)'];
    yaxis = [data.caldata.gator_.P.atRawtime(end,:)' data.caldata.gator_.P.atRawtime(end,:)'+0.25*data.caldata.gator_.dcm(2,:,end)'];
    zaxis = [data.caldata.gator_.P.atRawtime(end,:)' data.caldata.gator_.P.atRawtime(end,:)'+0.25*data.caldata.gator_.dcm(3,:,end)'];
    h3 = plot(xaxis(2,:), xaxis(1,:), 'g', 'LineWidth',4,'displayname', 'gator x axis');
    h4 = plot(yaxis(2,:), yaxis(1,:), 'm', 'LineWidth',4,'displayname', 'gator y axis');
    h5 = plot(zaxis(2,:), zaxis(1,:), 'k', 'LineWidth',4,'displayname', 'gator z axis');
    
    hold off
    xlabel('y [meters]')
    ylabel('x [meters]')
    grid on
    axis square
    set(gca,'YAxisLocation','right','xdir','reverse');
    try
        legend('toggle')
        legend([h1, h2, h3, h4, h5], 'Location', 'northwest')
        clear h1 h2 h3 h4 h5
    catch
    end
    clear xaxis yaxis zaxis
    
%%
imdir = '/Users/benjamin/ros/data/20180224/011/rectified/';

IL = imread([imdir 'left_rect_00053.png']);
IR = imread([imdir 'right_rect_00053.png']);

% F = zeros(3); F(2,3) = 1; F(3,2) = -1;
F = data.caldata.stereo.F;
vgg_gui_F(IL,IR,F')


%%
% for j = 1:53
% for j = [16; 23; 27; 29; 30; 31; 32; 35; 37; 40; 43; 44; 45; 46; 47; 48; 49; 51]
for j= 44
    data.caldata.H_test_cam2ugv = data.caldata.Hrcam2ugv(:,:,j);
    % data.caldata.H_test_cam2ugv(1:3,4) = 0.5*(data.caldata.Hlcam2ugv(1:3,4,50) + data.caldata.Hrcam2ugv(1:3,4,50));
    data.caldata.H_test_cam2ugv(1:3,4) = [0.0 0.0 0.0]';
    % data.caldata.H_test_cam2ugv(1:3,4) = [...;0.0 0.0 0.0]';
    %    0.205434514609920;...
    %    0.066102778271172;...
    %    0.357270579547507];
    data.caldata.H_test_cam2ugv(1:3,4) = [...;0.0 0.0 0.0]';
                    0.216143033313071; ...
                    0.029736772947058; ...
                    0.323833236722482];

    data.caldata.uav.P_ugv = [];
    data.caldata.Red.P_ugv = [];
    data.caldata.Blue.P_ugv = [];
    data.caldata.Green.P_ugv = [];
    
% compute RGB locations
    for i = 1:length(data.ugvStereo.Red.P_cam)
        data.caldata.Red.P_ugv(i,:) = (data.caldata.H_test_cam2ugv*[data.ugvStereo.Red.P_cam(i,:) 1]')';
        data.caldata.Blue.P_ugv(i,:) = (data.caldata.H_test_cam2ugv*[data.ugvStereo.Blue.P_cam(i,:) 1]')';
        data.caldata.Green.P_ugv(i,:) = (data.caldata.H_test_cam2ugv*[data.ugvStereo.Green.P_cam(i,:) 1]')'; 
    end

    data.caldata.Red.P_ugv(:,4) = [];
    data.caldata.Blue.P_ugv(:,4) = [];
    data.caldata.Green.P_ugv(:,4) = [];

%compute uav xyz
    data.caldata.uav.P_ugv = 0.5*(data.caldata.Red.P_ugv+data.caldata.Blue.P_ugv);
%compute uav yaw
    data.caldata.BmR = data.caldata.Blue.P_ugv - data.caldata.Red.P_ugv;
    data.caldata.GmB = data.caldata.Green.P_ugv - data.caldata.Blue.P_ugv;
    data.caldata.RmG = data.caldata.Red.P_ugv - data.caldata.Green.P_ugv;
    data.caldata.RmB = data.caldata.Red.P_ugv - data.caldata.Blue.P_ugv;

    for k = 1:length(data.caldata.RmB)
        znum = cross(data.caldata.GmB(k,:), data.caldata.RmB(k,:));
        normz = norm(znum);
        normy = norm(data.caldata.BmR(k,:));        
        data.caldata.z_axis(k,:) = znum / normz;
        data.caldata.y_axis(k,:) = data.caldata.BmR(k,:) / normy;
        data.caldata.x_axis(k,:) = cross(y_axis(k,:), z_axis(k,:));
        data.caldata.yaw_ugv(k,1) = atan2(x_axis(k,2), x_axis(k,1))*180/pi;
        clear znum normz normy
    end 



[dx, error.x] = fitDataY(data.ugvStereo.time, data.caldata.uav.P_ugv(:,1), data.vicon.uav.time, data.vicon.uav.P.global(:,1));
[dy, error.y] = fitDataY(data.ugvStereo.time, data.caldata.uav.P_ugv(:,2), data.vicon.uav.time, data.vicon.uav.P.global(:,2));
[dz, error.z] = fitDataY(data.ugvStereo.time, data.caldata.uav.P_ugv(:,3), data.vicon.uav.time, data.vicon.uav.P.global(:,3));

data.caldata.uav.error_ugv = [error.x error.y error.z];
data.caldata.uav.error_yaw = data.caldata.yaw_ugv - data.vicon.uav.splines.yaw.global.degrees_stereotime;



[data.caldata.Red.dx, data.caldata.Red.error.x] = fitDataY(data.ugvStereo.time, data.caldata.Red.P_ugv(:,1), data.vicon.uav.time, data.vicon.red_.P.global(:,1));
[data.caldata.Red.dy, data.caldata.Red.error.y] = fitDataY(data.ugvStereo.time, data.caldata.Red.P_ugv(:,2), data.vicon.uav.time, data.vicon.red_.P.global(:,2));
[data.caldata.Red.dz, data.caldata.Red.error.z] = fitDataY(data.ugvStereo.time, data.caldata.Red.P_ugv(:,3), data.vicon.uav.time, data.vicon.red_.P.global(:,3));

[data.caldata.Blue.dx, data.caldata.Blue.error.x] = fitDataY(data.ugvStereo.time, data.caldata.Blue.P_ugv(:,1), data.vicon.uav.time, data.vicon.blue_.P.global(:,1));
[data.caldata.Blue.dy, data.caldata.Blue.error.y] = fitDataY(data.ugvStereo.time, data.caldata.Blue.P_ugv(:,2), data.vicon.uav.time, data.vicon.blue_.P.global(:,2));
[data.caldata.Blue.dz, data.caldata.Blue.error.z] = fitDataY(data.ugvStereo.time, data.caldata.Blue.P_ugv(:,3), data.vicon.uav.time, data.vicon.blue_.P.global(:,3));

[data.caldata.Green.dx, data.caldata.Green.error.x] = fitDataY(data.ugvStereo.time, data.caldata.Green.P_ugv(:,1), data.vicon.uav.time, data.vicon.green_.P.global(:,1));
[data.caldata.Green.dy, data.caldata.Green.error.y] = fitDataY(data.ugvStereo.time, data.caldata.Green.P_ugv(:,2), data.vicon.uav.time, data.vicon.green_.P.global(:,2));
[data.caldata.Green.dz, data.caldata.Green.error.z] = fitDataY(data.ugvStereo.time, data.caldata.Green.P_ugv(:,3), data.vicon.uav.time, data.vicon.green_.P.global(:,3));

camOffset = [ dx dy dz]';




% figure(5001); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(5001); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
        try plot(data.vicon.uav.time, data.vicon.uav.P.global(:,1),'k.', 'displayname', 'uav x vicon'); catch; end
        try plot(data.ugvStereo.time, data.ugvStereo.uav.P_ugv(:,1), 'go', 'displayname', 'uav x stereo'); catch; end
        try plot(data.uavRecorder.est.time, data.uavRecorder.est.Position_gl(:,1), 'b.', 'displayname', 'uav x ckf est'); catch; end
        try plot(data.ugvStereo.time, data.caldata.uav.P_ugv(:,1)+dx, 'm*', 'displayname', 'uav x stereo'); catch; end
    hold off; grid on
    ylabel('X [m]'), xlabel('time [s]')
    title(['uav x position, global frame ' [meta.date meta.run]])
    legend('toggle');legend('Location', 'SouthEast')
    
% figure(5002); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(5002); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
        try plot(data.vicon.uav.time, data.vicon.uav.P.global(:,2),'k.', 'displayname', 'uav y vicon'); catch; end
        try plot(data.ugvStereo.time, data.ugvStereo.uav.P_ugv(:,2), 'go', 'displayname', 'uav y stereo'); catch; end
        try plot(data.uavRecorder.est.time, data.uavRecorder.est.Position_gl(:,2), 'b.', 'displayname', 'uav y ckf est'); catch; end
        try plot(data.ugvStereo.time, data.caldata.uav.P_ugv(:,2)+dy, 'm*', 'displayname', 'uav x stereo'); catch; end
    hold off; grid on
    title(['uav y position, global frame ' [meta.date meta.run]])
    xlabel('time [s]'); ylabel('Y [m]')
    legend('toggle');legend('Location', 'SouthEast')
    
% figure(5003); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(5003); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
        try plot(data.vicon.uav.time, data.vicon.uav.P.ugv(:,3),'k.', 'displayname', 'uav z vicon'); catch; end
        try plot(data.ugvStereo.time, data.ugvStereo.uav.P_ugv(:,3), 'go', 'displayname', 'uav z stereo'); catch; end
        try plot(data.uavRecorder.est.time, data.uavRecorder.est.Position_gl(:,3), 'b.', 'displayname', 'uav z ckf est'); catch; end
        try plot(data.ugvStereo.time, data.caldata.uav.P_ugv(:,3)+dz, 'm*', 'displayname', 'uav x stereo'); catch; end
    hold off; grid on
    title(['uav z position, global frame ' [meta.date meta.run]])
    xlabel('time [s]'); ylabel('z [m]')
    legend('toggle');legend('Location', 'SouthEast')
    
% figure(5004); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(5004); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
        try plot(data.ugvStereo.time, error.x, 'rx', 'displayname', 'uav x error'); catch; end
        try plot(data.ugvStereo.time, error.y, 'bx', 'displayname', 'uav y error'); catch; end
        try plot(data.ugvStereo.time, error.z, 'kx', 'displayname', 'uav z error'); catch; end
    hold off; grid on
    title(['uav z position, global frame ' [meta.date meta.run]])
    xlabel('time [s]'); ylabel('z [m]')
    legend('toggle');legend('Location', 'SouthEast')
    
% figure(5004); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
[data.caldata.Red.error.linxt, data.caldata.Red.error.linx] = linreg(data.ugvStereo.time, data.caldata.Red.error.x);
[data.caldata.Red.error.linyt, data.caldata.Red.error.liny] = linreg(data.ugvStereo.time, data.caldata.Red.error.y);
[data.caldata.Red.error.linzt, data.caldata.Red.error.linz] = linreg(data.ugvStereo.time, data.caldata.Red.error.z);
[data.caldata.Blue.error.linxt, data.caldata.Blue.error.linx] = linreg(data.ugvStereo.time, data.caldata.Blue.error.x);
[data.caldata.Blue.error.linyt, data.caldata.Blue.error.liny] = linreg(data.ugvStereo.time, data.caldata.Blue.error.y);
[data.caldata.Blue.error.linzt, data.caldata.Blue.error.linz] = linreg(data.ugvStereo.time, data.caldata.Blue.error.z);
[data.caldata.Green.error.linxt, data.caldata.Green.error.linx] = linreg(data.ugvStereo.time, data.caldata.Green.error.x);
[data.caldata.Green.error.linyt, data.caldata.Green.error.liny] = linreg(data.ugvStereo.time, data.caldata.Green.error.y);
[data.caldata.Green.error.linzt, data.caldata.Green.error.linz] = linreg(data.ugvStereo.time, data.caldata.Green.error.z);

figure(5000+j); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
        try plot(data.ugvStereo.time, data.caldata.Red.error.x, 'rx', 'displayname', 'red x error'); catch; end
        try plot(data.caldata.Red.error.linxt, data.caldata.Red.error.linx, 'k', 'LineWidth',4,'displayname', 'red x error lin'); catch; end
        try plot(data.ugvStereo.time, data.caldata.Red.error.y, 'rs', 'displayname', 'red y error'); catch; end
        try plot(data.caldata.Red.error.linyt, data.caldata.Red.error.liny, 'k', 'LineWidth',4,'displayname', 'red y error lin'); catch; end
        try plot(data.ugvStereo.time, data.caldata.Red.error.z, 'ro', 'displayname', 'red z error'); catch; end
        try plot(data.caldata.Red.error.linzt, data.caldata.Red.error.linz, 'k', 'LineWidth',4,'displayname', 'red z error lin'); catch; end
        try plot(data.ugvStereo.time, data.caldata.Blue.error.x, 'bx', 'displayname', 'blue x error'); catch; end
        try plot(data.caldata.Blue.error.linxt, data.caldata.Blue.error.linx, 'k', 'LineWidth',4,'displayname', 'blue x error lin'); catch; end
        try plot(data.ugvStereo.time, data.caldata.Blue.error.y, 'bs', 'displayname', 'blue y error'); catch; end
        try plot(data.caldata.Blue.error.linyt, data.caldata.Blue.error.liny, 'k', 'LineWidth',4,'displayname', 'blue y error lin'); catch; end
        try plot(data.ugvStereo.time, data.caldata.Blue.error.z, 'bo', 'displayname', 'blue z error'); catch; end
        try plot(data.caldata.Blue.error.linzt, data.caldata.Blue.error.linz, 'k', 'LineWidth',4,'displayname', 'blue z error lin'); catch; end
        try plot(data.ugvStereo.time, data.caldata.Green.error.x, 'gx', 'displayname', 'green x error'); catch; end
        try plot(data.caldata.Green.error.linxt, data.caldata.Green.error.linx, 'k', 'LineWidth',4,'displayname', 'green x error lin'); catch; end
        try plot(data.ugvStereo.time, data.caldata.Green.error.y, 'gs', 'displayname', 'green y error'); catch; end
        try plot(data.caldata.Green.error.linyt, data.caldata.Green.error.liny, 'k', 'LineWidth',4,'displayname', 'green y error lin'); catch; end
        try plot(data.ugvStereo.time, data.caldata.Green.error.z, 'go', 'displayname', 'green z error'); catch; end
        try plot(data.caldata.Green.error.linzt, data.caldata.Green.error.linz, 'k', 'LineWidth',4,'displayname', 'green z error lin'); catch; end
    hold off; 
    grid on
    title(['uav z position, global frame ' [meta.date meta.run]])
    xlabel('time [s]'); ylabel('z [m]')
%     legend('toggle');legend('Location', 'SouthEast')
    legend('off')
    axis([data.ugvStereo.time(1) data.ugvStereo.time(end) -0.5 0.5])
    try %current axis
        current_limits = axis; current_axes = gca; 
        current_axes.XLim = [(data.ugvStereo.time(1)) (data.uavRecorder.est.time(end))];
        current_axes.YLim = [-0.5 0.25];
        current_axes.YTick = [current_axes.YLim(1):0.125:current_axes.YLim(2)];
        clear current_axes current_limits
    end

end
    


%%

[data.caldata.Red.dx, data.caldata.Red.error.x] = fitDataY(data.ugvStereo.time, data.caldata.Red.P_ugv(:,1), data.vicon.uav.time, data.vicon.red_.P.global(:,1));
[data.caldata.Red.dy, data.caldata.Red.error.y] = fitDataY(data.ugvStereo.time, data.caldata.Red.P_ugv(:,2), data.vicon.uav.time, data.vicon.red_.P.global(:,2));
[data.caldata.Red.dz, data.caldata.Red.error.z] = fitDataY(data.ugvStereo.time, data.caldata.Red.P_ugv(:,3), data.vicon.uav.time, data.vicon.red_.P.global(:,3));







