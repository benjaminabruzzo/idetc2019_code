%%
clc

vicon_abs = sqrt(data.vicon.ugvk.P.global(end,1:3) * data.vicon.ugvk.P.global(end,1:3)')


H0 = data.caldata.left.H.H1;
Hend = data.caldata.left.H.(matlab.lang.makeValidName(['H' num2str(length(fields(data.caldata.left.H)))]));
H0inv = invertH(H0);
Hendinv = invertH(Hend);


z = [0 0 0 1]';
zp = Hendinv*z;
zpp = H0 * zp;

z_abs = sqrt(zpp(1:3)' * zpp(1:3))

p = zpp(1:3) / norm(zpp(1:3))

phi = tan(p(2)/p(3))

phi_deg = phi*180/pi


H0 = data.caldata.right.H.H1;
Hend = data.caldata.right.H.(matlab.lang.makeValidName(['H' num2str(length(fields(data.caldata.right.H)))]));
H0inv = invertH(H0);
Hendinv = invertH(Hend);


z = [0 0 0 1]';
zp = Hendinv*z;
zpp = H0 * zp;

z_abs = sqrt(zpp(1:3)' * zpp(1:3))

p = zpp(1:3) / norm(zpp(1:3))

phi = tan(p(2)/p(3))

phi_deg = phi*180/pi

%% left tilt angle
H0 = data.caldata.left.H.H1;
H0inv = invertH(H0);
z = [0 0 0 1]';

vicon_abs = sqrt(data.vicon.ugvk.P.global(end,1:3) * data.vicon.ugvk.P.global(end,1:3)')

clear p zp zpp 
for i = 1:length(fields(data.caldata.left.H))
    Hi = data.caldata.left.H.(matlab.lang.makeValidName(['H' num2str(i)])); 
    Hiinv = invertH(Hi);
    zp = Hiinv*z;
    zpp = H0 * zp;
    data.caldata.left.z_abs(i,1) = sqrt(zpp(1:3)' * zpp(1:3));
    data.caldata.left.p(i,:) = (zpp(1:3)' / norm(zpp(1:3)));
    data.caldata.left.phi(i,1) = tan(data.caldata.left.p(i,2)/data.caldata.left.p(i,3));
    data.caldata.left.phi_deg(i,1) = data.caldata.left.phi(i)*180/pi;
end

% right tilt angle
H0 = data.caldata.right.H.H1;
H0inv = invertH(H0);
z = [0 0 0 1]';

clear p zp zpp 
for i = 1:length(fields(data.caldata.right.H))
    Hi = data.caldata.right.H.(matlab.lang.makeValidName(['H' num2str(i)])); 
    Hiinv = invertH(Hi);
    zp = Hiinv*z;
    zpp = H0 * zp;
    data.caldata.right.z_abs(i,1) = sqrt(zpp(1:3)' * zpp(1:3));
    data.caldata.right.p(i,:) = (zpp(1:3)' / norm(zpp(1:3)));
    data.caldata.right.phi(i,1) = tan(data.caldata.right.p(i,2)/data.caldata.right.p(i,3));
    data.caldata.right.phi_deg(i,1) = data.caldata.right.phi(i)*180/pi;
end

disp(['left/right angle : [' num2str(data.caldata.left.phi_deg(end)) '] / [ ' num2str(data.caldata.right.phi_deg(end)) ' ] '])

% 20180209 001: left/right angle : [14.5407] / [ 20.7323 ] 32 images each (left/right), total ugv displacement: 2.037 m
% 20180209 002: left/right angle : [11.4769] / [ 24.492 ] 40 images each (left/right), total ugv displacement: 2.5273 m
% 20180209 003: left/right angle : [21.5285] / [ 21.99 ] 33 images each (left/right), total ugv displacement: 2.0359 m
% 20180209 004: left/right angle : [16.2818] / [ 20.2022 ] 14 images each (left/right), total ugv displacement: 0.7692 m
%% left to right transform


for i = 1:length(fields(data.caldata.right.H))
    HLeft = data.caldata.left.H.(matlab.lang.makeValidName(['H' num2str(i)])); 
    HLeftinv = invertH(HLeft);

    HRight = data.caldata.right.H.(matlab.lang.makeValidName(['H' num2str(i)])); 
    HRightinv = invertH(HRight);
    z = [0 0 0 1]';


    zp = HLeftinv * z;
    zpp(i,:) = (HRight * zp)';

end


%% Compute ugv and gator baord transforms at the time of there stereo raw images
% Spline UGV data
[data.rectdata.ugvk.q_x.spline, data.rectdata.ugvk.q_x.atRawtime]  = splineAndInterp(data.vicon.ugvk.time, data.vicon.ugvk.q_i(:,1), data.ugvStereo.leftraw.time);
[data.rectdata.ugvk.q_y.spline, data.rectdata.ugvk.q_y.atRawtime]  = splineAndInterp(data.vicon.ugvk.time, data.vicon.ugvk.q_i(:,2), data.ugvStereo.leftraw.time);
[data.rectdata.ugvk.q_z.spline, data.rectdata.ugvk.q_z.atRawtime]  = splineAndInterp(data.vicon.ugvk.time, data.vicon.ugvk.q_i(:,3), data.ugvStereo.leftraw.time);
[data.rectdata.ugvk.q_w.spline, data.rectdata.ugvk.q_w.atRawtime]  = splineAndInterp(data.vicon.ugvk.time, data.vicon.ugvk.q_i(:,4), data.ugvStereo.leftraw.time);
 data.rectdata.ugvk.q.atRawtime = [data.rectdata.ugvk.q_x.atRawtime data.rectdata.ugvk.q_y.atRawtime data.rectdata.ugvk.q_z.atRawtime data.rectdata.ugvk.q_w.atRawtime];

[data.rectdata.ugvk.P_x.spline, data.rectdata.ugvk.P_x.atRawtime]  = splineAndInterp(data.vicon.ugvk.time, data.vicon.ugvk.P.vicon(:,1), data.ugvStereo.leftraw.time);
[data.rectdata.ugvk.P_y.spline, data.rectdata.ugvk.P_y.atRawtime]  = splineAndInterp(data.vicon.ugvk.time, data.vicon.ugvk.P.vicon(:,2), data.ugvStereo.leftraw.time);
[data.rectdata.ugvk.P_z.spline, data.rectdata.ugvk.P_z.atRawtime]  = splineAndInterp(data.vicon.ugvk.time, data.vicon.ugvk.P.vicon(:,3), data.ugvStereo.leftraw.time);
 data.rectdata.ugvk.P.atRawtime = [data.rectdata.ugvk.P_x.atRawtime data.rectdata.ugvk.P_y.atRawtime data.rectdata.ugvk.P_z.atRawtime];

% Spline Gator data
[data.rectdata.gator_.q_x.spline, data.rectdata.gator_.q_x.atRawtime]  = splineAndInterp(data.vicon.gator_.time, data.vicon.gator_.q_i(:,1), data.ugvStereo.leftraw.time);
[data.rectdata.gator_.q_y.spline, data.rectdata.gator_.q_y.atRawtime]  = splineAndInterp(data.vicon.gator_.time, data.vicon.gator_.q_i(:,2), data.ugvStereo.leftraw.time);
[data.rectdata.gator_.q_z.spline, data.rectdata.gator_.q_z.atRawtime]  = splineAndInterp(data.vicon.gator_.time, data.vicon.gator_.q_i(:,3), data.ugvStereo.leftraw.time);
[data.rectdata.gator_.q_w.spline, data.rectdata.gator_.q_w.atRawtime]  = splineAndInterp(data.vicon.gator_.time, data.vicon.gator_.q_i(:,4), data.ugvStereo.leftraw.time);
 data.rectdata.gator_.q.atRawtime = [data.rectdata.gator_.q_x.atRawtime data.rectdata.gator_.q_y.atRawtime data.rectdata.gator_.q_z.atRawtime data.rectdata.gator_.q_w.atRawtime];
 
[data.rectdata.gator_.P_x.spline, data.rectdata.gator_.P_x.atRawtime]  = splineAndInterp(data.vicon.gator_.time, data.vicon.gator_.P.vicon(:,1), data.ugvStereo.leftraw.time);
[data.rectdata.gator_.P_y.spline, data.rectdata.gator_.P_y.atRawtime]  = splineAndInterp(data.vicon.gator_.time, data.vicon.gator_.P.vicon(:,2), data.ugvStereo.leftraw.time);
[data.rectdata.gator_.P_z.spline, data.rectdata.gator_.P_z.atRawtime]  = splineAndInterp(data.vicon.gator_.time, data.vicon.gator_.P.vicon(:,3), data.ugvStereo.leftraw.time);
 data.rectdata.gator_.P.atRawtime = [data.rectdata.gator_.P_x.atRawtime data.rectdata.gator_.P_y.atRawtime data.rectdata.gator_.P_z.atRawtime];


for i = 1:length(data.ugvStereo.leftraw.time)
    
    ugv_q = data.rectdata.ugvk.q.atRawtime(i,:); data.rectdata.ugvk.dcm(:,:,i) = QwedgeDCM(ugv_q);
    data.rectdata.ugvk.H_vic2lo(:,:,i) = wrapH(data.rectdata.ugvk.dcm(:,:,i), data.rectdata.ugvk.P.atRawtime(i,:)');
    
    gator_q = data.rectdata.gator_.q.atRawtime(i,:);data.rectdata.gator_.dcm(:,:,i) = QwedgeDCM(gator_q);
    data.rectdata.gator_.H_vic2lo(:,:,i) = wrapH(data.rectdata.gator_.dcm(:,:,i), data.rectdata.gator_.P.atRawtime(i,:)');
    
    Hgator2ugv(:,:,i) = data.rectdata.ugvk.H_vic2lo(:,:,i) * invertH(data.rectdata.gator_.H_vic2lo(:,:,i));
    Hchecker2ugv(:,:,i) = Hgator2ugv(:,:,i) * Ryh(pi);

    Rl = data.rectdata.left.R.R53;
    tl = data.rectdata.left.t.t53;
    Rr = data.rectdata.right.R.R53;
    tr = data.rectdata.right.t.t53;

    HLcam2ugv(:,:,i) = Hchecker2ugv(:,:,i) * wrapH(Rl',tl);
    HRcam2ugv(:,:,i) = Hchecker2ugv(:,:,i) * wrapH(Rr',tr);


    Hcam2ugv(:,:,i) = 0.5* (HLcam2ugv(:,:,i)+HRcam2ugv(:,:,i));

end

%% compute the camera to gator board transform
Hgator2ugv = data.rectdata.ugvk.H_vic2lo(:,:,i) * invertH(data.rectdata.gator_.H_vic2lo(:,:,i))
Hchecker2ugv = Hgator2ugv * Ryh(pi)

Rl = data.rectdata.left.R.R53;
tl = data.rectdata.left.t.t53;
Rr = data.rectdata.right.R.R53;
tr = data.rectdata.right.t.t53;

HLcam2ugv = Hchecker2ugv * wrapH(Rl',tl)
HRcam2ugv = Hchecker2ugv * wrapH(Rr',tr)


Hcam2ugv = 0.5* (HLcam2ugv+HRcam2ugv)


%%

Hcam2gator = invertH(data.rectdata.left.H.H1)
Hgator2vicon = invertH(data.rectdata.gator_.H_vic2lo(:,:,1))
Hcam2vicon = Hgator2vicon * Hcam2gator
Hcam2ugv = data.rectdata.ugvk.H_vic2lo(:,:,1) * Hcam2vicon



Hgator2ugv = data.rectdata.ugvk.H_vic2lo(:,:,1) * invertH(data.rectdata.gator_.H_vic2lo(:,:,1))

Hcam2ugv = data.rectdata.ugvk.H_vic2lo(:,:,1) * invertH(data.rectdata.gator_.H_vic2lo(:,:,1)) * data.rectdata.left.H.H1







