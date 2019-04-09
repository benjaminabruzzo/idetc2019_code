clc

i = 1;

fp_cam = [0 0 0 1]';% this is the focal point of the camera, in the camera frame

% to transform the camera frame to the gator board frame, use the calibration data from python:

H_gator2cam = eval(['data.caldata.left.H.H' num2str(i)]);
H_cam2gator = invertH(H_gator2cam)
H_vicon2gator = data.vicon.gatorboard_.atRawTime.H_i(:,:,i);
H_gator2vicon = invertH(H_vicon2gator)
H_vicon2ugvk = data.vicon.ugvk.atRawTime.H_i(:,:,i)

H_vicon2cam = H_vicon2gator * Rzh(pi) * H_gator2cam
H_cam2vicon = invertH(H_vicon2cam)

H_cam2ugv = H_vicon2ugvk * H_gator2vicon  * H_cam2gator* Rzh(pi)

fp_gator = H_cam2gator*fp_cam % camera to gatorboard transform

% to transform the camera origin to the vicon frame, use the vicon data of the gator board:

fp_vicon = H_gator2vicon*fp_gator% camera to gatorboard transform


fp_ugv = H_vicon2ugvk * fp_vicon


%%
H_cam2ugv = [...
   0.010449533005127  -0.350207858280466   0.936499704751460   (0.063186564632798+0.1);...
   0.999900273477287   0.001905671201873  -0.010348963176804   (0.041383462154539-0.1);...
   0.001762325671160   0.936562791728111   0.350171246189147   (0.234807488045910+0.1);...
                   0                   0                   0   1.000000000000000];           

for i = 1:length(data.ugvStereo.Red.P_cam)
    test.Red.P_ugv(i,:) = (H_cam2ugv * [data.ugvStereo.Red.P_cam(i,:) 1]')';
end

% H_cam2ugv = data.vicon.ugvk.atRawTime.H_stereoCenter
  
     
     
%%
clc

% [mean(vicon.ugvk.atRawTime.H_Lcam2ugv,3) mean(vicon.ugvk.atRawTime.H_Rcam2ugv,3)]
% [std(vicon.ugvk.atRawTime.H_Lcam2ugv,0,3) std(vicon.ugvk.atRawTime.H_Rcam2ugv,0,3)]


H_Lcam2ugv = data.vicon.ugvk.atRawTime.H_Lcam2ugv;

H_Lcam2ugv_re = reshape(H_Lcam2ugv, size(H_Lcam2ugv,1)*size(H_Lcam2ugv,2),size(H_Lcam2ugv,3),1)


%%

Right.Camera.Inverse = inv(data.caldata.rectify.RightR);
M = Right.Camera.Inverse*data.caldata.rectify.RightP;
Baseline = abs(M(1,4))
Cx = data.caldata.rectify.RightP(1,3)
Cy = data.caldata.rectify.RightP(2,3)
FocalLength = data.caldata.rectify.RightP(1,1)




%%
n=1
SimpleCellsRk{n} = 'varname'

simplestring = [SimpleCellsRk{n} '_re = reshape(' SimpleCellsRk{n} ', size(' SimpleCellsRk{n} ',1)*size(' SimpleCellsRk{n} ',2),size(' SimpleCellsRk{n} ',3),1);']
%%
clc
Z = zeros(4);

I4 = eye(4);

nI4 = -eye(4);

P = [0.9*I4 2*I4 3*I4 0.8*I4; 0.7*I4 4*I4 5*I4 6*I4; 0.6*I4 7*I4 8*I4 9*I4; 0.5*I4 0.4*I4 0.3*I4 0.2*I4]


Rkj = 0.01*I4;

Rk = [Rkj Z; Z 0.01* Rkj]

H = [I4 nI4 Z Z; I4 Z Z nI4]

HP = H*P

HPHt = HP*H'

Sk = HPHt + Rk

PHt = P*H'

PHtSk = PHt*inv(Sk)

size(PHtSk)

%%
[...
    FIDs.fid1.p.est.global(:,:) ...
    FIDs.fid2.p.est.global(:,:) ...
    FIDs.fid3.p.est.global(:,:) ...
    FIDs.fid4.p.est.global(:,:) ...
%     FIDs.fid5.p.est.global(:,:) ...
%     FIDs.fid6.p.est.global(:,:) ...
%     FIDs.fid7.p.est.global(:,:) ...
%     FIDs.fid8.p.est.global(:,:) ...
    ]%...
    

%%
[...
    record.fid1.p.est.global ...
    record.fid2.p.est.global ...
    record.fid3.p.est.global ...
    record.fid4.p.est.global ...
    ]%...

%%
[...
    record.fid5.p.est.global ...
    record.fid6.p.est.global ...
    record.fid7.p.est.global ...
    record.fid8.p.est.global ...
    ]%...
