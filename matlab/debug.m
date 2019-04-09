            P_red = vicon.red_.P.vicon - vicon.ugvk.P.vicon;
            P_blue = vicon.blue_.P.vicon - vicon.ugvk.P.vicon;
            P_green = vicon.green_.P.vicon - vicon.ugvk.P.vicon;
            
            %%
            for i = 1:size(P_red,1)
                vicon.red_.P.ugv(i,:) = vicon.ugvk.R_vic2lo(:,:,i) * P_red(i,:)';
                vicon.blue_.P.ugv(i,:) = vicon.ugvk.R_vic2lo(:,:,i) * P_blue(i,:)';
                vicon.green_.P.ugv(i,:) = vicon.ugvk.R_vic2lo(:,:,i) * P_green(i,:)';
            end
            
            %%
            vicon.uav.P.ugv = 0.5*(vicon.red_.P.ugv + vicon.blue_.P.ugv);
    %         disp(['     Setting global origin for uav'])
            vicon.uav.P.global = 0.5*(vicon.red_.P.global + vicon.blue_.P.global);
            vicon.uav.yaw.global.radians = vicon.uav.yaw.radians - vicon.global.orientation;
            vicon.uav.yaw.global.degrees = vicon.uav.yaw.global.radians * 180/pi;