
function data = gen_metrics(data)
%%
    try   
        timebool = (data.uavRecorder.est.time<(data.experiment.uav.FlyTime(end))) .* (data.uavRecorder.est.time>(data.experiment.uav.FlyTime(1)));
        timebool_stereo = (data.ugvRecorder.stereo.time<(data.experiment.uav.FlyTime(end))) .* (data.ugvRecorder.stereo.time>(data.experiment.uav.FlyTime(1)));
        
        data.metrics.uav.time  = data.uavRecorder.est.time .* timebool;data.metrics.uav.time(data.metrics.uav.time==0) = [];
        data.metrics.uav.x.err = data.vicon.uav.splines.P.global.x_diff .*timebool; data.metrics.uav.x.err(data.metrics.uav.x.err==0) = [];
        data.metrics.uav.y.err = data.vicon.uav.splines.P.global.y_diff .*timebool; data.metrics.uav.y.err(data.metrics.uav.y.err==0) = [];
        data.metrics.uav.z.err = data.vicon.uav.splines.P.global.z_diff .*timebool; data.metrics.uav.z.err(data.metrics.uav.z.err==0) = [];

        timebool_yaw = (data.uavRecorder.est.time<(data.experiment.uav.FlyTime(end))) .* (data.uavRecorder.est.time>(data.experiment.uav.FlyTime(1)));
        data.metrics.uav.yaw.time = data.uavRecorder.est.time .* timebool_yaw; data.metrics.uav.yaw.time(data.metrics.uav.yaw.time==0) = [];
        data.metrics.uav.yaw.err = data.vicon.est_yaw.spline_error .*timebool_yaw; data.metrics.uav.yaw.err(data.metrics.uav.yaw.err==0) = [];

        data.metrics.uav.yaw.time = data.metrics.uav.yaw.time - data.metrics.uav.yaw.time(1);
        data.metrics.uav.time = data.metrics.uav.time - data.metrics.uav.time(1);
        
    catch
        disp('gen_metrics try #1')
    end
    try

        data.metrics.uav.x.mean = mean(data.metrics.uav.x.err); 
        data.metrics.uav.y.mean = mean(data.metrics.uav.y.err); 
        data.metrics.uav.z.mean = mean(data.metrics.uav.z.err);
        data.metrics.uav.yaw.mean = mean(data.metrics.uav.yaw.err);
        disp('data.metrics.uav.[ x y z yaw].mean'); disp([data.metrics.uav.x.mean data.metrics.uav.y.mean data.metrics.uav.z.mean data.metrics.uav.yaw.mean])

        data.metrics.uav.x.max = max(abs(data.metrics.uav.x.err)); 
        data.metrics.uav.y.max = max(abs(data.metrics.uav.y.err)); 
        data.metrics.uav.z.max = max(abs(data.metrics.uav.z.err));
        data.metrics.uav.yaw.max = max(abs(data.metrics.uav.yaw.err));
        disp('data.metrics.uav.[ x y z yaw].max'); disp([data.metrics.uav.x.max data.metrics.uav.y.max data.metrics.uav.z.max data.metrics.uav.yaw.max])

        
        data.metrics.ugv.time = data.ugvRecorder.est.time;
        data.metrics.ugv.x.err = data.vicon.ugvk.splines.P.global.x_diff ; 
        data.metrics.ugv.y.err = data.vicon.ugvk.splines.P.global.y_diff ; 
        data.metrics.ugv.z.err = data.vicon.ugvk.splines.P.global.z_diff ;
        data.metrics.ugv.yaw.err = data.vicon.ugvk.splines.yaw.est_err;

        data.metrics.ugv.x.mean = mean(data.metrics.ugv.x.err); 
        data.metrics.ugv.y.mean = mean(data.metrics.ugv.y.err); 
        data.metrics.ugv.z.mean = mean(data.metrics.ugv.z.err);
        data.metrics.ugv.yaw.mean = mean(data.metrics.ugv.yaw.err);
        disp('data.metrics.ugv.[ x y z yaw].mean'); disp([data.metrics.ugv.x.mean data.metrics.ugv.y.mean data.metrics.ugv.z.mean data.metrics.ugv.yaw.mean])

        data.metrics.ugv.x.max = max(abs(data.metrics.ugv.x.err)); 
        data.metrics.ugv.y.max = max(abs(data.metrics.ugv.y.err)); 
        data.metrics.ugv.z.max = max(abs(data.metrics.ugv.z.err));
        data.metrics.ugv.yaw.max = max(abs(data.metrics.ugv.yaw.err));
        disp('data.metrics.ugv.[ x y z yaw].max'); disp([data.metrics.ugv.x.max data.metrics.ugv.y.max data.metrics.ugv.z.max data.metrics.ugv.yaw.max])

    catch
        disp('gen_metrics try #2')
    end
    try
        
        t_low = 10.2;
        t_high = 60;
        data.metrics.uav_only.time = data.uavRecorder.est.time .* (data.uavRecorder.est.time>t_low) .* (data.uavRecorder.est.time<t_high); data.metrics.uav_only.time(data.metrics.uav_only.time==0) = [];
        data.metrics.uav_only.x.err = data.vicon.uav.splines.P.global.x_diff .* (data.uavRecorder.est.time>t_low) .* (data.uavRecorder.est.time<t_high); data.metrics.uav_only.x.err(data.metrics.uav_only.x.err==0) = [];
        data.metrics.uav_only.y.err = data.vicon.uav.splines.P.global.y_diff .* (data.uavRecorder.est.time>t_low) .* (data.uavRecorder.est.time<t_high); data.metrics.uav_only.y.err(data.metrics.uav_only.y.err==0) = [];
        data.metrics.uav_only.z.err = data.vicon.uav.splines.P.global.z_diff .* (data.uavRecorder.est.time>t_low) .* (data.uavRecorder.est.time<t_high); data.metrics.uav_only.z.err(data.metrics.uav_only.z.err==0) = [];
        
        try data = splineYawFig502(data); catch; end
        data.metrics.uav_only.yaw.err = data.vicon.uav.yaw.global.degrees_error .* (data.uavRecorder.est.time>t_low) .* (data.uavRecorder.est.time<t_high); data.metrics.uav_only.yaw.err(data.metrics.uav_only.yaw.err==0) = [];

        data.metrics.uav_only.x.err_mean = mean(data.metrics.uav_only.x.err);
        data.metrics.uav_only.y.err_mean = mean(data.metrics.uav_only.y.err);
        data.metrics.uav_only.z.err_mean = mean(data.metrics.uav_only.z.err);
        data.metrics.uav_only.yaw.err_mean = mean(data.metrics.uav_only.yaw.err);

        data.metrics.uav_only.x.err_max = signmax(data.metrics.uav_only.x.err);
        data.metrics.uav_only.y.err_max = signmax(data.metrics.uav_only.y.err);
        data.metrics.uav_only.z.err_max = signmax(data.metrics.uav_only.z.err);
        data.metrics.uav_only.yaw.err_max = signmax(data.metrics.uav_only.yaw.err);

        
        disp(['UAV estimated x error: [mean, max]  [' num2str(data.metrics.uav_only.x.err_mean) ', ' num2str(data.metrics.uav_only.x.err_max) ']' ])
        disp(['UAV estimated y error: [mean, max]  [' num2str(data.metrics.uav_only.y.err_mean) ', ' num2str(data.metrics.uav_only.y.err_max) ']' ])
        disp(['UAV estimated z error: [mean, max]  [' num2str(data.metrics.uav_only.z.err_mean) ', ' num2str(data.metrics.uav_only.z.err_max) ']' ])
        disp(['UAV estimated yaw error: [mean, max]  [' num2str(data.metrics.uav_only.yaw.err_mean) ', ' num2str(data.metrics.uav_only.yaw.err_max) ']' ])
    catch
        disp('gen_metrics try #3')
    end
    try
        spline = csapi(data.vicon.ugvk.time,data.vicon.ugvk.P.global(:,1));
            vector_newtime = fnval(spline, data.ugvRecorder.est.time);
                data.vicon.ugvk.atEstTime.P.global(:,1) = vector_newtime;
        spline = csapi(data.vicon.ugvk.time,data.vicon.ugvk.P.global(:,2));
            vector_newtime = fnval(spline, data.ugvRecorder.est.time);
                data.vicon.ugvk.atEstTime.P.global(:,2) = vector_newtime;
        spline = csapi(data.vicon.ugvk.time,data.vicon.ugvk.P.global(:,3));
            vector_newtime = fnval(spline, data.ugvRecorder.est.time);
                data.vicon.ugvk.atEstTime.P.global(:,3) = vector_newtime;

        data.metrics.ugv_plan.error = data.ugvRecorder.est.Position_gl - data.vicon.ugvk.atEstTime.P.global;
                
        spline = csapi(data.vicon.ugvk.time,data.vicon.ugvk.yaw.global);
            vector_newtime = fnval(spline, data.ugvRecorder.est.time);
                data.vicon.ugvk.atEstTime.yaw.global = vector_newtime;
        
        data.metrics.ugv_plan.yaw_error = data.ugvRecorder.est.Yaw - 180*data.vicon.ugvk.atEstTime.yaw.global/pi;
        
    catch
        disp('gen_metrics try #4')
    end
        
    % time UGV is moving 20180223/006: 31 to 72.5
%         t_low = 31;
%         t_high = 72.5;
        
%         t_low = data.kobuki_logger.cmd_vel.time(1);
%         t_high = data.kobuki_logger.cmd_vel.time(end);

    try
        t_low = data.kobuki_logger.cmd_vel.time(1);
        t_high = data.kobuki_logger.cmd_vel.time(end);

  
        data.metrics.ugv_plan.time  = data.ugvRecorder.est.time   .* (data.ugvRecorder.est.time>t_low) .* (data.ugvRecorder.est.time<t_high); data.metrics.ugv_plan.time(data.metrics.ugv_plan.time==0) = [];
        data.metrics.ugv_plan.x.err = data.metrics.ugv_plan.error(:,1) .* (data.ugvRecorder.est.time>t_low) .* (data.ugvRecorder.est.time<t_high); data.metrics.ugv_plan.x.err(data.metrics.ugv_plan.x.err==0) = [];
        data.metrics.ugv_plan.y.err = data.metrics.ugv_plan.error(:,2) .* (data.ugvRecorder.est.time>t_low) .* (data.ugvRecorder.est.time<t_high); data.metrics.ugv_plan.y.err(data.metrics.ugv_plan.y.err==0) = [];
        data.metrics.ugv_plan.z.err = data.metrics.ugv_plan.error(:,3) .* (data.ugvRecorder.est.time>t_low) .* (data.ugvRecorder.est.time<t_high); data.metrics.ugv_plan.z.err(data.metrics.ugv_plan.z.err==0) = [];
        data.metrics.ugv_plan.yaw.err = data.metrics.ugv_plan.yaw_error .* (data.ugvRecorder.est.time>t_low) .* (data.ugvRecorder.est.time<t_high); data.metrics.ugv_plan.yaw.err(data.metrics.ugv_plan.yaw.err==0) = [];
        
        data.metrics.ugv_plan.x.err_mean = mean(data.metrics.ugv_plan.x.err); data.metrics.ugv_plan.x.err_max = signmax(data.metrics.ugv_plan.x.err);
        data.metrics.ugv_plan.y.err_mean = mean(data.metrics.ugv_plan.y.err); data.metrics.ugv_plan.y.err_max = signmax(data.metrics.ugv_plan.y.err);
        data.metrics.ugv_plan.z.err_mean = mean(data.metrics.ugv_plan.z.err); data.metrics.ugv_plan.z.err_max = signmax(data.metrics.ugv_plan.z.err);
        data.metrics.ugv_plan.yaw.err_mean = mean(data.metrics.ugv_plan.yaw.err); data.metrics.ugv_plan.yaw.err_max = signmax(data.metrics.ugv_plan.yaw.err);

        disp(['UGV estimated x error: [mean, max]  [' num2str(data.metrics.ugv_plan.x.err_mean) ', ' num2str(data.metrics.ugv_plan.x.err_max) ']' ])
        disp(['UGV estimated y error: [mean, max]  [' num2str(data.metrics.ugv_plan.y.err_mean) ', ' num2str(data.metrics.ugv_plan.y.err_max) ']' ])
        disp(['UGV estimated z error: [mean, max]  [' num2str(data.metrics.ugv_plan.z.err_mean) ', ' num2str(data.metrics.ugv_plan.z.err_max) ']' ])
        disp(['UGV estimated yaw error: [mean, max]  [' num2str(data.metrics.ugv_plan.yaw.err_mean) ', ' num2str(data.metrics.ugv_plan.yaw.err_max) ']' ])
    catch
        disp('gen_metrics try #5')
    end
    
    
%     generate error data for both vehicles while the ugv is moving during an action experiment
    try
        try
            data.metrics.kobuki.tend = data.kobuki_logger.local_plan.time(end);
            data.metrics.kobuki.tstart = data.kobuki_logger.local_plan.time(1);
        catch
            data.metrics.kobuki.tend = data.kobuki_logger.cmd_vel.time(end);
            data.metrics.kobuki.tstart = data.kobuki_logger.cmd_vel.time(1);
        end
        
        
        % UGV data
        
        data.metrics.kobuki.time = data.vicon.ugvk.time; 
            data.metrics.kobuki.time(data.vicon.ugvk.time>data.metrics.kobuki.tend) = [];
            data.metrics.kobuki.time(data.vicon.ugvk.time<data.metrics.kobuki.tstart) = [];
            
        data.metrics.kobuki.vPx = data.vicon.ugvk.P.global(:,1);
            data.metrics.kobuki.vPx(data.vicon.ugvk.time>data.metrics.kobuki.tend) = [];
            data.metrics.kobuki.vPx(data.vicon.ugvk.time<data.metrics.kobuki.tstart) = [];

            data.metrics.kobuki.Px_vicon = data.vicon.ugvk.P.vicon(:,1);
                data.metrics.kobuki.Px_vicon(data.vicon.ugvk.time>data.metrics.kobuki.tend) = [];
                data.metrics.kobuki.Px_vicon(data.vicon.ugvk.time<data.metrics.kobuki.tstart) = [];

        data.metrics.kobuki.vPy = data.vicon.ugvk.P.global(:,2);
            data.metrics.kobuki.vPy(data.vicon.ugvk.time>data.metrics.kobuki.tend) = [];
            data.metrics.kobuki.vPy(data.vicon.ugvk.time<data.metrics.kobuki.tstart) = [];

            data.metrics.kobuki.Py_vicon = data.vicon.ugvk.P.vicon(:,2);
                data.metrics.kobuki.Py_vicon(data.vicon.ugvk.time>data.metrics.kobuki.tend) = [];
                data.metrics.kobuki.Py_vicon(data.vicon.ugvk.time<data.metrics.kobuki.tstart) = [];

        data.metrics.kobuki.ePx = data.ugvRecorder.est.Position_gl(:,1); 
            data.metrics.kobuki.ePx(data.ugvRecorder.est.time>data.metrics.kobuki.tend) = [];
            data.metrics.kobuki.ePx(data.ugvRecorder.est.time<data.metrics.kobuki.tstart) = [];
        
        data.metrics.kobuki.ePy = data.ugvRecorder.est.Position_gl(:,2);
            data.metrics.kobuki.ePy(data.ugvRecorder.est.time>data.metrics.kobuki.tend) = [];
            data.metrics.kobuki.ePy(data.ugvRecorder.est.time<data.metrics.kobuki.tstart) = [];

        data.metrics.kobuki.errtime = data.ugvRecorder.est.time; 
            data.metrics.kobuki.errtime(data.metrics.kobuki.errtime>data.metrics.kobuki.tend) = [];
            data.metrics.kobuki.errtime(data.metrics.kobuki.errtime<data.metrics.kobuki.tstart) = [];

        data.metrics.kobuki.odom = getDisplacement(data.vicon.ugvk.atEstTime.P.global(:,1:2)');
            data.metrics.kobuki.odom(data.ugvRecorder.est.time>data.metrics.kobuki.tend) = [];
            data.metrics.kobuki.odom(data.ugvRecorder.est.time<data.metrics.kobuki.tstart) = [];

        data.metrics.kobuki.errx = data.metrics.ugv_plan.error(:,1); 
            data.metrics.kobuki.errx(data.ugvRecorder.est.time>data.metrics.kobuki.tend) = [];
            data.metrics.kobuki.errx(data.ugvRecorder.est.time<data.metrics.kobuki.tstart) = [];

        data.metrics.kobuki.erry = data.metrics.ugv_plan.error(:,2); 
            data.metrics.kobuki.erry(data.ugvRecorder.est.time>data.metrics.kobuki.tend) = [];
            data.metrics.kobuki.erry(data.ugvRecorder.est.time<data.metrics.kobuki.tstart) = [];

        data.metrics.kobuki.erryaw = data.metrics.ugv_plan.yaw_error; 
            data.metrics.kobuki.erryaw(data.ugvRecorder.est.time>data.metrics.kobuki.tend) = [];
            data.metrics.kobuki.erryaw(data.ugvRecorder.est.time<data.metrics.kobuki.tstart) = [];

        % UAV data
        data.metrics.uav_during_path.time = data.uavRecorder.est.time;
            data.metrics.uav_during_path.time(data.uavRecorder.est.time>data.metrics.kobuki.tend) = [];
            data.metrics.uav_during_path.time(data.uavRecorder.est.time<data.metrics.kobuki.tstart) = [];
            
        data.metrics.uav_during_path.errx = data.vicon.uav.splines.P.global.x_diff;
            data.metrics.uav_during_path.errx(data.uavRecorder.est.time>data.metrics.kobuki.tend) = [];
            data.metrics.uav_during_path.errx(data.uavRecorder.est.time<data.metrics.kobuki.tstart) = [];

        data.metrics.uav_during_path.erry = data.vicon.uav.splines.P.global.y_diff;
            data.metrics.uav_during_path.erry(data.uavRecorder.est.time>data.metrics.kobuki.tend) = [];
            data.metrics.uav_during_path.erry(data.uavRecorder.est.time<data.metrics.kobuki.tstart) = [];

        data.metrics.uav_during_path.errz = data.vicon.uav.splines.P.global.z_diff;
            data.metrics.uav_during_path.errz(data.uavRecorder.est.time>data.metrics.kobuki.tend) = [];
            data.metrics.uav_during_path.errz(data.uavRecorder.est.time<data.metrics.kobuki.tstart) = [];

        data.metrics.uav_during_path.erryaw = data.vicon.est_yaw.spline_error;
            data.metrics.uav_during_path.erryaw(data.uavRecorder.est.time>data.metrics.kobuki.tend) = [];
            data.metrics.uav_during_path.erryaw(data.uavRecorder.est.time<data.metrics.kobuki.tstart) = [];
            
    catch
        disp('gen_metrics try e')
    end
    
    try
        % goal location : 
        data.metrics.ugv_plan.goal_final = data.kobuki_logger.global_plan.path.xyz{1}(end,1:2);
        data.metrics.ugv_plan.final_actual = data.vicon.ugvk.P.global(end,1:2);
        data.metrics.ugv_plan.final_estimated = data.ugvRecorder.est.Position_gl(end,1:2);
        
        data.metrics.ugv_plan.final_actual_error = data.metrics.ugv_plan.final_actual - data.metrics.ugv_plan.goal_final;
        data.metrics.ugv_plan.final_estimated_error = data.metrics.ugv_plan.final_estimated - data.metrics.ugv_plan.goal_final;
    catch
        disp('gen_metrics try #6')
    end

end


function sump = getDisplacement(A)
% getDisplacement(data.vicon.ugvk.atEstTime.P.global')
    p = vecnorm(A)';
    dp(1) = 0;
    sump(1) = 0;
    for i = 1:(length(p)-1)
        dp(i+1,1) = abs(p(i+1)-p(i));
        sump(i+1,1) = sump(i,1) + dp(i+1,1);
    end
end