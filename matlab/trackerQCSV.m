% function [trackerData] = LoadTrackerCSV(meta)
% %% New function to load Vicon data from tracker
% 
%     csvlist = {...
%         'april02_'; ...
%         'april03_'; ...
%         'red_'; ...
%         'blue_'; ...
%         'green_'; ...
%         'create_'; ...
%         };
%     
% %     for csv_i = 1: length(csvlist)
% 
%     csv_i = 1;
% %           '/Users/benjamin/ros/data/20170911/039'
%     filename = ['/Users/benjamin/ros/data/'  meta.date 'csv/' csvlist{csv_i} meta.run '.csv'];
%     [trackerData] = trackerCSV(filename);
% 
% 
% end

function [out] = trackerQCSV(filename)
    out.csv=csvread(filename, 2,0); % the first line is a header
    out.time = (out.csv(:,1) - out.csv(1,1))/100; %tracker is 100fps
    out.Position = out.csv(:,[6,7,8])/1000;
    out.P.vicon = out.csv(:,[6,7,8])/1000;
    out.meanPosition = mean(out.Position);
    out.P.x = out.Position(:,1);
    out.P.y = out.Position(:,2);
    out.P.z = out.Position(:,3);

    % this is the rotation of the object back to the origin
    out.q_i = out.csv(:,[2,3,4,5]); 
    
    x = out.q_i(:,1);
    y = out.q_i(:,2);
    z = out.q_i(:,3);
    w = out.q_i(:,4);
    
    out.yaw.radians = -atan2(2*(x.*y - z.*w ) , 1 - 2*(y.^2 + z.^2)); %atan2(2,1);
    out.yaw.degrees = out.yaw.radians * 180 / pi;
    out.yaw.std.rad = std(out.yaw.radians);
    out.yaw.mean.rad = mean(out.yaw.radians);
    
    
    for i = 1:length(out.q_i)
%     [out.H, out.H_i, out.yaw.radians, out.yaw.degrees, out.R_vic2lo] = AngleAxis2Homogenous(out.csv);
        RH = Rz(out.yaw.radians(i));
        out.R_vic2lo(:,:,i) = RH;
        out.H_i(:,:,i) = [RH [RH*(-out.Position(i,:))']; [0 0 0 1]];
        out.x_axis(i,:) = RH(1,:);
        
        out.DCM(:,:,i) = eye(3);
    end
    clear i
    
    
    out.R_vic2lo_columnized = reshape(out.R_vic2lo, 9,size(out.R_vic2lo,3),1);
    
%     [out.H, out.H_i, out.yaw.radians, out.yaw.degrees, out.R_vic2lo] = AngleAxis2Homogenous(out.csv);
%     out.yaw.std.rad = std(out.yaw.radians);
%     out.yaw.mean.rad = mean(out.yaw.radians);
% 
%     out.R_vic2lo_columnized = reshape(out.R_vic2lo, 9,size(out.R_vic2lo,3),1);
%     out.x_axis = [out.R_vic2lo_columnized(1,:)' out.R_vic2lo_columnized(4,:)' out.R_vic2lo_columnized(7,:)'];
% 
%     [out.Q, out.q_i, out.axis, out.angle] = AngleAxis2Quat(out.csv);
% %     out.q_i = out.q_i;
%     x = out.q_i(:,1);
%     y = out.q_i(:,2);
%     z = out.q_i(:,3);
%     w = out.q_i(:,4);
%     out.yaw = atan2(2*(q.y.*q.z+q.w.*q.x), ...
%         (q.w.*q.w-q.x.*q.x-q.y.*q.y+q.z.*q.z));

%     out.yaw.radians = atan2(2*(x.*y - z.*w ) , 1 - 2*(y.^2 + z.^2));%atan2(2,1);
%     out.yaw.radians = atan2(2*(x.*y - z.*w ) , w.^2 - z.^2 - y.^2 - x.^2);%atan2(2,1);
%     out.yaw.radians = asin(-2*(x.*z - w.*y));
%     out.yaw.radians = asin(2*x.*y + 2*z.*w);
%     out.yaw.degrees = out.yaw.radians * 180 / pi;
end

function [Quaternion, Quaternion_i, Axis, Angle] = AngleAxis2Quat(Data)
%% Convert vicon angle-axis data to tb homegnous transform
    LengthOfData = length(Data);
    % Compute displacement of tb in vicon frame.
        DataP_v = [Data(:,[5,6,7])/1000]';% convert mm to meter
        
        Angle = sqrt(sum(Data(:,2:4).^2,2));
        for i = 1: LengthOfData
            if isequal(sign([-1 -1 -1]),sign(Data(i,2:4)))
                Angle(i) = -Angle(i);
                Data(i,2:4) = -Data(i,2:4);
            end
        end
        sinAngleOver2 = sin(Angle/2);
        cosAngleOver2 = cos(Angle/2);
        QX = (Data(:,2)./ Angle) .* sinAngleOver2 ;
        QY = (Data(:,3)./ Angle) .* sinAngleOver2 ;
        QZ = (Data(:,4)./ Angle) .* sinAngleOver2 ;
        QW = cosAngleOver2;
        
        Quaternion_i = [QX QY QZ QW];
        Quaternion = mean(Quaternion_i')';
        Axis = [(QX ./ sinAngleOver2) (QY ./ sinAngleOver2) (QZ ./ sinAngleOver2)];

end

function [Homogenous, Homogenous_i, yaw_rad, yaw_deg, Rotz] = AngleAxis2Homogenous(Data)
%% Convert vicon angle-axis data to tb homegnous transform

    LengthOfData = length(Data);

    Homogenous_i = zeros(4,4,LengthOfData);
    DataP_tb = zeros(3,LengthOfData);

    % Compute displacement of tb in vicon frame.
    DataP_v = [Data(:,[5,6,7])/1000]';

    % Compute rotation angle at each step
    DataR_Angle = sqrt( ...
    Data(:,2).*Data(:,2) + ...
    Data(:,3).*Data(:,3) + ...
    Data(:,4).*Data(:,4)) *pi/180;

    % Compute roation axis at each step
    DataR_Axis =[ ...
    Data(:,2)./DataR_Angle ...
    Data(:,3)./DataR_Angle ...
    Data(:,4)./DataR_Angle];

    DataR_w = skewCrossN(DataR_Axis);

    %    LengthOfData = length(Data);
    for i = 1:LengthOfData
    % Exponential Map from so(3) to SO(3): (also Rodrigues's formula) 
    % http://en.wikipedia.org/wiki/Axis?angle_representation#Exponential_map_from_so.283.29_to_SO.283.29
        DataR_Rotation(:,:,i) = eye(3) + ...
        sin(DataR_Angle(i))*DataR_w(:,:,i) + ...
        (1-cos(DataR_Angle(i)))*DataR_w(:,:,i)*DataR_w(:,:,i);
        % Compute position of Tb wrt the origin in the Tb frame
        %       DataP_tb(:,i) = DataR_Rotation(:,:,i)'*DataP_v(:,i);
        DataR_Rotation(1,3,i) = (DataR_Rotation(1,3,i)-DataR_Rotation(3,1,i))/2;
        DataR_Rotation(3,1,i) = -DataR_Rotation(1,3,i);
        DataR_Rotation(2,3,i) = (DataR_Rotation(2,3,i)-DataR_Rotation(3,2,i))/2;
        DataR_Rotation(3,2,i) = -DataR_Rotation(2,3,i);
        DataR_Rotation(:,:,i) = DataR_Rotation(:,:,i)/norm(DataR_Rotation(:,:,i));
        yaw_rad(i,1) = atan2(DataR_Rotation(2,1,i), DataR_Rotation(1,1,i));
        Rotz(:,:,i) = Rz(yaw_rad(i,1));
        %       pointing(i,:) = [Rotz(1,1,i) Rotz(1,2,i)];
        yaw_deg(i,1) = yaw_rad(i,1)*180/pi;

        % Compile homogenous matrix
        Homogenous_i(:,:,i) = [DataR_Rotation(:,:,i) , DataR_Rotation(:,:,i)*DataP_v(:,i); 0 0 0 1];
    end

    Homogenous = mean(Homogenous_i,3);


    stdH = std(Homogenous_i,0,3);


    % double theta;
    % double c, s, x, y, z;
    % double M[3][3];
    % theta = sqrt( ax*ax + ay*ay + az*az );
    % if (theta < 1e-15)
    % {
    % M[0][0] = M[1][1] = M[2][2] = 1.0;
    % M[0][1] = M[0][2] = M[1][0] = M[1][2] = M[2][0] = M[2][1] = 0.0; }
    % else {
    % x = (ax*pi/180)/theta;
    % y = (ay*pi/180)/theta;
    % z = (az*pi/180)/theta;
    % c = cos(theta);
    % s = sin(theta);
    % M[0][0] = c + (1-c)*x*x;
    % M[0][1] =     (1-c)*x*y + s*(-z);
    % M[0][2] =     (1-c)*x*z + s*y;
    % M[1][0] =     (1-c)*y*x + s*z;
    % M[1][1] = c + (1-c)*y*y;
    % M[1][2] =     (1-c)*y*z + s*(-x);
    % M[2][0] =     (1-c)*z*x + s*(-y);
    % M[2][1] =     (1-c)*z*y + s*x;
    % M[2][2] = c + (1-c)*z*z;
    % }


end