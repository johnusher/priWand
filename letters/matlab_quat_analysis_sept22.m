% matlab_quat_analysis_sept22.m

% using nim's analysis.ipynb

% IMU data recorded uisng test_save_IMU.go
% IMU unit: X is forward, Y to right,Z up
% record quaternions, gravity and euler data
% quats is w,x,y,z
% gavity is x,y,z
% euler is Bearing, Roll, Tilt



% all recordings draw letter L
% 1-5: button up, pointing south
% 5-10: button left, pointing south
% 11-15: button up, pointing west


clear all

% change to di of this file
if(~isdeployed)
    cd(fileparts(which(mfilename)));
end

skip = 10;

% pn = '../letters/feb22'
pn = 'C:\Users\john\Documents\Arduino\priWand\priWand\letters\O'
cd(pn)

d = dir;
d=d(~ismember({d.name},{'.','..'}));
measurements = d;
nMeasurements = size(measurements,1);

% subplot: m by n images for each measurement
sp_m = ceil(sqrt(nMeasurements));
sp_n = ceil(nMeasurements/sp_m);

% nDirs2 = sp_m*sp_n;
startInd = 1;
figure



for d = 1:nMeasurements
    
    dn=(measurements(d).name)
    
    cd(dn)
    
    %% read in quat data, gravity, and euler
    % quats is w,x,y,z
    % gavity is x,y,z
    
    
    fn = 'quat_data.txt';
    fn = 'quaternion_data.txt'
    X=importdata(fn,'%s');
    dataL_Quat = length(X);
    quat = [];
    for i = startInd:dataL_Quat
        quat(i-startInd+1,:) = str2num(cell2mat(X(i)));
    end
    dataL_Quat = floor(dataL_Quat-startInd+1);
    
    
    
    
    %%
    cd ..
    
    %% now process quats
    
    %% step 1.
    % continuously as we received quat data from the imu:
    
    projected = zeros(dataL_Quat,3);
    
    
    quats = quat;
    %         1. get3DTipTrajectory
    %       # Direction of the wand tip relative to quat frame.
    %   # Chosen by trial and error to give positive Z values and clear shapes.
    %   # 1, 0, 0  works reasonably well too.
    tip = [0.9, -0.5, -0.4];  %   tip = np.array([0.8, -0.5, -0.4], dtype=np.float64)
    %      tip = [1, 0, 0];
    tip = tip ./ norm(tip);   % tip /= np.linalg.norm(tip)
    
    result = zeros(size(quats,1),3);  % result = np.empty([quats.shape[0], 3], np.float64)
    
    for n=skip:dataL_Quat  % skip first 10
        %      result[i, :] = applyRotation(quats[i, :], tip);
        %        -> applyRotation: def applyRotation(q, p):
        %          return mulQuat(q, mulQuat(np.concatenate([[0], p]), invQuat(q)))[1:]
        %              ->mulQuat:
        % first mulQuat: mulQuat(np.concatenate([[0], p]), invQuat(q)))[1:]
        % def mulQuat(q0, q1):
        
        q = quats(n, :);
        p = tip;
        
        p_concat = [0, tip]; %   first arg into mulQuat= np.concatenate([[0], p])
        
        invQuatQ = [q(1) -q(2) -q(3) -q(4)];     %   second arg into mulQuat = invQuat(q))
        %  invQuat =  """Invert a quaternion rotation (quat conjugate)."""
        
        % now send this to 1st mulQuat
        
        q0 = p_concat;
        q1 = invQuatQ;
        
        w0 = q0(1);
        x0 = q0(2);
        y0 = q0(3);
        z0 = q0(4);
        
        w1 = q1(1);
        x1 = q1(2);
        y1 = q1(3);
        z1 = q1(4);
        
        firstMulQuatOut = [w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1 ,...
            w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1,...
            w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1,...
            w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1];
        
        % now send to second  mulQuat
        
        q0 = quats(i, :);
        
        q1 = firstMulQuatOut;
        
        w0 = q0(1);
        x0 = q0(2);
        y0 = q0(3);
        z0 = q0(4);
        
        w1 = q1(1);
        x1 = q1(2);
        y1 = q1(3);
        z1 = q1(4);
        
        secondMulQuatOut = [w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1 ,...
            w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1,...
            w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1,...
            w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1];
        
        %  we have the output of applyRotation
        projected(n,1) = secondMulQuatOut(2) ;
        projected(n,2) = secondMulQuatOut(3) ;
        projected(n,3) =  secondMulQuatOut(4) ;
        
    end
    
    x = projected(:,1);
    y = projected(:,2);
    z = projected(:,3);
    
    %     scatter3(x,y,z)
    
    %% part two:
    %      Now we want to project each trajectory into a small 2D image.
    %      The idea is to convert the trajectory back to Euler angles,
    %      find the smallest yaw and pitch ranges that encompass the whole trajectory,
    %      and treat those values as X and Y in the image space.
    
    yaw = [];
    pitch = [];
    for n=2:dataL_Quat-skip  % skip first 10
        %            angle_range = wrappingAngleRange(yawAngles(get3DTipTrajectory(recordings[i])))
        
        % def normaliseTrajectory(trajectory):
        yaw(n-1) = atan2(y(n),x(n));   %  return np.arctan2(trajectory[:, 0], trajectory[:, 1])
        pitch(n-1) = asin(z(n));  % return np.arcsin(trajectory[:, 2])
        
    end
    
    yaw=-yaw;
    
    %yaw_range = wrappingAngleRange(yaw)
    %       wrappingAngleRange : Returns the smallest angle range that encompasses all angles.
    %         Returns:   a tuple of two angles.
    %         If the second is smaller than the first, then the angle range contains pi.
    
    %     yawMin = min(yaw)*180/pi
    
    
    angles = yaw;
    angles = sort(angles);  % angles = np.sort(angles)
    angles_diff = angles - circshift(angles,1);  % angles_diff = angles - np.roll(angles, 1)
    
    %     angles_diff(1) = 2*pi+ angles_diff(1);  % angles_diff[0] = 2*np.pi + angles_diff[0]
    
    first_angleInd = find(angles_diff==max(angles_diff));  % first_angle = np.argmax(angles_diff)
    
    first_angle = angles(first_angleInd);
    
    %     angle_range = wrappingAngleRange(yawAngles(get3DTipTrajectory(recordings[i])))
    %   print(i, "[%0.1f, %0.1f]" % (angle_range[0] * 180 / np.pi, angle_range[1] * 180 / np.pi))
    %     angle_range = [first_angle * 180/pi last_angle * 180/pi];
    %     disp(angle_range)
    
    % now unwrap yaw
    
    yaw = yaw -  first_angle;  % yaw -= yaw_range[0]
    
    
    for n=1:dataL_Quat-skip-1   %yaw[yaw < 0] += 2 * np.pi
        if yaw(n)<0
            %             yaw(n) = yaw(n) + 2*pi;
            yaw(n) = 0;
        end
    end
    
    % now unwrap pitch:
    
    if(0)
        angles = pitch;
        angles = sort(angles);  % angles = np.sort(angles)
        angles_diff = angles - circshift(angles,1);  % angles_diff = angles - np.roll(angles, 1)
        %     angles_diff(1) = 2*pi+ angles_diff(1);  % angles_diff[0] = 2*np.pi + angles_diff[0]
        
        first_angleInd = find(angles_diff==max(angles_diff));  % first_angle = np.argmax(angles_diff)
        
        first_angle = angles(first_angleInd);
    end
    
    %     pitch = pitch -first_angle;
    
    
    pitch = pitch-min(pitch);
    for n=1:dataL_Quat-1-skip   %yaw[yaw < 0] += 2 * np.pi
        if pitch(n)<0
            %             pitch(n) = pitch(n) + 2*pi;
        end
    end
    
    %      for n=1:dataL_Quat-1   %yaw[yaw < 0] += 2 * np.pi
    %         if pitch(n)>1 || pitch(n)<0
    %             pitch(n) = 0;
    %         end
    %         if yaw(n)>1 || yaw(n)<0
    %             yaw(n) = 0;
    %         end
    %      end
    
        
    
%     hold on;plot(yaw,pitch,'x')
    
    x = yaw;
    y = pitch;
    
dataL_Quat = length(x);
    
    
%     % scale
%     
%     if(abs(min(x))>max(x))
%         maxX = min(x);
%     else
%         maxX = max(x);
%     end
%     
%     if(abs(min(y))>max(y))
%         maxY = min(y);
%     else
%         maxY = max(y);
%     end
%     
%     if abs(maxY)>abs(maxX)
%         maxdim = maxY;
%     else
%         maxdim = maxX;
%     end
%     
%     scaler = 1/abs(maxdim);
%     

    scaler = 1/max(y);
    x = x.*scaler;
    y = y.*scaler;
    
%     scaler
    
    
    
    %%
    % convert vector into bitmap
    
    m_x = 28;   % pixels in square
    m_y = m_x;
    m = zeros(m_x,m_x);
    
    x_int = round(x*(m_x-1));
    y_int = round(y*(m_x-1));
    
    
    for n=1:dataL_Quat
%         x_int_d = x_int(n)+m_x/2 + 1;
%         y_int_d = y_int(n)+m_y/2 + 1;
        
         x_int_d(n) = x_int(n)+ 1;
        y_int_d(n) = m_x-y_int(n)+ 1;
        
%                x_int_d = x_int(n)+1;
%         y_int_d = y_int(n)+1;
        
%         m(x_int_d(n),y_int_d(n)) = 1;
        m(y_int_d(n),x_int_d(n)) = 1;
    end
    
%     image 0,0 is top left
    
    %     subplot(sp_m,sp_n,d-2)
    figure
    I = mat2gray(m);
    imshow(I)
    
    ipn = 'C:\Users\john\Documents\Arduino\priWand\priWand\letters\';
    fn = [int2str(d) '_O2.bmp']
    imwrite(I,[ipn fn])
    
    

    
    
    
    
    
end

% figure;plot(yawMatrix,pitchMatrix,'x')

