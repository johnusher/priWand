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

skip = 11;

% pn = '../letters/feb22'
% pn = 'C:\Users\john\Documents\Arduino\priWand\priWand\letters\M'
% pn = 'C:\Users\john\Documents\Arduino\priWand\john'
pn = 'C:\Users\john\Documents\Arduino\priWand\2710\M2'
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
% figure

% nSubplots = nMeasurements;
mSubplots  = ceil(sqrt(nMeasurements));
nSubplots  = ceil(nMeasurements/mSubplots);



for d = 1:nMeasurements
    
    dn=(measurements(d).name)
    
%     dn = 'M_22-14-45'
%     dn = 'M_20-45-40'
 
    
    cd(dn)
    
    %% read in quat data, gravity, and euler
    % quats is w,x,y,z
    % gavity is x,y,z
    
    
    fn = 'quat_data.txt';
%         fn = 'quaternion_data.txt'
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
    tip = [0.8, -0.5, -0.4];  %   tip = np.array([0.8, -0.5, -0.4], dtype=np.float64)
    %      tip = [1, 0, 0];
    tip = tip ./ norm(tip);   % tip /= np.linalg.norm(tip)
    
    result = zeros(size(quats,1),3);  % result = np.empty([quats.shape[0], 3], np.float64)
    quatsA = quats;quats=[];
    quats = quatsA(11:end,:);
    for n=1:dataL_Quat-skip-1  % skip first 10
        %      result[i, :] = applyRotation(quats[i, :], tip);
        %        -> applyRotation: def applyRotation(q, p):
        %          return mulQuat(q, mulQuat(np.concatenate([[0], p]), invQuat(q)))[1:]
        %              ->mulQuat:
        % first mulQuat: mulQuat(np.concatenate([[0], p]), invQuat(q)))[1:]
        % def mulQuat(q0, q1):
        
        s = quats(n,1);
        x = quats(n,2);
        y = quats(n,3);
        z = quats(n,4);
        
        x0 = tip(1);
        y0 = tip(2);
        z0 = tip(3);
        
        w1 = s;
        x1 = -x;
        y1 = -y;
        z1 = -z;
        
        firstMulQuatOut = [ - x0 * x1 - y0 * y1 - z0 * z1 ,...
            x0 * w1 + y0 * z1 - z0 * y1,...
            - x0 * z1 + y0 * w1 + z0 * x1,...
            x0 * y1 - y0 * x1 + z0 * w1];
        
        % now send to second  mulQuat
        
        w0 = s;
        x0 = x;
        y0 = y;
        z0 = z;
        
        w1 = firstMulQuatOut(1);
        x1 = firstMulQuatOut(2);
        y1 = firstMulQuatOut(3);
        z1 = firstMulQuatOut(4);
        
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
    for n=1:dataL_Quat-skip-1  % skip first 10
        %            angle_range = wrappingAngleRange(yawAngles(get3DTipTrajectory(recordings[i])))
        
        % def normaliseTrajectory(trajectory):
        yaw(n) = atan2(x(n),y(n));   %  return np.arctan2(trajectory[:, 0], trajectory[:, 1])
        pitch(n) = asin(z(n));  % return np.arcsin(trajectory[:, 2])
        
    end
     
%     normaliseTrajectory:

    angles = yaw;
    angles = sort(angles);  % angles = np.sort(angles)
    angles_diff = angles - circshift(angles,1);  % angles_diff = angles - np.roll(angles, 1)
    
    angles_diff(1) = 2*pi+ angles_diff(1);  % angles_diff[0] = 2*np.pi + angles_diff[0]
    
    first_angleInd = find(angles_diff==max(angles_diff));  % first_angle = np.argmax(angles_diff)
    
    first_angle = angles(first_angleInd);
    
    yaw_range = first_angle;
    
    
    % now unwrap pitch:
%       # note: wrappingAngleRange would work for pitch, but we don't need it, since the
%   # angles are in [-pi/2, pi/2] and don't wrap around
%     

   pitch = pitch - min(pitch); 
   yaw = yaw - first_angle;
   
   
   for n=1:dataL_Quat-skip-1   %yaw[yaw < 0] += 2 * np.pi
       if yaw(n)<0
           yaw(n) = yaw(n) + 2*pi;
       end
       if pitch(n)<0
           pitch(n) = pitch(n) + 2*pi;
       end
   end
   
   % now  fitInBox(yaw, pitch):
   
   min_x = min(yaw);
   x_range = max(yaw) - min_x;
   min_y = min(pitch);
   y_range = max(pitch) - min_y;
   scale = 1.0 / max(x_range, y_range);
   yaw = 0.5 + (yaw - min_x - x_range / 2) * scale;
   pitch = 0.5 + (pitch - min_y - y_range / 2) * scale;
    
    
    x = yaw;
    y = pitch;
    
%      figure;plot(yaw,pitch,'x')
    
    dataL_Quat = length(x);
    
    if(1)
%         scaler = 1/max(y);
%         x = x.*scaler;
%         y = y.*scaler;
        
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

            m(y_int_d(n),x_int_d(n)) = 1;
        end
        
    end
    
    %     image 0,0 is top left
% nSubplots = nMeasurements;
% mSubplots  = ceil(sqrt(nMeasurements))
% nSubplots  = ceil(nMeasurements/m);
%     figure
    subplot(mSubplots,nSubplots,d)
    
    I = mat2gray(m);
    imshow(I)
    
    
    if(0)
%     ipn = 'C:\Users\john\Documents\Arduino\priWand\priWand\letters\';
%     fn = [int2str(d) '_O2x.bmp']
    fn = [pn '\' dn '\' 'O.bmp']
%     imwrite(I,[ipn fn])   
    imwrite(I,[fn]) 
    end
    
 
    
    
end

% figure;plot(yawMatrix,pitchMatrix,'x')

