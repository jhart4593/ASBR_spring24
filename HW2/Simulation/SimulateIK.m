function [TTheta_body] = SimulateIK(BScrews,M,JointAngleIter,filename,gif_delay)
%+ Given the body screws of the serial chain robot, the zero configuration,
% and joint angles at each IK iteration, find the pose (or intermediate pose)
% aka the foreward kinematics using the body representation and simulate.
% Also input filename of gif and delay time between each gif frame.
%+ Reference: Section 6.2.2 Modern Robotics Mechanics, Planning and
% Control, W7-L1/2 notes
% Matlab gif function: https://www.mathworks.com/matlabcentral/fileexchange/63239-gif
%+ Revision List: 
%+ Rev 1.0: Initial Release

addpath('../CommonFunctions/');

% Find position of each joint at each iteration.
for jj = 1:size(JointAngleIter,1)
    for ii = 1:size(BScrews,2)
        theta = JointAngleIter(jj,1:ii); % Angle instances for joint ii
        M_ii = M{ii}; % zero pose for joint ii 
    
        BScrewsSub = BScrews(ii,1:ii); % Body Screws
    
        TTheta{jj,ii} = FK_body(BScrewsSub,M_ii,theta);
        Posx(jj,ii) = TTheta{jj,ii}(1,4);
        Posy(jj,ii) = TTheta{jj,ii}(2,4);
        Posz(jj,ii) = TTheta{jj,ii}(3,4);
    end
    
end

% Plot all joints at each incremental value. Save each frame and compile
% into gif.

% get min/max x/y/z values to set common axis limits
[min_x,max_x] = bounds(Posx,'all');
[min_y,max_y] = bounds(Posy,'all');
[min_z,max_z] = bounds(Posz,'all');
axis_lims = 1.1*[min_x max_x min_y max_y min_z max_z];

% plot home position of each joint
plotPos(Posx(1,:),Posy(1,:),Posz(1,:),axis_lims) 

gif(filename,'DelayTime',gif_delay,'LoopCount',0)

for kk = 1:(size(Posx,1)-1)
    plotPos(Posx(kk+1,:),Posy(kk+1,:),Posz(kk+1,:),axis_lims) 
    
    gif

end

%Output
TTheta_body = TTheta;

end