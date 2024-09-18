function [TTheta_space] = SimulateFK(Screws,M,theta_final,filename)
%+ Given the screws of the serial chain robot, the zero configuration, and
% the final joint angles (radians), find the pose (or intermediate pose) aka the foreward
% kinematics using the space representation and simulate.
% Also input gif filename.
%+ Output:
% Pose of each joint at each iteration (FK)
%+ Reference: Chs 4-6 Modern Robotics Mechanics, Planning and Control 
% Matlab gif function: https://www.mathworks.com/matlabcentral/fileexchange/63239-gif
%+ Revision List: 
%+ Rev 1.0: Initial Release

% delta - sets simulation step number from start to final position
options.delta = 50;

addpath('../CommonFunctions/');

% Simulation using FK--------------------------------------------------

% Calculate position of each joint at all increments starting from
% initial position
% calculate initial angles of each joint from M
theta0 = [];
for m = 1:numel(Screws)
    [~,angle] = T2ScrewAxisandAng(M{m});
    theta0(m) = angle;
end

% split final theta values for each joint into specified increments
theta_delta = (theta_final-theta0)/options.delta;

theta_new = theta0;
for jj = 1:options.delta+1
    for ii = 1:numel(Screws)
        theta = theta_new(1:ii); % Angle instances for joint ii
        M_ii = M{ii}; % zero pose for joint ii 
    
        ScrewsSub = Screws(1:ii); % Space Screws
    
        TTheta{jj,ii} = FK_space(ScrewsSub,M_ii,theta);
        Posx(jj,ii) = TTheta{jj,ii}(1,4);
        Posy(jj,ii) = TTheta{jj,ii}(2,4);
        Posz(jj,ii) = TTheta{jj,ii}(3,4);
    end
    theta_new = theta_new + theta_delta;
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

gif(filename,'LoopCount',0)

for kk = 1:options.delta
    plotPos(Posx(kk+1,:),Posy(kk+1,:),Posz(kk+1,:),axis_lims) 
    
    gif

end

% Output-----------------------------------------------------------------
TTheta_space = TTheta;
end