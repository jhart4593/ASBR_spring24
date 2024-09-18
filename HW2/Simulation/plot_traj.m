function [joint_pos,TTheta_traj] = plot_traj(BScrews,M,trajectory,filename)
%+ Given the body screws of the serial chain robot, the zero configuration,
% and measured joint angle positions at each time step, simulate motion of 
% Harmony over time.
% Matlab gif function: https://www.mathworks.com/matlabcentral/fileexchange/63239-gif
%+ Revision List: 
%+ Rev 1.0: Initial Release

% Get virtual 
for ii = 1:size(trajectory,1)
    virtualTheta(ii,:) = getVirtualTheta(trajectory(ii,:));
end    

% Get cartesian coordinates of each joint at each iteration
for jj = 1:size(virtualTheta,1)
    for ii = 1:size(BScrews,2)
        theta = virtualTheta(jj,1:ii); % Angle instances for joint ii
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

gif(filename,'LoopCount',0)

for kk = 1:(size(Posx,1)-1)
    plotPos(Posx(kk+1,:),Posy(kk+1,:),Posz(kk+1,:),axis_lims) 
    
    gif

end

% Output
joint_pos{1} = Posx;
joint_pos{2} = Posy;
joint_pos{3} = Posz;
TTheta_traj = TTheta;
end