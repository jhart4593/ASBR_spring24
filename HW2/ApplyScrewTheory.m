clc,clear, close all
addpath('../Unit4New')
addpath('./CommonFunctions')
addpath('./FKandIK')
addpath('./HarmonyData')
%% Use Config Values to Get DH Table and Zero Pose
load('KinematicSet.mat','ConfigVals'); 
side = 'R'; 

DH_to_joint_sign = ones(1,10); 
DH_to_joint_sign(5) = -1; 

if strcmpi(side,'L')
    sizes = [1016.8,201.667,255,314];
    angleSign = -1; 
elseif strcmpi(side,'R')
    sizes = [1016.8,176.667,274,278]; 
    angleSign = 1; 
end

dh_table = getDHTable(ConfigVals,sizes,side);
[JointFrameZero,RobotFrameZero] = getHarmonyZeroPose(dh_table);
AngleInstance = [3.109	3.188	0.987	-2.091	9.257	88.015	66.7]*pi/180; % Arbitrary angle instance, Can set to any angles
virtualTheta = angleSign*getVirtualTheta(AngleInstance);


%% Forward Kinematics
[SScrews, BScrews] = getScrews(RobotFrameZero.pose);
figure(1); clf;

for i = 1:numel(SScrews)
    theta = virtualTheta(1:i);
    M = RobotFrameZero.pose{i};
    SScrewsSub = SScrews(1:i); 
    BScrewsSub = BScrews(i,1:i); 

    TTheta{i,1} = FK_space(SScrewsSub,M,theta);
    TThetaBody{i,1} = FK_body(BScrewsSub,M,theta);
    plotPose(TTheta{i,1})
    hold on
end
xlabel('X Axis')
ylabel('Y Axis')
zlabel('Z Axis')
grid on; grid minor
title(['Forward Kinematics' ' ' side ' side'])
legend('','X Axis','Y axis','Z axis')
if 0
    % Checker code
    [JointFrame,~,robotFrame0T] = updateKinematics(AngleInstance,dh_table, DH_to_joint_sign*angleSign);
    for i = 1:numel(TTheta)
        isequal(round(TTheta{i},10), round(robotFrame0T.pose{i},10))
        isequal(round(TThetaBody{i},10), round(robotFrame0T.pose{i},10))
    end
end

%% Jacobian
JacobianSpace = J_space(SScrews,virtualTheta);
JacobianBody = J_body(BScrews(end,:),virtualTheta);

isequal(round(JacobianBody,10),round(T2Adj(Tinv(TTheta{10}))*JacobianSpace,10)) % Eqn 5.22
isequal(round(JacobianSpace,10),round(T2Adj(TTheta{10})*JacobianBody,10)) % Eqn 5.23

ellipsoid_plot_angular(JacobianBody);
ellipsoid_plot_linear(JacobianBody);
% % Displaying matrix
% mat = JacobianBody; 
% % fprintf('Space Jacobian:\n');
% fprintf('Body Jacobian:\n');
% for i = 1:size(mat, 1)
%     for j = 1:size(mat, 2)
%         if mat(i,j) == 0
%             fprintf('%.0f ', mat(i, j));
%         else
%             fprintf('%.4f ', mat(i, j));
%         end
%     end
%     fprintf('\n');
% end
%% Load Trajectory
% Using the right arm collected data wearing the robot for various
% trajectories to compare against IK
TrajNum = 2;

switch TrajNum
    case 1
    % Trajectory 1: Upwards shoulder press
    TrajData = readtable('Trajectory_1.txt'); 
    k1 = 175; % Traj 1
    k2 = 175; % Traj 1
    k3 = 1; % Traj 1
    k4 = 200; % Traj 1
    case 2
    % Trajectory 2: In move in parallel plane across the body
    TrajData = readtable('Trajectory_2.txt'); 
    k1 = 175; % Traj 2
    k2 = 100; % Traj 2
    k3 = 1; % Traj 2
    k4 = 100; % Traj 2
    case 3
    % Trajectory 3: Swipe bottom up across body 
    TrajData = readtable('Trajectory_3.txt'); 
    k1 = 200; % Traj 3
    k2 = 100; % Traj 3
    k3 = 1; % Traj 3
    k4 = 175; % Traj 3
end

TrajJointAngles = table2array(TrajData(1:end,2:8));

initialPos = TrajJointAngles(1,:); % initial position
finalPos = TrajJointAngles(end,:); % final position

virtualTheta0 = angleSign*getVirtualTheta(initialPos); % virtual theta initial position
virtualThetaf = angleSign*getVirtualTheta(finalPos); % virtual theta final position

% Get cartesian coordinates for the movement
origin = [0;0;0;1];
TrajCartPt = zeros(size(TrajJointAngles,1),3);
for i = 1:size(TrajJointAngles,1) 
    virtualTheta = angleSign*getVirtualTheta(TrajJointAngles(i,:));
    Tsb = FK_body(BScrews(end,:),M,virtualTheta);
    cartesian = Tsb*origin; 
    TrajCartPt(i,:) = cartesian(1:3)';
end
numTrajPts = size(TrajJointAngles,1);

%% Inverse Kinematics

[endEffectorPosIK, magsIK, VbIterIK, JointAngleIterIK] = J_inverse_kinematics(BScrews(end,3:end),M,virtualTheta0(3:end),virtualThetaf(3:end));
[endEffectorPosTK, magsTK, VbIterTK, JointAngleIterTK] = J_transpose_kinematics(BScrews(end,3:end),M,virtualTheta0(3:end),virtualThetaf(3:end));
[endEffectorPosDLS, magsDLS, VbIterDLS, JointAngleIterDLS] = DLS_inverse_kinematics(BScrews(end,3:end),M,virtualTheta0(3:end),virtualThetaf(3:end));

numIKpts = size(endEffectorPosIK,1);
numTKpts = size(endEffectorPosTK,1);
numDLSpts = size(endEffectorPosDLS,1);

x1 = 1:numTrajPts;
x2 = linspace(1,numTrajPts,numIKpts); 
x3 = linspace(1,numTrajPts,numTKpts);
x4 = linspace(1,numTrajPts,numDLSpts);

figure; clf % End Effector Position for Various IKs
subplot(3,1,1)

hold on;
plot(x1,TrajCartPt(:,1),'-o','Color',[0 0.4470 0.7410])
plot(x2,endEffectorPosIK(:,1),'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
plot(x3,endEffectorPosTK(:,1),'square','Color',[0.9290 0.6940 0.1250])
plot(x4,endEffectorPosDLS(:,1),'-+','Color',[0.4940 0.1840 0.5560],'LineWidth',2)
legend('Real Traj','Pseudo-Inverse','Jacobian Transpose','Damped Least Squares'); 

grid on; grid minor; 
title('Inverse Kinematics')
ylabel('X End Effector (m)')

subplot(3,1,2)

hold on;
plot(x1,TrajCartPt(:,2),'-o','Color',[0 0.4470 0.7410])
plot(x2,endEffectorPosIK(:,2),'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
plot(x3,endEffectorPosTK(:,2),'square','Color',[0.9290 0.6940 0.1250])
plot(x4,endEffectorPosDLS(:,2),'-+','Color',[0.4940 0.1840 0.5560],'LineWidth',2)
grid on; grid minor; 
ylabel('Y End Effector (m)')

subplot(3,1,3)

hold on;
plot(x1,TrajCartPt(:,3),'-o','Color',[0 0.4470 0.7410])
plot(x2,endEffectorPosIK(:,3),'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
plot(x3,endEffectorPosTK(:,3),'square','Color',[0.9290 0.6940 0.1250])
plot(x4,endEffectorPosDLS(:,3),'-+','Color',[0.4940 0.1840 0.5560],'LineWidth',2)
grid on; grid minor; 
ylabel('Z End Effector (m)')
xlabel('Sample Point')

% Manipulability metrics 
wIsoTraj = zeros(numTrajPts,1);
vIsoTraj = zeros(numTrajPts,1);
wCondTraj = zeros(numTrajPts,1);
vCondTraj = zeros(numTrajPts,1);
wVolTraj = zeros(numTrajPts,1);
vVolTraj = zeros(numTrajPts,1);

for i = 1:numTrajPts
    virtualTheta = angleSign*getVirtualTheta(TrajJointAngles(i,:));
    Jb = J_body(BScrews,virtualTheta);
    [angular_isotropy, linear_isotropy] = J_isotropy(Jb);
    [angular_condition, linear_condition] = J_condition(Jb);
    [angular_volume, linear_volume] = J_ellipsoid_volume(Jb);

    wIsoTraj(i,1) = angular_isotropy;
    vIsoTraj(i,1) = linear_isotropy;

    wCondTraj(i,1) = angular_condition;
    vCondTraj(i,1) = linear_condition;

    wVolTraj(i,1) = angular_volume;
    vVolTraj(i,1) = linear_volume;
end
wIsoIK = zeros(numIKpts,1);
vIsoIK = zeros(numIKpts,1);
wCondIK = zeros(numIKpts,1);
vCondIK = zeros(numIKpts,1);
wVolIK = zeros(numIKpts,1);
vVolIK = zeros(numIKpts,1);
mMetricIK = zeros(numIKpts,1);
for i = 1:numIKpts
    Jb = J_body(BScrews(end,3:end),JointAngleIterIK(i,:));
    [angular_isotropy, linear_isotropy] = J_isotropy(Jb);
    [angular_condition, linear_condition] = J_condition(Jb);
    [angular_volume, linear_volume] = J_ellipsoid_volume(Jb);

    wIsoIK(i,1) = angular_isotropy;
    vIsoIK(i,1) = linear_isotropy;

    wCondIK(i,1) = angular_condition;
    vCondIK(i,1) = linear_condition;

    wVolIK(i,1) = angular_volume;
    vVolIK(i,1) = linear_volume;
    mMetricIK(i,1) = sqrt((det(Jb*Jb')));
end

wIsoTK = zeros(numTKpts,1);
vIsoTK = zeros(numTKpts,1);
wCondTK = zeros(numTKpts,1);
vCondTK = zeros(numTKpts,1);
wVolTK = zeros(numTKpts,1);
vVolTK = zeros(numTKpts,1);

for i = 1:numTKpts
    Jb = J_body(BScrews(end,3:end),JointAngleIterTK(i,:));
    [angular_isotropy, linear_isotropy] = J_isotropy(Jb);
    [angular_condition, linear_condition] = J_condition(Jb);
    [angular_volume, linear_volume] = J_ellipsoid_volume(Jb);

    wIsoTK(i,1) = angular_isotropy;
    vIsoTK(i,1) = linear_isotropy;

    wCondTK(i,1) = angular_condition;
    vCondTK(i,1) = linear_condition;

    wVolTK(i,1) = angular_volume;
    vVolTK(i,1) = linear_volume;
end

wIsoDLS = zeros(numDLSpts,1);
vIsoDLS = zeros(numDLSpts,1);
wCondDLS = zeros(numDLSpts,1);
vCondDLS = zeros(numDLSpts,1);
wVolDLS = zeros(numDLSpts,1);
vVolDLS = zeros(numDLSpts,1);
for i = 1:numDLSpts
    Jb = J_body(BScrews(end,3:end),JointAngleIterDLS(i,:));
    [angular_isotropy, linear_isotropy] = J_isotropy(Jb);
    [angular_condition, linear_condition] = J_condition(Jb);
    [angular_volume, linear_volume] = J_ellipsoid_volume(Jb);

    wIsoDLS(i,1) = angular_isotropy;
    vIsoDLS(i,1) = linear_isotropy;

    wCondDLS(i,1) = angular_condition;
    vCondDLS(i,1) = linear_condition;

    wVolDLS(i,1) = angular_volume;
    vVolDLS(i,1) = linear_volume;
end

figure; % Manipulability Metrics for Various IKs
subplot(3,2,1)
title('Manipulability Metrics');
hold on;
plot(x1,wIsoTraj,'-o','Color',[0 0.4470 0.7410])
plot(x2,wIsoIK,'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
plot(x3,wIsoTK,'square','Color',[0.9290 0.6940 0.1250])
plot(x4,wIsoDLS,'-+','Color',[0.4940 0.1840 0.5560],'LineWidth',2)

grid on; grid minor; 
ylabel('Angular Isotropy')

subplot(3,2,2)

hold on;
plot(x1,vIsoTraj,'-o','Color',[0 0.4470 0.7410])
plot(x2,vIsoIK,'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
plot(x3,vIsoTK,'square','Color',[0.9290 0.6940 0.1250])
plot(x4,vIsoDLS,'-+','Color',[0.4940 0.1840 0.5560],'LineWidth',2)

grid on; grid minor; 
ylabel('Linear Isotropy')
legend('Real Traj','Pseudo-Inverse','Jacobian Transpose','Damped Least Squares'); 

subplot(3,2,3)

hold on;
plot(x1,wCondTraj,'-o','Color',[0 0.4470 0.7410])
plot(x2,wCondIK,'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
plot(x3,wCondTK,'square','Color',[0.9290 0.6940 0.1250])
plot(x4,wCondDLS,'-+','Color',[0.4940 0.1840 0.5560],'LineWidth',2)

grid on; grid minor; 
ylabel('Angular Condition')

subplot(3,2,4)

hold on;
plot(x1,vCondTraj,'-o','Color',[0 0.4470 0.7410])
plot(x2,vCondIK,'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
plot(x3,vCondTK,'square','Color',[0.9290 0.6940 0.1250])
plot(x4,vCondDLS,'-+','Color',[0.4940 0.1840 0.5560],'LineWidth',2)

grid on; grid minor; 
ylabel('Linear Condition')


subplot(3,2,5)

hold on;
plot(x1,wVolTraj,'-o','Color',[0 0.4470 0.7410])
plot(x2,wVolIK,'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
plot(x3,wVolTK,'square','Color',[0.9290 0.6940 0.1250])
plot(x4,wVolDLS,'-+','Color',[0.4940 0.1840 0.5560],'LineWidth',2)
xlabel('Sample Point')
grid on; grid minor; 
ylabel('Angular Volume')

subplot(3,2,6)

hold on;
plot(x1,vVolTraj,'-o','Color',[0 0.4470 0.7410])
plot(x2,vVolIK,'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
plot(x3,vVolTK,'square','Color',[0.9290 0.6940 0.1250])
plot(x4,vVolDLS,'-+','Color',[0.4940 0.1840 0.5560],'LineWidth',2)
xlabel('Sample Point')
grid on; grid minor; 
ylabel('Linear Volume')

%% Redundancy Resolution

% Get Robot Position Points
obstaclePos = [.103, -.0055, .8];
clearvars lowerJointLimit7Dof upperJointLimit7Dof
% Get Joint Limits
for i = 1:numel(initialPos)
    bnds = getJointLimits(i,side);
    lowerJointLimit7Dof(i,1) = bnds(1); 
    upperJointLimit7Dof(i,1) = bnds(2);
end
lowerJointLimit = [lowerJointLimit7Dof(1:2); lowerJointLimit7Dof(2); lowerJointLimit7Dof(3:end)];
upperJointLimit = [upperJointLimit7Dof(1:2); upperJointLimit7Dof(2); upperJointLimit7Dof(3:end)];


RobotFrameZeroSub.pose = RobotFrameZero.pose(3:end);
RobotFrameZeroSub.orientation = RobotFrameZero.orientation(3:end);
RobotFrameZeroSub.position = RobotFrameZero.position(3:end);

[endEffectorPosR1, magsR1, VbIterR1, JointAngleIterR1,objIterR1] = redundancy_resolution(BScrews(end,3:end),M,virtualTheta0(3:end),virtualThetaf(3:end),1,k1);
[endEffectorPosR2, magsR2, VbIterR2, JointAngleIterR2,objIterR2] = redundancy_resolution(BScrews(end,3:end),M,virtualTheta0(3:end),virtualThetaf(3:end),2,k2);
[endEffectorPosR3, magsR3, VbIterR3, JointAngleIterR3,objIterR3] = redundancy_resolution(BScrews(end,3:end),M,virtualTheta0(3:end),virtualThetaf(3:end),3,k3,upperJointLimit,lowerJointLimit);
[endEffectorPosR4, magsR4, VbIterR4, JointAngleIterR4,objIterR4] = redundancy_resolution(BScrews(end,3:end),M,virtualTheta0(3:end),virtualThetaf(3:end),4,k4,RobotFrameZeroSub,obstaclePos);

numR1pts = size(endEffectorPosR1,1);
numR2pts = size(endEffectorPosR2,1);
numR3pts = size(endEffectorPosR3,1);
numR4pts = size(endEffectorPosR4,1);


x5 = linspace(1,numTrajPts,numR1pts);
x6 = linspace(1,numTrajPts,numR2pts);
x7 = linspace(1,numTrajPts,numR3pts);
x8 = linspace(1,numTrajPts,numR4pts);

wIsoR1 = zeros(numR1pts,1);
vIsoR1 = zeros(numR1pts,1);
wCondR1 = zeros(numR1pts,1);
vCondR1 = zeros(numR1pts,1);
wVolR1 = zeros(numR1pts,1);
vVolR1 = zeros(numR1pts,1);

for i = 1:numR1pts
    Jb = J_body(BScrews(end,3:end),JointAngleIterR1(i,:));
    [angular_isotropy, linear_isotropy] = J_isotropy(Jb);
    [angular_condition, linear_condition] = J_condition(Jb);
    [angular_volume, linear_volume] = J_ellipsoid_volume(Jb);

    wIsoR1(i,1) = angular_isotropy;
    vIsoR1(i,1) = linear_isotropy;

    wCondR1(i,1) = angular_condition;
    vCondR1(i,1) = linear_condition;

    wVolR1(i,1) = angular_volume;
    vVolR1(i,1) = linear_volume;
end

wIsoR2 = zeros(numR2pts,1);
vIsoR2 = zeros(numR2pts,1);
wCondR2 = zeros(numR2pts,1);
vCondR2 = zeros(numR2pts,1);
wVolR2 = zeros(numR2pts,1);
vVolR2 = zeros(numR2pts,1);

for i = 1:numR2pts
    Jb = J_body(BScrews(end,3:end),JointAngleIterR2(i,:));
    [angular_isotropy, linear_isotropy] = J_isotropy(Jb);
    [angular_condition, linear_condition] = J_condition(Jb);
    [angular_volume, linear_volume] = J_ellipsoid_volume(Jb);

    wIsoR2(i,1) = angular_isotropy;
    vIsoR2(i,1) = linear_isotropy;

    wCondR2(i,1) = angular_condition;
    vCondR2(i,1) = linear_condition;

    wVolR2(i,1) = angular_volume;
    vVolR2(i,1) = linear_volume;
end

jointLimitObjIK = zeros(numIKpts,1);
obsAvoidObjIK = zeros(numIKpts,1);
for i = 1:numIKpts
    theta = JointAngleIterIK(i,:); 
    jointLimitObjIK(i,1) = jointLimitObj(theta,upperJointLimit,lowerJointLimit); 
    obsAvoidObjIK(i,1) = obsAvoidObj(theta,RobotFrameZeroSub,obstaclePos);
end
figure; clf  % End Effector Position for Manipulability Objective
subplot(3,1,1)

hold on;
plot(x2,endEffectorPosIK(:,1),'-o','Color',[0 0.4470 0.7410])
plot(x5,endEffectorPosR1(:,1),'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
plot(x6,endEffectorPosR2(:,1),'square','Color',[0.9290 0.6940 0.1250])
legend('IK','Manip Obj 1','Manip Obj 2'); 

grid on; grid minor; 
title('Inverse Kinematics')
ylabel('X End Effector (m)')

subplot(3,1,2)

hold on;
plot(x2,endEffectorPosIK(:,2),'-o','Color',[0 0.4470 0.7410])
plot(x5,endEffectorPosR1(:,2),'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
plot(x6,endEffectorPosR2(:,2),'square','Color',[0.9290 0.6940 0.1250])
grid on; grid minor; 
ylabel('Y End Effector (m)')

subplot(3,1,3)

hold on;
plot(x2,endEffectorPosIK(:,3),'-o','Color',[0 0.4470 0.7410])
plot(x5,endEffectorPosR1(:,3),'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
plot(x6,endEffectorPosR2(:,3),'square','Color',[0.9290 0.6940 0.1250])
grid on; grid minor; 
ylabel('Z End Effector (m)')
xlabel('Sample Point')

figure; % Manipulability for Manipulability Objective
subplot(3,2,1)
title('Manipulability Metrics');
hold on;
plot(x2,wIsoIK,'-o','Color',[0 0.4470 0.7410])
plot(x5,wIsoR1,'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
plot(x6,wIsoR2,'square','Color',[0.9290 0.6940 0.1250])

grid on; grid minor; 
ylabel('Angular Isotropy')

subplot(3,2,2)

hold on;
plot(x2,vIsoIK,'-o','Color',[0 0.4470 0.7410])
plot(x5,vIsoR1,'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
plot(x6,vIsoR2,'square','Color',[0.9290 0.6940 0.1250])

grid on; grid minor; 
ylabel('Linear Isotropy')
legend('IK','Manip Obj 1','Manip Obj 2'); 

subplot(3,2,3)

hold on;
plot(x2,wCondIK,'-o','Color',[0 0.4470 0.7410])
plot(x5,wCondR1,'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
plot(x6,wCondR2,'square','Color',[0.9290 0.6940 0.1250])

grid on; grid minor; 
ylabel('Angular Condition')

subplot(3,2,4)

hold on;
plot(x2,vCondIK,'-o','Color',[0 0.4470 0.7410])
plot(x5,vCondR1,'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
plot(x6,vCondR2,'square','Color',[0.9290 0.6940 0.1250])

grid on; grid minor; 
ylabel('Linear Condition')


subplot(3,2,5)

hold on;
plot(x2,wVolIK,'-o','Color',[0 0.4470 0.7410])
plot(x5,wVolR1,'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
plot(x6,wVolR2,'square','Color',[0.9290 0.6940 0.1250])
xlabel('Sample Point')
grid on; grid minor; 
ylabel('Angular Volume')

subplot(3,2,6)

hold on;
plot(x2,vVolIK,'-o','Color',[0 0.4470 0.7410])
plot(x5,vVolR1,'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
plot(x6,vVolR2,'square','Color',[0.9290 0.6940 0.1250])
xlabel('Sample Point')
grid on; grid minor; 
ylabel('Linear Volume')


figure; % Manipulability objective for Manipulability Objective

title('Manipulability Objective');
hold on;
plot(x2,mMetricIK,'-o','Color',[0 0.4470 0.7410])
plot(x5,objIterR1,'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
plot(x6,objIterR2,'-square','Color',[0.9290 0.6940 0.1250],'LineWidth',2)
legend('IK','Manip Obj 1','Manip Obj 2'); 
grid on; grid minor; 
ylabel('w')
xlabel('Sample Pts')

figure; clf % Joint angles for joint limit avoidance objective
subplot(4,2,1)

hold on;
jIndx = 1; 
plot(x1,TrajJointAngles(:,jIndx),'-o','Color',[0 0.4470 0.7410])
plot(x2,JointAngleIterIK(:,jIndx),'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
plot(x7,JointAngleIterR3(:,jIndx),'-square','Color',[0.9290 0.6940 0.1250],'LineWidth',2)
plot(x1,ones(size(x1))*upperJointLimit(jIndx),'Color',[0.6350 0.0780 0.1840],'LineWidth',2)
plot(x1,ones(size(x1))*lowerJointLimit(jIndx),'Color',[0.6350 0.0780 0.1840],'LineWidth',2)
grid on; grid minor
ylim([lowerJointLimit(jIndx)-1 upperJointLimit(jIndx)+1])
ylabel('Joint 1 (rad)')

subplot(4,2,2)

hold on;
jIndx = 2; 
plot(x1,TrajJointAngles(:,jIndx),'-o','Color',[0 0.4470 0.7410])
plot(x2,JointAngleIterIK(:,jIndx),'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
plot(x7,JointAngleIterR3(:,jIndx),'-square','Color',[0.9290 0.6940 0.1250],'LineWidth',2)
plot(x1,ones(size(x1))*upperJointLimit(jIndx),'Color',[0.6350 0.0780 0.1840],'LineWidth',2)
plot(x1,ones(size(x1))*lowerJointLimit(jIndx),'Color',[0.6350 0.0780 0.1840],'LineWidth',2)
legend('Real','IK','Joint Limit Obj'); 
ylim([lowerJointLimit(jIndx)-1 upperJointLimit(jIndx)+1])
grid on; grid minor
ylabel('Joint 2 (rad)')

subplot(4,2,3)

hold on;
jIndx = 3; 
plot(x1,-TrajJointAngles(:,2),'-o','Color',[0 0.4470 0.7410])
plot(x2,JointAngleIterIK(:,jIndx),'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
plot(x7,JointAngleIterR3(:,jIndx),'-square','Color',[0.9290 0.6940 0.1250],'LineWidth',2)
plot(x1,ones(size(x1))*upperJointLimit(jIndx),'Color',[0.6350 0.0780 0.1840],'LineWidth',2)
plot(x1,ones(size(x1))*lowerJointLimit(jIndx),'Color',[0.6350 0.0780 0.1840],'LineWidth',2)
ylim([lowerJointLimit(jIndx)-1 upperJointLimit(jIndx)+1])
grid on; grid minor
ylabel('Joint 2 virtual (rad)')

subplot(4,2,4)

hold on;
jIndx = 4; 
plot(x1,TrajJointAngles(:,jIndx-1),'-o','Color',[0 0.4470 0.7410])
plot(x2,JointAngleIterIK(:,jIndx),'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
plot(x7,JointAngleIterR3(:,jIndx),'-square','Color',[0.9290 0.6940 0.1250],'LineWidth',2)
plot(x1,ones(size(x1))*upperJointLimit(jIndx),'Color',[0.6350 0.0780 0.1840],'LineWidth',2)
plot(x1,ones(size(x1))*lowerJointLimit(jIndx),'Color',[0.6350 0.0780 0.1840],'LineWidth',2)
ylim([lowerJointLimit(jIndx)-1 upperJointLimit(jIndx)+1])
grid on; grid minor
ylabel('Joint 3 (rad)')

subplot(4,2,5)

hold on;
jIndx = 5; 
plot(x1,TrajJointAngles(:,jIndx-1),'-o','Color',[0 0.4470 0.7410])
plot(x2,JointAngleIterIK(:,jIndx),'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
plot(x7,JointAngleIterR3(:,jIndx),'-square','Color',[0.9290 0.6940 0.1250],'LineWidth',2)
plot(x1,ones(size(x1))*upperJointLimit(jIndx),'Color',[0.6350 0.0780 0.1840],'LineWidth',2)
plot(x1,ones(size(x1))*lowerJointLimit(jIndx),'Color',[0.6350 0.0780 0.1840],'LineWidth',2)
ylim([lowerJointLimit(jIndx)-1 upperJointLimit(jIndx)+1])
grid on; grid minor
ylabel('Joint 4 (rad)')

subplot(4,2,6)

hold on;
jIndx = 6; 
plot(x1,TrajJointAngles(:,jIndx-1),'-o','Color',[0 0.4470 0.7410])
plot(x2,JointAngleIterIK(:,jIndx),'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
plot(x7,JointAngleIterR3(:,jIndx),'-square','Color',[0.9290 0.6940 0.1250],'LineWidth',2)
plot(x1,ones(size(x1))*upperJointLimit(jIndx),'Color',[0.6350 0.0780 0.1840],'LineWidth',2)
plot(x1,ones(size(x1))*lowerJointLimit(jIndx),'Color',[0.6350 0.0780 0.1840],'LineWidth',2)
ylim([lowerJointLimit(jIndx)-1 upperJointLimit(jIndx)+1])
grid on; grid minor
ylabel('Joint 5 (rad)')

subplot(4,2,7)

hold on;
jIndx = 7; 
plot(x1,TrajJointAngles(:,jIndx-1),'-o','Color',[0 0.4470 0.7410])
plot(x2,JointAngleIterIK(:,jIndx),'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
plot(x7,JointAngleIterR3(:,jIndx),'-square','Color',[0.9290 0.6940 0.1250],'LineWidth',2)
plot(x1,ones(size(x1))*upperJointLimit(jIndx),'Color',[0.6350 0.0780 0.1840],'LineWidth',2)
plot(x1,ones(size(x1))*lowerJointLimit(jIndx),'Color',[0.6350 0.0780 0.1840],'LineWidth',2)
ylim([lowerJointLimit(jIndx)-1 upperJointLimit(jIndx)+1])
grid on; grid minor
ylabel('Joint 6 (rad)')

subplot(4,2,8)

hold on;
jIndx = 8; 
plot(x1,TrajJointAngles(:,jIndx-1),'-o','Color',[0 0.4470 0.7410])
plot(x2,JointAngleIterIK(:,jIndx),'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
plot(x7,JointAngleIterR3(:,jIndx),'-square','Color',[0.9290 0.6940 0.1250],'LineWidth',2)
plot(x1,ones(size(x1))*upperJointLimit(jIndx),'Color',[0.6350 0.0780 0.1840],'LineWidth',2)
plot(x1,ones(size(x1))*lowerJointLimit(jIndx),'Color',[0.6350 0.0780 0.1840],'LineWidth',2)
ylim([lowerJointLimit(jIndx)-1 upperJointLimit(jIndx)+1])
grid on; grid minor
ylabel('Joint 7 (rad)')


figure; % Joint limit objective 

title('Joint Limit Objective');
hold on;
plot(x2,jointLimitObjIK,'-o','Color',[0 0.4470 0.7410])
plot(x7,objIterR3,'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
legend('IK','Joint Limit Objective'); 
grid on; grid minor; 
ylabel('w')
xlabel('Sample Pts')

figure; clf % End Effector Postion for Obstacle Avoidance Objective
subplot(3,1,1)

hold on;
plot(x1,TrajCartPt(:,1),'-o','Color',[0 0.4470 0.7410])
plot(x2,endEffectorPosIK(:,1),'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
plot(x8,endEffectorPosR4(:,1),'-square','Color',[0.9290 0.6940 0.1250],'LineWidth',2)
plot(x1,ones(size(x1))*obstaclePos(1),'Color',[0.6350 0.0780 0.1840],'LineWidth',2)
legend('Real Traj','IK','Obstacle Avoidance','Obstacle'); 

grid on; grid minor; 
title('Inverse Kinematics')
ylabel('X End Effector (m)')

subplot(3,1,2)

hold on;
plot(x1,TrajCartPt(:,2),'-o','Color',[0 0.4470 0.7410])
plot(x2,endEffectorPosIK(:,2),'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
plot(x8,endEffectorPosR4(:,2),'-square','Color',[0.9290 0.6940 0.1250],'LineWidth',2)
plot(x1,ones(size(x1))*obstaclePos(2),'Color',[0.6350 0.0780 0.1840],'LineWidth',2)
grid on; grid minor; 
ylabel('Y End Effector (m)')

subplot(3,1,3)

hold on;
plot(x1,TrajCartPt(:,3),'-o','Color',[0 0.4470 0.7410])
plot(x2,endEffectorPosIK(:,3),'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
plot(x8,endEffectorPosR4(:,3),'-square','Color',[0.9290 0.6940 0.1250],'LineWidth',2)
plot(x1,ones(size(x1))*obstaclePos(3),'Color',[0.6350 0.0780 0.1840],'LineWidth',2)
grid on; grid minor; 
ylabel('Z End Effector (m)')
xlabel('Sample Point')

figure; % Obstacle avoidance objective 

title('Obstacle Avoidance Objective');
hold on;
plot(x2,obsAvoidObjIK,'-o','Color',[0 0.4470 0.7410])
plot(x8,objIterR4,'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
legend('IK','Obstacle Avoid Objective'); 
grid on; grid minor; 
ylabel('w')
xlabel('Sample Pts')

% figure;
% SScrewsSub = SScrews;
% for j = 1:numIKpts
%     theta = [0 0 JointAngleIterIK(j,:)];
%     % Compute forward Kinematics
%     for i = 1:numel(SScrewsSub)
%         thetaSub = theta(1:i);
%         SScrewsSubSub =  SScrewsSub(1:i); 
%         M = RobotFrameZero.pose{i};
%         TTheta{i,1} = FK_body(SScrewsSubSub,M,thetaSub);
%     end
%     
%     % Get Robot Position
%     numFrames = numel(SScrewsSub); 
%     for i = 1:numFrames
%         TTemp = TTheta{i};
%         posePos(i,:) = TTemp(1:3,4);
%     end
%     robotPos = getRobotPosition(posePos);
%     plot3(robotPos(:,1),robotPos(:,2),robotPos(:,3),'-*','Color',[0 0.4470 0.7410],'LineWidth',2)
%     hold on
% end
% grid on; grid minor
% plot3(obstaclePos(1),obstaclePos(2),obstaclePos(3),'square','MarkerSize',10,'MarkerFaceColor',[0.6350 0.0780 0.1840])
% 
% 
% 
% figure;
% SScrewsSub = SScrews;
% for j = 1:numIKpts
%     theta = [0 0 JointAngleIterR4(j,:)];
%     % Compute forward Kinematics
%     for i = 1:numel(SScrewsSub)
%         thetaSub = theta(1:i);
%         SScrewsSubSub =  SScrewsSub(1:i); 
%         M = RobotFrameZero.pose{i};
%         TTheta{i,1} = FK_body(SScrewsSubSub,M,thetaSub);
%     end
%     
%     % Get Robot Position
%     numFrames = numel(SScrewsSub); 
%     for i = 1:numFrames
%         TTemp = TTheta{i};
%         posePos(i,:) = TTemp(1:3,4);
%     end
%     robotPos = getRobotPosition(posePos);
%     plot3(robotPos(:,1),robotPos(:,2),robotPos(:,3),'-*','Color',[0.9290 0.6940 0.1250],'LineWidth',2)
%     hold on
% end
% grid on; grid minor
% plot3(obstaclePos(1),obstaclePos(2),obstaclePos(3),'square','MarkerSize',10,'MarkerFaceColor',[0.6350 0.0780 0.1840])
%% Testing IK 
% Checker Code
% Reference: Example 6.1 Modern Robotics Mechanics, Planning and Control
BScrews = {[0;0;1;0;2;0], [0;0;1;0;1;0]}; 

M = [1 0 0 2
    0 1 0 0
    0 0 1 0
    0 0 0 1]; 
desiredAngle = [30;90]*pi/180;
initialAngle = [0;30]*pi/180; 
[endEffectorPosIK, magsIK, VbIterIK, JointAngleIterIK] = J_inverse_kinematics(BScrews,M,initialAngle,desiredAngle);
[endEffectorPosTK, magsTK, VbIterTK, JointAngleIterTK] = J_transpose_kinematics(BScrews,M,initialAngle,desiredAngle);
[endEffectorPosDLS, magsDLS, VbIterDLS, JointAngleIterDLS] = DLS_inverse_kinematics(BScrews,M,initialAngle,desiredAngle);


numIKpts = size(endEffectorPosIK,1);
numTKpts = size(endEffectorPosTK,1);
numDLSpts = size(endEffectorPosDLS,1);

x2 = 1:numTKpts;
x1 = linspace(1,numTKpts,numIKpts); 
x3 = linspace(1,numTKpts,numDLSpts);

figure; clf
subplot(2,1,1)

hold on;
plot(x1,endEffectorPosIK(:,1),'-o','Color',[0 0.4470 0.7410],'LineWidth',2)
plot(x2,endEffectorPosTK(:,1),'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
plot(x3,endEffectorPosDLS(:,1),'-square','Color',[0.9290 0.6940 0.1250],'LineWidth',2)
legend('Pseudo-Inverse','Jacobian Transpose','Damped Least Squares'); 

grid on; grid minor; 
title('Inverse Kinematics')
ylabel('X End Effector (m)')

subplot(2,1,2)

hold on;
plot(x1,endEffectorPosIK(:,2),'-o','Color',[0 0.4470 0.7410],'LineWidth',2)
plot(x2,endEffectorPosTK(:,2),'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
plot(x3,endEffectorPosDLS(:,2),'-square','Color',[0.9290 0.6940 0.1250],'LineWidth',2)
grid on; grid minor; 
ylabel('Y End Effector (m)')

figure; 
subplot(2,1,1)

hold on;
plot(x1,JointAngleIterIK(:,1),'-o','Color',[0 0.4470 0.7410],'LineWidth',2)
plot(x2,JointAngleIterTK(:,1),'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
plot(x3,JointAngleIterDLS(:,1),'-square','Color',[0.9290 0.6940 0.1250],'LineWidth',2)
grid on; grid minor; 
ylabel('Joint Angle 1 (rad)')
xlabel('Sample Point')
legend('Pseudo-Inverse','Jacobian Transpose','Damped Least Squares'); 

subplot(2,1,2)

hold on;
plot(x1,JointAngleIterIK(:,2),'-o','Color',[0 0.4470 0.7410],'LineWidth',2)
plot(x2,JointAngleIterTK(:,2),'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
plot(x3,JointAngleIterDLS(:,2),'-square','Color',[0.9290 0.6940 0.1250],'LineWidth',2)
grid on; grid minor; 
ylabel('Joint Angle 2 (rad)')
xlabel('Sample Point')
%% Testing Redundancy Resolution 
BScrews = {[0;0;1;0;2;0], [0;0;1;0;1;0]}; 
M1 = [1 0 0 0
    0 1 0 0
    0 0 1 0
    0 0 0 1]; 
M2 = [1 0 0 1
    0 1 0 0
    0 0 1 0
    0 0 0 1]; 
M3 = [1 0 0 2
    0 1 0 0
    0 0 1 0
    0 0 0 1]; 
desiredAngle = [30;90]*pi/180;
initialAngle = [0;30]*pi/180; 

upperJointLimit = [90;90]*pi/180;
lowerJointLimit = [0;0];
k1 = 200; % Traj 3
k2 = 100; % Traj 3
k3 = 1; % Traj 3
k4 = 175; % Traj 3
ZeroPose.pose = {M1 M2 M3};
[SScrews,BScrews1] = getScrews(ZeroPose.pose);
ZeroPose.pose = {M2 M3};
obstaclePos = [.569;.777];
[endEffectorPosIK, magsIK, VbIterIK, JointAngleIterIK] = J_inverse_kinematics(BScrews,M3,initialAngle,desiredAngle);
[endEffectorPosR1, magsR1, VbIterR1, JointAngleIterR1] = redundancy_resolution(BScrews,M3,initialAngle,desiredAngle,1,k1);
[endEffectorPosR2, magsR2, VbIterR2, JointAngleIterR2] = redundancy_resolution(BScrews,M3,initialAngle,desiredAngle,2,k2);
[endEffectorPosR3, magsR3, VbIterR3, JointAngleIterR3] = redundancy_resolution(BScrews,M3,initialAngle,desiredAngle,3,k3,upperJointLimit,lowerJointLimit);
[endEffectorPosR4, magsR4, VbIterR4, JointAngleIterR4] = redundancy_resolution(BScrews,M3,initialAngle,desiredAngle,4,k4,ZeroPose,obstaclePos);

numIKPts = size(JointAngleIterIK,1);
numR1pts = size(JointAngleIterR1,1);
numR2pts = size(JointAngleIterR2,1);
numR3pts = size(JointAngleIterR3,1);
numR4pts = size(JointAngleIterR4,1);

x1 = 1:numIKPts;
x2 = linspace(1,numIKPts,numR1pts);
x3 = linspace(1,numIKPts,numR2pts);
x4 = linspace(1,numIKPts,numR3pts);
x5 = linspace(1,numIKPts,numR4pts);


figure;
for j = 1:numIKPts
    theta = JointAngleIterIK(j,:);
    % Compute forward Kinematics
    for i = 1:numel(BScrews)
        thetaSub = theta(1:i);
        BScrewsSub = BScrews(1:i); 
        M = ZeroPose.pose{i};
        TTheta{i,1} = FK_body(BScrewsSub,M,thetaSub);
    %     plotPose(TTheta{i,1})
    %     hold on
    end
    
    % Get Robot Position
    numFrames = numel(BScrews); 
    for i = 1:numFrames
        TTemp = TTheta{i};
        posePos(i,:) = TTemp(1:3,4);
    end
    robotPos = getRobotPosition(posePos);
    plot(robotPos(:,1),robotPos(:,2),'-*','Color',[0 0.4470 0.7410],'LineWidth',2)
    hold on
end
plot(obstaclePos(1),obstaclePos(2),'square','MarkerSize',10,'MarkerFaceColor',[0.6350 0.0780 0.1840])

for i = 1:size(JointAngleIterIK,1)
    theta = JointAngleIterIK(i,:);
    Jb = J_body(BScrews,theta);
    wIK(i) =sqrt((det(Jb*Jb')));
end


for i = 1:size(JointAngleIterR1,1)
    theta = JointAngleIterR1(i,:);
    Jb = J_body(BScrews,theta);
    wR1(i) =sqrt((det(Jb*Jb')));
end


for i = 1:size(JointAngleIterR2,1)
    theta = JointAngleIterR2(i,:);
    Jb = J_body(BScrews,theta);
    wR2(i) =sqrt((det(Jb*Jb')));
end


% figure; clf
% subplot(2,1,1)
% 
% hold on;
% plot(x1,endEffectorPosIK(:,1),'-o','Color',[0 0.4470 0.7410],'LineWidth',2)
% plot(x2,endEffectorPosR1(:,1),'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
% plot(x3,endEffectorPosR2(:,1),'-square','Color',[0.9290 0.6940 0.1250],'LineWidth',2)
% legend('IK','Manip Obj 1','Manip Obj 2'); 
% 
% grid on; grid minor; 
% title('Inverse Kinematics')
% ylabel('X End Effector (m)')
% 
% subplot(2,1,2)
% 
% hold on;
% plot(x1,endEffectorPosIK(:,2),'-o','Color',[0 0.4470 0.7410],'LineWidth',2)
% plot(x2,endEffectorPosR1(:,2),'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
% plot(x3,endEffectorPosR2(:,2),'-square','Color',[0.9290 0.6940 0.1250],'LineWidth',2)
% grid on; grid minor; 
% ylabel('Y End Effector (m)')
% 
% figure; 
% subplot(2,1,1)
% 
% hold on;
% plot(x1,wIK,'-o','Color',[0 0.4470 0.7410],'LineWidth',2)
% plot(x2,wR1,'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
% plot(x3,wR2,'-square','Color',[0.9290 0.6940 0.1250],'LineWidth',2)
% grid on; grid minor; 
% ylabel('Manipulability Metric')
% xlabel('Sample Point')
% legend('IK','Manip Obj 1','Manip Obj 2');  

figure; clf
subplot(2,1,1)

hold on;
plot(x1,endEffectorPosIK(:,1),'-o','Color',[0 0.4470 0.7410],'LineWidth',2)
plot(x4,endEffectorPosR3(:,1),'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
legend('Pseudo-Inverse','Joint Limit Avoidance'); 

grid on; grid minor; 
title('Inverse Kinematics')
ylabel('X End Effector (m)')

subplot(2,1,2)

hold on
plot(x1,endEffectorPosIK(:,2),'-o','Color',[0 0.4470 0.7410],'LineWidth',2)
plot(x4,endEffectorPosR3(:,2),'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
grid on; grid minor; 
ylabel('Y End Effector (m)')

figure; 
subplot(2,1,1)
jIndx = 1; 
hold on;
plot(x1,JointAngleIterIK(:,1),'-o','Color',[0 0.4470 0.7410],'LineWidth',2)
plot(x4,JointAngleIterR3(:,1),'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
plot(x1,ones(size(x1))*upperJointLimit(jIndx),'Color',[0.6350 0.0780 0.1840],'LineWidth',2)
plot(x1,ones(size(x1))*lowerJointLimit(jIndx),'Color',[0.6350 0.0780 0.1840],'LineWidth',2)
ylim([lowerJointLimit(jIndx)-1 upperJointLimit(jIndx)+1])
grid on; grid minor; 
ylabel('Joint Angle 1 (rad)')
xlabel('Sample Point')
legend('Pseudo-Inverse','Joint Limit Avoidance');  

subplot(2,1,2)
jIndx = 2; 
hold on;
plot(x1,JointAngleIterIK(:,2),'-o','Color',[0 0.4470 0.7410],'LineWidth',2)
plot(x4,JointAngleIterR3(:,2),'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
plot(x1,ones(size(x1))*upperJointLimit(jIndx),'Color',[0.6350 0.0780 0.1840],'LineWidth',2)
plot(x1,ones(size(x1))*lowerJointLimit(jIndx),'Color',[0.6350 0.0780 0.1840],'LineWidth',2)
ylim([lowerJointLimit(jIndx)-1 upperJointLimit(jIndx)+1])

grid on; grid minor; 
ylabel('Joint Angle 2 (rad)')
xlabel('Sample Point')


figure; clf
subplot(2,1,1)

hold on;
plot(x1,endEffectorPosIK(:,1),'-o','Color',[0 0.4470 0.7410],'LineWidth',2)
plot(x5,endEffectorPosR4(:,1),'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
plot(x1,ones(size(x1))*obstaclePos(1),'Color',[0.6350 0.0780 0.1840],'LineWidth',2)
legend('Pseudo-Inverse','Obstacle Avoidance'); 

grid on; grid minor; 
title('Inverse Kinematics')
ylabel('X End Effector (m)')

subplot(2,1,2)

hold on
plot(x1,endEffectorPosIK(:,2),'-o','Color',[0 0.4470 0.7410],'LineWidth',2)
plot(x5,endEffectorPosR4(:,2),'-*','Color', [0.8500 0.3250 0.0980],'LineWidth',2)
plot(x1,ones(size(x1))*obstaclePos(2),'Color',[0.6350 0.0780 0.1840],'LineWidth',2)
grid on; grid minor; 
ylabel('Y End Effector (m)')