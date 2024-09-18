% Simulation Cases for Harmony Robot
% 1. plot 3 separate measured trajectories of robot
% 2. simulate FK of three separate trajectories using final joint angles
% 3. simulate IK of trajectory using pseudo-inverse,
% jacobian transpose, and redundancy analysis for each trajectory
addpath('../CommonFunctions')
addpath('../HarmonyData')
addpath('../FKandIK')
addpath(genpath('./gif'))
%% Measured Trajectories
% plot three separate measured trajectories for comparison
clc;clear;close all

load('IKInfo.mat')

M = RobotFrameZero.pose;

% Trajectory 1: Upwards shoulder press
TrajData1 = readtable('Trajectory_1.txt'); 
TrajJointAngles1 = table2array(TrajData1(1:end,2:8));
T1 = [TrajJointAngles1(1:100:end,:); TrajJointAngles1(end,:)];

% Trajectory 2: In move in parallel plane across the body
TrajData2 = readtable('Trajectory_2.txt'); 
TrajJointAngles2 = table2array(TrajData2(1:end,2:8));
T2 = [TrajJointAngles2(1:100:end,:); TrajJointAngles2(end,:)];

% Trajectory 3: Swipe bottom up across body 
TrajData3 = readtable('Trajectory_3.txt'); 
TrajJointAngles3 = table2array(TrajData3(1:end,2:8));
T3 = [TrajJointAngles3(1:100:end,:); TrajJointAngles3(end,:)];

% Plot each trajectory and save as gif
[joint_angles1,TTheta1] = plot_traj(BScrews,M,T1,'./Results/traj_1.gif');
[joint_angles2,TTheta2] = plot_traj(BScrews,M,T2,'./Results/traj_2.gif');
[joint_angles3,TTheta3] = plot_traj(BScrews,M,T3,'./Results/traj_3.gif');

save('./Results/Traj.mat','TTheta1','TTheta2','TTheta3','joint_angles1','joint_angles2','joint_angles3')
% View gifs
% web('traj_1.gif')
% web('traj_2.gif')
% web('traj_3.gif')

%% FK - use final joint angles of measured trajectories
clc;clear;close all

load('IKInfo.mat')

M = RobotFrameZero.pose;

% Compute FK for each trajectory using final angle positions
FKsim1 = SimulateFK(SScrews,M,virtDesPos(1,:),'./Results/FK_sim1.gif'); % Trajectory 1: Upwards shoulder press
FKsim2 = SimulateFK(SScrews,M,virtDesPos(2,:),'./Results/FK_sim2.gif'); % Trajectory 2: In move in parallel plane across the body
FKsim3 = SimulateFK(SScrews,M,virtDesPos(3,:),'./Results/FK_sim3.gif'); % Trajectory 3: Swipe bottom up across body 

save('./Results/FKsim.mat','FKsim1','FKsim2','FKsim3')
% View gifs
% web('FK_sim1.gif')
% web('FK_sim2.gif')
% web('FK_sim3.gif')

%% IK
% Simulate Trajectory 1 IK using pseduo-inverse, jacobian transpose, and
% redundancy resolution methods.
clc;clear;close all

load('IKInfo.mat')

M = RobotFrameZero.pose;

% Pseudo-inverse
[~, ~, ~, JointAngleIterIK] = J_inverse_kinematics(BScrews(end,3:end),M{end},virInitPos(1,3:end)',virtDesPos(1,3:end)');

% Jacobian transpose
[~, ~, ~, JointAngleIterJK] = J_transpose_kinematics(BScrews(end,3:end),M{end},virInitPos(1,3:end)',virtDesPos(1,3:end)');

% Redundancy resolution
[~, ~, ~, JointAngleIterRK] = redundancy_resolution(BScrews(end,3:end),M{end},virInitPos(1,3:end)',virtDesPos(1,3:end)',2,175);


JointAngleIterIK = [zeros(size(JointAngleIterIK,1),2) JointAngleIterIK];
JointAngleIterJK = [zeros(size(JointAngleIterJK,1),2) JointAngleIterJK];
JointAngleIterRK = [zeros(size(JointAngleIterRK,1),2) JointAngleIterRK];
% Compute IK for trajectory 1 using three different methods given joint 
% angles at each IK iteration, then simulate.
IKsim = SimulateIK(BScrews,M,JointAngleIterIK,'./Results/IK_sim.gif',1);
JKsim = SimulateIK(BScrews,M,JointAngleIterJK(1:50:end,:),'./Results/JK_sim.gif',1/15);
RKsim = SimulateIK(BScrews,M,JointAngleIterRK,'./Results/RK_sim.gif',1);

save('./Results/InvKinSim.mat','IKsim','JKsim','RKsim')
% View gifs
% web('IK_sim.gif')
% web('JK_sim.gif')
% web('RK_sim.gif')

%% Results comparison
clear;clc;close all;

load('./Results/FKsim.mat');
load('./Results/InvKinSim.mat');
load('./Results/Traj.mat')

% FK comparison-----------------------------------------------------------
% Compare final end-effector position from simulation to final end-effector
% position for measured trajectory

% Traj 1
sim_pos1 = FKsim1{end,end}(1:3,4);
meas_pos1 = [joint_angles1{1}(end,end) joint_angles1{2}(end,end) joint_angles1{3}(end,end)]';

% Traj 2
sim_pos2 = FKsim2{end,end}(1:3,4);
meas_pos2 = [joint_angles2{1}(end,end) joint_angles2{2}(end,end) joint_angles2{3}(end,end)]';

% Traj 3
sim_pos3 = FKsim3{end,end}(1:3,4);
meas_pos3 = [joint_angles3{1}(end,end) joint_angles3{2}(end,end) joint_angles3{3}(end,end)]';

disp('Traj 1 meas vs sim:')
meas_pos1-sim_pos1

disp('Traj 2 meas vs sim:')
meas_pos2-sim_pos2

disp('Traj 3 meas vs sim:')
meas_pos3-sim_pos3


%IK comparison---------------------------------------------------------
% Compare final end-effector pose from simulation vs measured trajectory 1.

% Traj 1
Traj_1_pose = TTheta1{end,end};

% Pseudo-inverse
IK_pose = IKsim{end,end};

% Jacobian transpose
JK_pose = JKsim{end,end};

% Redundancy resolution
RK_pose = RKsim{end,end};

disp(['The end-effector pose for measured trajectory 1 vs. end-effector pose'...
    'for three separate inverse kinematics methods'])
IK_diff = Traj_1_pose-IK_pose
JK_diff = Traj_1_pose-JK_pose
RK_diff = Traj_1_pose-RK_pose