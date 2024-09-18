addpath('./SupportingFunctions/')

load('HarmonyScrews.mat')
side = 'R'; 

% Set Initial Position and Goal
q0 = [-0 -5 -5 10 5 -90 0]*pi/180; % Arbitrary angle instance, Can set to any angles
q0 = getVirtualTheta(q0);
q0 = q0(3:end);

dq0 = zeros(size(q0));

toolTipGoal = [.25; -.37; .7];
% toolTipGoal = [.18; -.28; .6];
% toolTipGoal = [.5; -.2; .5];

% Problem Requirements
mm2m = 1e-3;
toolLength = 100*mm2m; % m
radErr = 3*mm2m; 

% Get Joint Limits
for i = 1:7
    bnds = getJointLimits(i,side);
    lowerJointLimit7Dof(i,1) = bnds(1); 
    upperJointLimit7Dof(i,1) = bnds(2);
end
lowerJointLimit = [lowerJointLimit7Dof(1:2); -upperJointLimit7Dof(2); lowerJointLimit7Dof(3:end)];
upperJointLimit = [upperJointLimit7Dof(1:2); -lowerJointLimit7Dof(2); upperJointLimit7Dof(3:end)];
lb = lowerJointLimit-q0; 
ub  = upperJointLimit-q0;

% Define Objective Functions
[toolTip0, toolAxis0] = calculateToolTip(SScrews(3:end),RobotFrameZero.pose{end},toolLength,q0);
JacobianSpace = J_space(SScrews(3:end),q0);
alpha = @(dq) JacobianSpace(1:3,:)*dq;
eps = @(dq) JacobianSpace(4:6,:)*dq;


ptObj = @(dq) norm(vec2SkewMat(alpha(dq))*toolTip0+eps(dq)+toolTip0-toolTipGoal);
orienObj = @(dq) norm(vec2SkewMat(alpha(dq))*toolAxis0);

% Define Constraints
Aeq = [0 1 1 zeros(1,5)]; % Special constraint for harmony q2 = -q3
beq = 0;
A = []; 
b = [];
nonlcon = @(dq) nonlconPt(ptObj,radErr,dq);

% Run fmincon (Point Objective)
[dqPt,fvalPt,historyPt] = runfmincon(ptObj,dq0,A,b,Aeq,beq,lb,ub,nonlcon);
[toolTipPtf,toolAxisPtf] = calculateToolTip(SScrews(3:end),RobotFrameZero.pose{end},toolLength,q0+dqPt);
norm(toolTipPtf-toolTipGoal)/mm2m
norm(toolAxisPtf-toolAxis0)

% Run fmincon (Point Objective+Orientation Objective)
w1 = .7; 
w2 = .3; 
obj = @(dq) w1*ptObj(dq)+w2*orienObj(dq);
[dqPtOrien,fvalPtOrien,historyPtOrien] = runfmincon(obj,dq0,A,b,Aeq,beq,lb,ub,nonlcon);
[toolTipPtOrienf,toolAxisPtOrienf]  = calculateToolTip(SScrews(3:end),RobotFrameZero.pose{end},toolLength,q0+dqPtOrien);
norm(toolTipPtOrienf-toolTipGoal)/mm2m
norm(toolAxisPtOrienf-toolAxis0)

%% Generate plots
load('PlotData.mat')
plotOptResults(infoCell,labels)
% labels.yAxisJoint = 'Joint Angle (deg)';
% labels.xAxisJoint = 'Iteration';
% labels.legend ={'JL+Goal (Close)', 'JL+Goal+Orien (Close)', 'JL+Goal (Mid)', 'JL+Goal+Orien (Mid)','JL+Goal (Far)', 'JL+Goal+Orien (Far)'};
% infoPt = historyPt; 
% infoPt.x = (infoPt.x+q0')*180/pi; 
% infoPt.SScrews = SScrews(3:end); 
% infoPt.M = RobotFrameZero.pose{end};
% infoPt.toolLength = toolLength; 
% infoPt.toolTipGoal = toolTipGoal; 
% infoPt.initalOrien = toolAxis0; 
% 
% 
% infoPtOrien = historyPtOrien; 
% infoPtOrien.x = (infoPtOrien.x+q0')*180/pi; 
% infoPtOrien.SScrews = SScrews(3:end); 
% infoPtOrien.M = RobotFrameZero.pose{end};
% infoPtOrien.toolLength = toolLength; 
% infoPtOrien.toolTipGoal = toolTipGoal; 
% infoPtOrien.initalOrien = toolAxis0; 

% Functions 
function [c,ceq] = nonlconPt(ptObj,radErr,dq)
    c = ptObj(dq)-radErr;
    ceq =[];
end