function [endEffectorPos, mags, VbIter, JointAngleIter,objIter] = redundancy_resolution(BScrews,M,initialAngle,desiredAngle,obj,k,varargin)
%+ Given the body screws of the serial chain robot, the zero configuration,
% an initial joint angle guess, a desired joint angle configuration, a
% selected objective function, and a gain
% compute the inverse kinematics numerically using a null space objective
% function
%+ Returns:
% Matrix of end effector cartesian points in fixed frame (per iteration)
% Magnitude [norm(omega) norm(velocity)] (per iteration)
% Vb (per iteration)
% Joint angles (per iteration)
% Objective value (per iteration)
%+ Reference: Section 6.2.2 Modern Robotics Mechanics, Planning and
% Control and W8-L1 notes
%+ Revision List: 
%+ Rev 1.0: Initial Release


% Define variables 
ew = .001; % error margin for omega norm
ev = 1e-4; % error margin for velocity norm
theta = initialAngle; 
i = 1; % iteration counter
magW = 1; % preset magnitude of omega
magV = 1; % preset magnitude of velocity
origin = [0;0;0;1]; % origin in body frame
maxIter = 10000; % max allowable iterations

h = 1e-5; % finite difference step size

n = numel(BScrews);
Tsd = FK_body(BScrews,M,desiredAngle); % compute final end effector pose
switch obj
    case 1
        objFun = @(theta) manipulabilityMeasure(BScrews,theta);
        grad_f = @(theta) numGradient(objFun, theta, h);
    case 2
        objFun = @(theta) manipulabilityMeasure(BScrews,theta);
        grad_f = @(theta) J_manipulability(J_body(BScrews,theta));

    case 3
        upperBnd = varargin{1};
        lowerBnd = varargin{2}; 
        objFun = @(theta) jointLimitObj(theta,upperBnd,lowerBnd);
        grad_f = @(theta) numGradient(objFun,theta,h);

    case 4
        RobotFrameZero = varargin{1};
        obstaclePos = varargin{2};
        objFun = @(theta) obsAvoidObj(theta,RobotFrameZero,obstaclePos);
        grad_f = @(theta) numGradient(objFun,theta,h);
end

while magW > ew || magV > ev

    JacobianBody = J_body(BScrews,theta); % get jacobian 
    Jbp = pinv(JacobianBody);  % pseudo inverse 
    
    Tsb = FK_body(BScrews,M,theta); % compute current end effector pose
   
    Tbd = Tinv(Tsb)*Tsd; 
    [S, ang] = T2ScrewAxisandAng(Tbd); 
    Vb = S*ang; % Vb
    
    magW = norm(Vb(1:3)); % update error check 
    magV = norm(Vb(4:6));


    cartesian = Tsb*origin; % compute end effector pos

    endEffectorPos(i,:) = cartesian(1:3)'; % store variables
    mags(i,:) = [magW, magV]; 
    VbIter(i,:) = Vb';
    JointAngleIter(i,:) = theta; 
    objIter(i,1) = objFun(theta);
    theta = theta + Jbp*Vb + k*(eye(n)-Jbp*JacobianBody)*grad_f(theta);
    i = i+1;

    if i > maxIter
        warning('Exceeds max iterations, could not coverge');
        break
    end
end

function grad_f = numGradient(f, q0, h)
    grad_f = zeros(size(q0));
    for j = 1:length(q0)
        q1 = q0;
        q1(j) = q1(j) + h;
        grad_f(j) = (f(q1) - f(q0)) / h;
    end
end

end