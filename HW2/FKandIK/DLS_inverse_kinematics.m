function [endEffectorPos, mags, VbIter, JointAngleIter] = DLS_inverse_kinematics(BScrews,M,initialAngle,desiredAngle)
%+ Given the body screws of the serial chain robot, the zero configuration,
% an initial joint angle guess, a desired joint angle configuration
% compute the inverse kinematics numerically using the damped least squares
% method
%+ Returns:
% Matrix of end effector cartesian points in fixed frame (per iteration)
% Magnitude [norm(omega) norm(velocity)] (per iteration)
% Vb (per iteration)
% Joint angles (per iteration)
%+ Reference: Buss, S.R., 2004 Introduction to inverse kinematics with
% Jacobian transpose, pseudoinverse and damped least squares methods
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
 
lamda = .1;
Tsd = FK_body(BScrews,M,desiredAngle); % compute final end effector pose


while magW > ew || magV > ev

    Jb = J_body(BScrews,theta); % get jacobian 
    
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

    theta = theta+Jb'/(Jb*Jb'+lamda^2*eye(6))*Vb;
    i = i+1;

    if i > maxIter
        warning('Exceeds max iterations, could not coverge');
        break
    end
end

end