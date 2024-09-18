function [RotMat] = AxisAngle2RotMat(axis,angle)
%+ Applies Rodrigues' formula to compute the rotation matrix from an input
% rotation axis and angle (radians)
%+ Reference: Section 3.2.3.2 Modern Robotics Mechanics, Planning and
% Control
%+ Revision List: 
%+ Rev 1.0: Initial Release

if ~(isequal(size(axis), [3, 1]) || isequal(size(axis), [1, 3]))
    error('Error: Axis must be a 1x3 or 3x1 vector');
end

if numel(angle) ~= 1
    error('Error: Angle is a scalar value');
end

w = axis; 

wSkew = [0 -w(3) w(2)
    w(3) 0 -w(1)
    -w(2) w(1) 0]; % Eqn 3.30

RotMat = eye(3) + sin(angle)*wSkew + (1-cos(angle))*wSkew^2; % Eqn 3.51

% % Test Code

% % Fails axis size check
% axis = zeros(3,2);
% angle = 0;
% [RotMat] = AxisAngle2RotMat(axis,angle);
% 
% % Passes axis size check variation 1
% axis = zeros(1,3);
% angle = 0;
% [RotMat] = AxisAngle2RotMat(axis,angle);
% 
% % Passes axis size check variation 2
% axis = zeros(3,1);
% angle = 0;
% [RotMat] = AxisAngle2RotMat(axis,angle);
% 
% % Fails angle size check
% axis = zeros(3,1);
% angle = [2 3];
% [RotMat] = AxisAngle2RotMat(axis,angle);

% phi = .132;
% theta = -.643; 
% psi = -1.432;
% 
% Rx = [1 0 0; 
%     0 cos(phi) -sin(phi)
%     0 sin(phi) cos(phi)]; 
% 
% Ry = [cos(theta) 0 sin(theta); 
%     0 1 0; 
%     -sin(theta) 0 cos(theta)];
% 
% Rz = [cos(psi) -sin(psi) 0
% sin(psi) cos(psi) 0 
% 0 0 1];
% RotMatIn = Rx*Rz*Ry
% [axis, angle] = RotMat2AxisAngle(RotMatIn); 
% [RotMat] = AxisAngle2RotMat(axis,angle)
% isequal(round(RotMatIn,6), round(RotMat,6))

end