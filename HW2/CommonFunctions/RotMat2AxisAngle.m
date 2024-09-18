function [axis, angle] = RotMat2AxisAngle(RotMat)
%+ Given a rotation matrix, find it's axis and angle (radians) representation
%+ Reference: Section 3.2.3.3 Modern Robotics Mechanics, Planning and
% Control
%+ Revision List: 
%+ Rev 1.0: Initial Release

if ~isequal(size(RotMat), [3, 3])
    error('Error: Rotation matrix must be 3x3')
end

 
detRot = round(det(RotMat),8); % accounts for numerical inprecision 
if abs(detRot) ~= 1
    error('Error: Rotation matrix is not orthogonal')
elseif detRot ~= 1
    error('Error: Rotation matrix does not preserve orientation')
end

traceR = trace(RotMat); 

if traceR == 3 && sum(sum(RotMat)) == 3 % Rot = eye(3), theta = 0, w^ is undefined
    angle = 0; 
    axis = nan(3,1); 
elseif traceR == -1 % theta = pi
    angle = pi; 
    if 1+RotMat(3,3) ~= 0
        axis = 1/sqrt(2*(1+RotMat(3,3)))*[RotMat(1,3); RotMat(2,3); 1+RotMat(3,3)]; % Eqn 3.58
    elseif 1+RotMat(2,2) ~= 0
        axis = 1/sqrt(2*(1+RotMat(2,2)))*[RotMat(1,2); 1+RotMat(2,2); RotMat(3,2)]; % Eqn 3.59
    else
        axis = 1/sqrt(2*(1+RotMat(1,1)))*[1+RotMat(1,1); RotMat(2,1); RotMat(3,1)]; % Eqn 3.60
    end
else
    angle = acos(.5*(traceR-1)); % Eqn 3.54
    axisSkew = 1/(2*sin(angle))*(RotMat-RotMat'); % Eqn 3.53
    axis = [axisSkew(3,2); axisSkew(1,3); axisSkew(2,1)];
end

% % Test Code
% % Test Error Condition 1 (matrix size check) 
% RotMat = zeros(4,3); 
% [axis, angle] = RotMat2AxisAngle(RotMat)
% 
% % Test Error Condition 2 (orthogonal)
% RotMat = zeros(3,3);
% [axis, angle] = RotMat2AxisAngle(RotMat)
% 
% % Test Error Condition 2 (preserves orientation)
% RotMat = -eye(3);
% [axis, angle] = RotMat2AxisAngle(RotMat)
% 
% % First Case
% RotMat = eye(3);
% [axis, angle] = RotMat2AxisAngle(RotMat)
% 
% % 2nd Case (x rotation)
% phi = pi;
% Rx = [1 0 0; 
%     0 cos(phi) -sin(phi)
%     0 sin(phi) cos(phi)]; 
% [axis, angle] = RotMat2AxisAngle(Rx)
% 
% % 2nd Case (y rotation)
% theta = pi; 
% Ry = [cos(theta) 0 sin(theta); 
%     0 1 0; 
%     -sin(theta) 0 cos(theta)];
% [axis, angle] = RotMat2AxisAngle(Ry)
% 
% % 2nd Case (z rotation)
% psi = pi;
% Rz = [cos(psi) -sin(psi) 0
% sin(psi) cos(psi) 0 
% 0 0 1];
% [axis, angle] = RotMat2AxisAngle(Rz)
% 
% % 3rd Case (arbitrary none pi rotation)
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
% 
% [axis, angle] = RotMat2AxisAngle(Rx*Rz*Ry)


end