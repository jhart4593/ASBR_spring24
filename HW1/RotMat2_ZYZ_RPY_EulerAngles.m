function [ZYZEuler, RollPitchYawEuler] = RotMat2_ZYZ_RPY_EulerAngles(RotMat)
%+ Given a rotation matrix, find it's ZYZ and Roll/Pitch/Yaw Euler angle (radians) representation
%+ Reference: Section 2.3 A Mathematical Introduction to Robotic Manipulation
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

roll = atan2(RotMat(3,2),RotMat(3,3));
pitch = atan2(-RotMat(3,1),sqrt(RotMat(1,1)^2+RotMat(2,1)^2));
yaw = atan2(RotMat(2,1),RotMat(1,1));

if abs(round(RotMat(1,1),10)) == 0 && abs(round(RotMat(2,1),10)) == 0
    warning('Pitch angle is close to +/- pi/2, a singularity for ZYX Euler angles')
end

alpha = atan2(RotMat(2,3),RotMat(1,3));
beta = atan2(sqrt(RotMat(3,1)^2+RotMat(3,2)^2),RotMat(3,3));
gamma = atan2(RotMat(3,2),-RotMat(3,1));

if trace(RotMat) == 3 && sum(sum(RotMat)) == 3 
    warning('Rotation Matrix is an identify matrix, a singularity for ZYZ Euler angles')
end


ZYZEuler = [alpha,beta,gamma];
RollPitchYawEuler = [roll, pitch, yaw];


% Setup
Rx = @(ang) [1 0 0; 
    0 cos(ang) -sin(ang)
    0 sin(ang) cos(ang)]; 

Ry = @(ang) [cos(ang) 0 sin(ang); 
    0 1 0; 
    -sin(ang) 0 cos(ang)];

Rz = @(ang) [cos(ang) -sin(ang) 0
sin(ang) cos(ang) 0 
0 0 1];

% Derivation
syms alpha beta gamma roll pitch yaw real

RZYX = Rz(yaw)*Ry(pitch)*Rx(roll);

yawExp = simplify(RZYX(2,1)/RZYX(1,1),'steps',3)
rollExp = simplify(RZYX(3,2)/RZYX(3,3),'steps',3)
pitchExp = simplify(-RZYX(3,1)/sqrt(RZYX(1,1)^2+RZYX(2,1)^2),'steps',100)


RZYZ = Rz(alpha)*Ry(beta)*Rz(gamma);

alphaExp = simplify(RZYZ(2,3)/RZYZ(1,3),'steps',3)
betaExp = simplify(sqrt(RZYZ(3,1)^2+RZYZ(3,2)^2)/RZYZ(3,3),'steps',100)
gammaExp = simplify(RZYZ(3,2)/-RZYZ(3,1),'steps',3)

% % Test Code
% % Test Error Condition 1 (matrix size check) 
% RotMat = zeros(4,3); 
% [ZYZEuler, RollPitchYawEuler]  = RotMat2_ZYZ_RPY_EulerAngles(RotMat)
% 
% % Test Error Condition 2 (orthogonal)
% RotMat = zeros(3,3);
% [ZYZEuler, RollPitchYawEuler]  = RotMat2_ZYZ_RPY_EulerAngles(RotMat)
% 
% % Test Error Condition 2 (preserves orientation)
% RotMat = -eye(3);
% [ZYZEuler, RollPitchYawEuler]  = RotMat2_ZYZ_RPY_EulerAngles(RotMat)
% 
% % Test Non Singularity
% ang1 = -.21442; 
% ang2 = .38713; 
% ang3 = 1.4217; 
% 
% RotMat = Rz(ang1)*Ry(ang2)*Rz(ang3);
% [ZYZEuler,~]  = RotMat2_ZYZ_RPY_EulerAngles(RotMat);
% 
% isequal(round(ZYZEuler,10),round([ang1 ang2 ang3],10))
% 
% RotMat = Rz(ang3)*Ry(ang2)*Rx(ang1);
% [~,RollPitchYawEuler]  = RotMat2_ZYZ_RPY_EulerAngles(RotMat);
% 
% isequal(round(RollPitchYawEuler,10),round([ang1 ang2 ang3],10))
% 
% % Test Singularity (ZYZ)
% RotMat = eye(3);
% [ZYZEuler,~]  = RotMat2_ZYZ_RPY_EulerAngles(RotMat);
% 
% % Test Singularity (ZYX)
% ang1 = -1.71384; 
% ang2 = pi/2; 
% ang3 = -0.4217; 
% 
% RotMat = Rz(ang3)*Ry(ang2)*Rx(ang1);
% [~,RollPitchYawEuler]  = RotMat2_ZYZ_RPY_EulerAngles(RotMat);
% 
% isequal(round(RollPitchYawEuler,10),round([ang1 ang2 ang3],10))
end