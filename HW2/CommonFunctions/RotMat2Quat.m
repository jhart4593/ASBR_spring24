function [Quat] = RotMat2Quat(RotMat)
%+ Given a rotation matrix, find it's quaternion representation
%+ Reference: ASBR Week 3 Lecture 1 Notes
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

R = RotMat; 
Quat = zeros(4,1); 

Quat(1) = .5*sqrt(trace(R)+1); 
Quat(2) = .5*sign(R(3,2)-R(2,3))*sqrt(R(1,1)-R(2,2)-R(3,3)+1); 
Quat(3) = .5*sign(R(1,3)-R(3,1))*sqrt(-R(1,1)+R(2,2)-R(3,3)+1);
Quat(4) = .5*sign(R(2,1)-R(1,2))*sqrt(-R(1,1)-R(2,2)+R(3,3)+1);

% % Test Error Condition 1 (matrix size check) 
% RotMat = zeros(4,3); 
% [Quat] = RotMat2Quat(RotMat);
% 
% % Test Error Condition 2 (orthogonal)
% RotMat = zeros(3,3);
% [Quat] = RotMat2Quat(RotMat);
% 
% % Test Error Condition 2 (preserves orientation)
% RotMat = -eye(3);
% [Quat] = RotMat2Quat(RotMat);
% 
% % Test Computation
% theta = rand(1); % angle
% w = rand(3,1); % axis
% 
% w = w/norm(w);
% w1 = w(1); 
% w2 = w(2); 
% w3 = w(3);
% 
% vt = 1-cos(theta); 
% st = sin(theta); 
% ct = cos(theta); 
% 
% ewht = [w1^2*vt+ct w1*w2*vt-w3*st w1*w3*vt+w2*st 
%     w1*w2*vt+w3*st w2^2*vt+ct w2*w3*vt-w1*st
%     w1*w3*vt-w2*st w2*w3*vt+w1*st w3^2*vt+ct]; % Rotation matrix angle/axis
% 
% Q = [cos(theta/2); w*sin(theta/2)]; % Quaterion from axis angle
% 
% [Quat] = RotMat2Quat(ewht);
% isequal(round(Q,8),round(Quat,8))
end