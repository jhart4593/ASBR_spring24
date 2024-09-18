function [RotMat] = Quat2RotMat(Quat)
%+ Given a quaternion, find it's rotation matrix representation
%+ Reference: ASBR W3-L1 notes or https://en.wikipedia.org/wiki/Quaternions
% +_and_spatial_rotation
%+ Revision List: 
%+ Rev 1.0: Initial Release

if ~(isequal(size(Quat), [4, 1]) || isequal(size(Quat), [1, 4]))
    error('Error: Quat must be a 1x4 or 4x1 vector');
end
q = Quat; 

% slide 3
q = q/norm(q); % quaternion must be unit quaternion
% (note norm of a vector is same as taking quat norm, it can be shown
% Q*QConj = ||Q||^2 = q0^2+q1^2+q2^2+q3^2; therefore ||Q|| = vecnorm(Q)

% slide 5
q0 = q(1); 
q1 = q(2); 
q2 = q(3); 
q3 = q(4); 
RotMat = [q0^2+q1^2-q2^2-q3^2,  2*(q1*q2-q3*q0),    2*(q1*q3+q2*q0)
          2*(q1*q2+q3*q0),  q0^2-q1^2+q2^2-q3^2,    2*(q2*q3-q1*q0)
          2*(q1*q3-q2*q0),  2*(q2*q3+q1*q0),    q0^2-q1^2-q2^2+q3^2];

% % Test Code
% % Fails quat size check
% Q = zeros(3,2);
% [RotMat] = Quat2RotMat(Q);
% 
% % Passes quat size check variation 1
% Q = zeros(1,4);
% [RotMat] = Quat2RotMat(Q);
% 
% % Passes quat size check variation 2
% Q = zeros(4,1);
% [RotMat] = Quat2RotMat(Q);

% Test Computation
% theta = rand(1); % angle
% 
% w = rand(3,1); % axis
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
% [RotMat] = Quat2RotMat(Q);
% isequal(round(ewht,8),round(RotMat,8))
end