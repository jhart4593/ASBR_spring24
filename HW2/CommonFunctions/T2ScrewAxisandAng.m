function [S, theta] = T2ScrewAxisandAng(T)
%+ Given a transformation matrix (T), find it's screw axis (S) and distance
% (theta) representation 
%+ Reference: Section 3.3 Modern Robotics Mechanics, Planning and
% Control
%+ Revision List: 
%+ Rev 1.0: Initial Release
    
if ~isequal(size(T), [4, 4])
    error('Error: Transformation matrix must be 4x4')
end

RotMat = T(1:3,1:3);

[omega, theta] = RotMat2AxisAngle(RotMat);

omegaSkew = vec2SkewMat(omega); 

Ginv = 1/theta*eye(3) -.5*omegaSkew+(1/theta-.5*cot(theta/2))*omegaSkew^2;  % Modern Robotics Eqn 3.92
v = Ginv*T(1:3,4);  % Modern Robotics Eqn 3.91

S = [omega;v];
if norm(omega) == 0
    S = S/norm(v);
else
    S = S/norm(omega);
end

end