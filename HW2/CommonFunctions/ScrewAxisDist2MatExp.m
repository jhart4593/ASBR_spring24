function eStheta = ScrewAxisDist2MatExp(S,theta)
%+ Given a screw S and a distance theta, find it's matrix exponential
% representation 
%+ Reference: Section 3.3.3 Modern Robotics Mechanics, Planning and
% Control
%+ Revision List: 
%+ Rev 1.0: Initial Release


if ~(isequal(size(S), [6, 1]))
    error('Error: S must be a 6x1 vector');
end

if numel(theta) ~= 1
    error('Error: Theta is not a scalar value');
end

omega = S(1:3); 
v = S(4:end); 

normOmega = round(norm(omega),10); 
normV = round(norm(v),10);


if normOmega == 0 && normV ~= 1
    warning('S is not normalized, norm(omega) is zero, normalizing with v');
    S = S/normV;
elseif normOmega ~=1
    warning('S is not normalized, normalizing with omega');
    S = S/normOmega;
end

omega = S(1:3); 
v = S(4:end); 

omegaSkew = vec2SkewMat(omega); 

ewtheta = AxisAngle2RotMat(omega,theta);
Gtheta = eye(3)*theta+(1-cos(theta))*omegaSkew+(theta-sin(theta))*omegaSkew^2; % Modern Robotics Eqn 3.87

Gv = Gtheta*v; 

eStheta = [ewtheta Gv
    zeros(1,3) 1]; % Modern Robotics Eqn 3.86


% % Test Error Handling
% % Fails screw size check
% S = zeros(1,6);
% theta = 0; 
% eStheta = ScrewAxisDist2MatExp(S,theta);
% 
% % Passes screw size check and gets norm(omega) = 0 normalizing with v
% % warning
% S = zeros(6,1);
% theta = 0; 
% eStheta = ScrewAxisDist2MatExp(S,theta);
% 
% % Non zero omega normalizing warning
% S = [4,42,1,1,2,1]'; 
% theta = 1; 
% eStheta = ScrewAxisDist2MatExp(S,theta);
end