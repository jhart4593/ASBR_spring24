function S = q_sh_h2ScrewAxis(q,sh,h)
%+ Given a unit direction vector (sh), pitch or linear
% velocity along the screw axis to the angular velocity about the screw
% axis ratio (h), and any point along the axis (q) return it's screw axis 
% (S) representation
%+ Reference: Section 3.3.2.2 Modern Robotics Mechanics, Planning and
% Control
%+ Revision List: 
%+ Rev 1.0: Initial Release

qColVec3 = isequal(size(q), [3, 1]); 
qRowVec3 = isequal(size(q), [1, 3]); 

if ~(qColVec3 || qRowVec3)
    error('Error: q must be a 1x3 or 3x1 vector');
elseif qRowVec3
    q = q';
end

shColVec3 = isequal(size(sh), [3, 1]); 
shRowVec3 = isequal(size(sh), [1, 3]); 

if ~(shColVec3|| shRowVec3)
    error('Error: sh must be a 1x3 or 3x1 vector');
elseif shRowVec3
    sh = sh';
end

if numel(h) ~= 1
    error('Error: h is not a scalar value');
end

S = [sh; cross(-sh,q+h*sh)];

omega = S(1:3); 
v = S(4:end); 

normOmega = round(norm(omega),10); 
normV = round(norm(v),10);

if normOmega == 0 && normV ~= 1
    warning('Inputs may be incorrect: S is not normalized, norm(omega) is zero, normalizing with v');
    S = S/normV;
elseif normOmega ~=1
    warning('Inputs may be incorrect: S is not normalized, normalizing with omega');
    S = S/normOmega;
end
end