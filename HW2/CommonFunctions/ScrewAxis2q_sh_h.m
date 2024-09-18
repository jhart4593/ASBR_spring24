function [q,sh,h] = ScrewAxis2q_sh_h(S)

%+ Given a screw S find it's unit direction vector (sh), pitch or linear
% velocity along the screw axis to the angular velocity about the screw
% axis ratio (h), and any point along the axis (q) representation
%+ Reference: Section 3.3.2.2 Modern Robotics Mechanics, Planning and
% Control
%+ Revision List: 
%+ Rev 1.0: Initial Release

if ~(isequal(size(S), [6, 1]))
    error('Error: S must be a 6x1 vector');
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

syms q1 q2 q3
q = [q1;q2;q3];

sh = omega;
h = omega'*v;
q = solve(v-h*sh == cross(-sh,q),q);
if isempty(q.q1)
    q = nan(3,1);
else
    q = double([q.q1;q.q2;q.q3]);
end
    

end