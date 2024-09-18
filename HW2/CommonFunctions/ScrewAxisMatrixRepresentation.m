function SMat = ScrewAxisMatrixRepresentation(S)
%+ Get matrix representation of a screw or a twist vector
%+ Reference: Section 3.3.2.2 Modern Robotics Mechanics, Planning and
% Control (Equation 3.85)
%+ Revision List: 
%+ Rev 1.0: Initial Release

if ~(isequal(size(S), [6, 1]))
    error('Error: S must be a 6x1 vector');
end

omega = S(1:3); 
v = S(4:end); 

omegaSkew = vec2SkewMat(omega); 

SMat = [omegaSkew v
    zeros(1,4)];
end