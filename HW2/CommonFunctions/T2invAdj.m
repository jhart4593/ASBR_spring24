function invAdj = T2invAdj(T)
%+ Get the adjoint inverse representation of a transformation matrix
%+ Reference: Section 3.3.2.1 Modern Robotics Mechanics, Planning and
% Control (Equation 3.84)
%+ Revision List: 
%+ Rev 1.0: Initial Release

if ~isequal(size(T), [4, 4])
    error('Error: Transformation matrix must be 4x4')
end

RotMat = T(1:3,1:3);

detRot = round(det(RotMat),8); % accounts for numerical inprecision 
if abs(detRot) ~= 1
    error('Error: Rotation matrix is not orthogonal')
elseif detRot ~= 1
    error('Error: Rotation matrix does not preserve orientation')
end
pSkew = vec2SkewMat(T(1:3,4));

RotMatT = RotMat';

invAdj = [RotMatT zeros(3,3)
    -RotMatT*pSkew RotMatT];

end