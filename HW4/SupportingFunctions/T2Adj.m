function Adj = T2Adj(T)
%+ Get the adjoint representation of a transformation matrix
%+ Reference: Section 3.3.2 Modern Robotics Mechanics, Planning and
% Control (Definition 3.20)
%+ Revision List: 
%+ Rev 1.0: Initial Release

if ~isequal(size(T), [4, 4])
    error('Error: Transformation matrix must be 4x4')
end

RotMat = T(1:3,1:3);

if ~isa(RotMat,'sym')
    detRot = double(round(det(RotMat),8)); % accounts for numerical inprecision 
    if abs(detRot) ~= 1
        error('Error: Rotation matrix is not orthogonal')
    elseif detRot ~= 1
        error('Error: Rotation matrix does not preserve orientation')
    end
end

pSkew = vec2SkewMat(T(1:3,4));

Adj = [RotMat zeros(3,3)
    pSkew*RotMat RotMat];

end