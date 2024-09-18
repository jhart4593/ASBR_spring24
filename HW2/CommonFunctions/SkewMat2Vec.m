function vec = SkewMat2Vec(skewMat)
%+ Given a skew matrix return its vector representation
%+ Reference: Modern Robotics Mechanics, Planning and Control
%+ Revision List: 
%+ Rev 1.0: Initial Release

if ~isequal(size(skewMat), [3, 3])
    error('Error: Skew matrix input must be 3x3')
end

if ~issymmetric(round(skewMat,8),"skew")
    error('Error: Matrix is not skew symmetric')
end

vec = [skewMat(2,1); skewMat(1,3); skewMat(3,2)];

end