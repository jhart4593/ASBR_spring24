function skewMat = vec2SkewMat(vec)
%+ Given a vector return its skew matrix representation
%+ Reference: Modern Robotics Mechanics, Planning and Control
%+ Revision List: 
%+ Rev 1.0: Initial Release

if ~(isequal(size(vec), [3, 1]) || isequal(size(vec), [1, 3]))
    error('Error: Vector must be a 1x3 or 3x1 vector');
end

skewMat= [0 -vec(3) vec(2); 
    vec(3) 0 -vec(1); 
    -vec(2) vec(1) 0];  % Modern Robotics Eqn 3.85


% % Test code
% vec = rand(1,3); 
% skewMat = vec2SkewMat(vec); 
% 
% issymmetric(skewMat,"skew")

end