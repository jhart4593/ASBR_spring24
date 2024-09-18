function [X,errorNorm] = HandEyeCalRotKronProd(quatA,quatB)
%+ Hand Eye Calibration given two sets of quaterions Where the rows 
% correspond to each quaterion. Solves for X such that AX = XB
%+ Reference: (Matt J) https://www.mathworks.com/matlabcentral/answers/1900510-way-to-solve-ax-xb
% https://en.wikipedia.org/wiki/Sylvester_equation
%+ Revision List: 
%+ Rev 1.0: Initial Release

C = []; 
numPts = size(quatA,1); 
for i = 1:numPts
    A = Quat2RotMat(quatA(i,:)); % Define your matrix A
    B = Quat2RotMat(quatB(i,:)); % Define your matrix B
    C(end+1:end+9,:) = kron(eye(3),A) - kron(B',eye(3));
end

[U, S, V] = svd(C);
nullBasis = V*[zeros(8,1);1];
X = reshape(nullBasis, [3,3]);

for i = 1:numPts
    A = Quat2RotMat(quatA(i,:)); % Define your matrix A
    B = Quat2RotMat(quatB(i,:)); % Define your matrix B
    errorNorm(i,1) = norm(A*X-X*B);
end

end