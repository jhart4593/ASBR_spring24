function [X,errorNorm] = HandEyeCalRotQuat(quatA,quatB)
%+ Hand Eye Calibration given two sets of quaterions Where the rows 
% correspond to each quaterion. Solves for X such that AX = XB
%+ Reference: ASBR Lecture Notes W10-2
%+ Revision List: 
%+ Rev 1.0: Initial Release

M = []; 
numPts = size(quatA,1);
for i = 1:numPts
    qAi = quatA(i,:)/norm(quatA(i,:));
    qBi = quatB(i,:)/norm(quatB(i,:));
    sDiff = qAi(1)-qBi(1);
    vDiff = qAi(2:4)-qBi(2:4);
    vSum = qAi(2:4)+qBi(2:4);

    MSub = [sDiff -vDiff
       vDiff'  sDiff*eye(3)+vec2SkewMat(vSum)];
    M(end+1:end+4,:) = MSub;
end

[~,S,V] = svd(M);
q = V*[0;0;0;1];

X = Quat2RotMat(q);

for i = 1:numPts
    A = Quat2RotMat(quatA(i,:)); % Define your matrix A
    B = Quat2RotMat(quatB(i,:)); % Define your matrix B
    errorNorm(i,1) = norm(A*X-X*B);
end

end