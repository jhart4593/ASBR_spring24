function T = quatCorrespondence(aSet,bSet)
%+ Quaterion Correspondence method to find the transformation matrix
% between 3D point set A and B such that R*a+p = b
%+ Reference: ASBR W10-L1 notes
%+ Revision List: 
%+ Rev 1.0: Initial Release

numPts = size(aSet); 
numPts = numPts(numPts~=3);

aCent = mean(aSet);
bCent = mean(bSet);

aT = aSet-aCent; 
bT = bSet-bCent; 

M = []; 
for i = 1:numPts
    aTi = aT(i,:); 
    bTi = bT(i,:); 
    MSub = [0 bTi-aTi;
        (bTi-aTi)' vec2SkewMat(bTi+aTi)];
    M(end+1:end+4,:) = MSub;
end

[~,~,V] = svd(M);

q = V*[0;0;0;1];

R = Quat2RotMat(q); 

p = bCent'-R*aCent';

T = [R p
    zeros(1,3) 1];

end