function T = svdCorrespondence(aSet,bSet)
%+ SVD Correspondence method to find the transformation matrix
% between 3D point set A and B such that R*a+p = b
%+ Reference: ASBR W8-L2 notes
%+ Revision List: 
%+ Rev 1.0: Initial Release

numPts = size(aSet); 
numPts = numPts(numPts~=3);

aCent = mean(aSet);
bCent = mean(bSet);

aT = aSet-aCent; 
bT = bSet-bCent; 

H = zeros(3,3); 
for i = 1:numPts
    aTi = aT(i,:); 
    bTi = bT(i,:); 
    HSub = [aTi(1)*bTi(1) aTi(1)*bTi(2) aTi(1)*bTi(3)
        aTi(2)*bTi(1) aTi(2)*bTi(2) aTi(2)*bTi(3)
        aTi(3)*bTi(1) aTi(3)*bTi(2) aTi(3)*bTi(3)];
    H = H+HSub; 
end

[U,~,V] = svd(H);

R = V*U'; 

if round(det(R),8)~= 1
    warning('Determinant of R is not equal to 1, algorithm may fail')
end

p = bCent'-R*aCent';

T = [R p
    zeros(1,3) 1];
end