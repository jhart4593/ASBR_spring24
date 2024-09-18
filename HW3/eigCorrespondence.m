function  T = eigCorrespondence(aSet,bSet)

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

delta = [H(2,3)-H(3,2); H(3,1)-H(1,3); H(1,2)-H(2,1)];
G = [trace(H) delta'
    delta H+H'-trace(H)*eye(3)];

[V,D] = eig(G);
[~,ind] = sort(diag(D),'descend'); 


q = V*(ind == 1);

R = Quat2RotMat(q); 
p = bCent'-R*aCent';

T = [R p
    zeros(1,3) 1];
end