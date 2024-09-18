function [px, errorNorm] = LeastSquaresTranslation(qA,pA,pB,Rx)
%+ Least squares solution for Hand Eye Calibration translation vector
% Given A set quaterions, A set translation, B set translations 
% Where the rows correspond to each data point. And Rx where A*Rx = Rx*B
% Solves for px such that (R_a,k - eye())px = Rx*p_B,k -p_a,k
%+ Reference: ASBR Lecture Notes W10-2
%+ Revision List: 
%+ Rev 1.0: Initial Release

A = [];
b = []; 
numPts = size(qA,1);
for i = 1:numPts
    A(end+1:end+3,:) = Quat2RotMat(qA(i,:))-eye(3);
    b(end+1:end+3,:) = Rx*pB(i,:)'-pA(i,:)';
end

% Least Squares Solution
[U,S,V] = svd(A); % A = U*S*V'
S = round(S,10);
Srecp = S; 
Srecp(S~=0) = 1./S(S~=0);

px = V*Srecp'*U'*b;
% isequal(round(V*Srecp'*U',9),round(pinv(A),9))

for i = 1:numPts
    Ra = Quat2RotMat(qA(i,:)); 
    errorNorm(i,1) = norm((Ra-eye(3))*px - Rx*pB(i,:)'+pA(i,:)'); 
    
end

end