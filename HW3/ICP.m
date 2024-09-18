function T = ICP(aSet,bSet,corrFun,errorThresh,maxIter)
%+ Iterative Closest Point algorithm to find the transformation
% between 3D point set A and B such that R*a+p = b, iteratively given a
% coorespondence function, an error threshold and a max amount of
% iterations
%+ Reference: ASBR W10-L1 notes
%+ Revision List: 
%+ Rev 1.0: Initial Release


numPts = size(aSet); 
numPts = numPts(numPts~=3);

iter = 1; 

aOrig = aSet;
T = eye(4);
errorK1 = Inf; 
while iter <= maxIter
    T = corrFun(aSet,bSet)*T;

    for i = 1:numPts
        aSetTemp = T*[aOrig(i,:)'; 1];
        aSet(i,:) = aSetTemp(1:3)';
    end
    errorK0 = sum(vecnorm(aSet-bSet,2,2));
    errorRat = errorK1/errorK0; 

    iter = iter+1;
%     if errorK0 <= errorThresh && .95 <= errorRat && errorRat <= 1
%         break
%     end
    if errorK0 <= errorThresh
        break
    end

    errorK1 = errorK0;
end
if iter == maxIter+1
    warning('Could not converge, max iterations reached');
end

% % Test Code
% rng(1000)
% 
% bPt = rand(8,3);
% 
% RotMat = [0.1107    0.9904   -0.0830
%    -0.8646    0.1371    0.4833
%     0.4901    0.0182    0.8715];
% p = [12; 1.345;-6];
% TKnown = [RotMat p
%     zeros(1,3) 1];
% 
% for i = 1:size(bPt,1)
%     aPtTemp = TKnown*[bPt(i,:)'; 1]; 
%     aPt(i,:) = aPtTemp(1:3)'; 
% end
% 
% errorThresh = .001; 
% maxIter = 10000; 
% T = ICP(aPt,bPt,@svdCorrespondence,errorThresh,maxIter);
% 
% for i = 1:size(bPt,1)
%     bPtTemp = T*[aPt(i,:)'; 1]; 
%     bPtCalc(i,:) = bPtTemp(1:3)'; 
% end
% error = bPt-bPtCalc

end