
inc = 7*pi/180; 

jointAngles = cell(1, 7);
for i = 1:7
    bnds = getJointLimits(i,'R'); 
    jointAngles{i} = [bnds(1):inc:bnds(2) bnds(2)];
end

[j1, j2, j3, j4, j5, j6, j7] = ndgrid(jointAngles{:});


jointSweep = [j1(:), j2(:), j3(:), j4(:), j5(:), j6(:), j7(:)];
%%
SignVals = zeros(size(jointSweep,1),1);

tic 
parfor i = 1:size(SignVals,1)
    angleInstance = getVirtualTheta(jointSweep(i,:));
    JacobianSpace = J_space(Screws,angleInstance);
    SingVals(i,1) = min(svd(JacobianSpace));
    
end
toc

save('C:\Users\alexa\OneDrive\Desktop\ASBRData\CoarseSweep.mat', 'jointSweep','SingVals','Screws','-v7.3')

%%
minThresh = .01; 

SingValsClose = SingVals <= minThresh; 
index = 1:numel(SingVals);
closeIndx = index(SingValsClose);
val = jointSweep(closeIndx,:);
oneUp = jointSweep(closeIndx+1,:);
oneDown = jointSweep(closeIndx-1,:);

diffUp = oneUp-val; 
diffDown = val-oneDown; 


numDiffUp = sum(diffUp~=0,2);
numDiffDown = sum(diffDown~=0,2);
numJoints = 1:7; 

bnds = zeros(numel(closeIndx),3);
for i = 1:numel(closeIndx)
    up = numDiffUp(i); 
    down = numDiffDown(i); 

    if up == 1 && down == 1
        jointIndx1 = numJoints(diffUp(i,:)~=0);
        jointIndx2 = numJoints(diffDown(i,:)~=0);
        if jointIndx1 ~= jointIndx2
            bnds(i,:) = nan(1,3); 
        else
            bnds(i,:) = [jointIndx1, oneDown(i,jointIndx), oneUp(i,jointIndx)];
        end
    elseif up == 1
         jointIndx = numJoints(diffUp(i,:)~=0); 
         bnds(i,:) = [jointIndx, val(i,jointIndx), oneUp(i,jointIndx)];
    elseif down == 1
        jointIndx = numJoints(diffDown(i,:)~=0);
        bnds(i,:) = [jointIndx, oneDown(i,jointIndx), val(i,jointIndx)];
    end

end