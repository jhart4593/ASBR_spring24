function [SScrews, BScrews]= getScrews(poses)
%+ Gets screws from poses
%+ Reference: Section 3.3 Modern Robotics Mechanics, Planning and
% Control
%+ Revision List: 
%+ Rev 1.0: Initial Release
numPose = numel(poses); 

Bscrews = cell(numPose,numPose);

for i = 1:numel(poses)
    poseTemp = poses{i}; 
    omega = poseTemp(1:3,3); 

    q = poseTemp(1:3,4); 

    v = cross(-omega,q); 

    SScrews{i} = [omega;v];

    M = poses{i}; 
    AdjinvM = T2invAdj(M);
    BScrews(i,1:i) = cellfun(@(S) AdjinvM * S, SScrews(1:i), 'UniformOutput', false); % Eqn 3.85
end


end