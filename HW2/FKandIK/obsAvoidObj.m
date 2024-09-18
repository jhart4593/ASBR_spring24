function w = obsAvoidObj(theta,RobotFrameZero,obstaclePos)
[SScrews, ~] = getScrews(RobotFrameZero.pose);
% Compute forward Kinematics
for i = 1:numel(SScrews)
    thetaSub = theta(1:i);
    SScrewsSub = SScrews(1:i); 
    M = RobotFrameZero.pose{i};
    TTheta{i,1} = FK_space(SScrewsSub,M,thetaSub);
%     plotPose(TTheta{i,1})
%     hold on
end

% Get Robot Position
numFrames =numel(SScrews);
for i = 1:numFrames
    TTemp = TTheta{i};
    posePos(i,:) = TTemp(1:3,4);
end

RobotPos = getRobotPosition(posePos);

for i = 1:size(RobotPos,1)
    distSq(i,1) = norm(RobotPos(i,:)-obstaclePos);
end

w = min(distSq);
end