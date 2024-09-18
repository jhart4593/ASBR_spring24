function RobotPos = getRobotPosition(poseOrigins)
numFrames = size(poseOrigins,1);
numPts = 10; 
% pt1 = zeros(3,1);
% pt2 = poseOrigins(1,:); 
% t = linspace(0, 1, numPts); % Parameterization from 0 to 1
% xInter = (1 - t') * pt1(1) + t' * pt2(1);
% yInter = (1 - t') * pt1(2) + t' * pt2(2);
% zInter = (1 - t') * pt1(3) + t' * pt2(3);

% RobotPos = [xInter, yInter, zInter];

for i = 1:numFrames-1
    pt1 = poseOrigins(i,:); 
    pt2 = poseOrigins(i+1,:); 
    t = linspace(0, 1, numPts); % Parameterization from 0 to 1
    xInter = (1 - t') * pt1(1) + t' * pt2(1);
    yInter = (1 - t') * pt1(2) + t' * pt2(2);
    zInter = (1 - t') * pt1(3) + t' * pt2(3);
    if i == 1
        RobotPos = [xInter, yInter, zInter];
    else
        RobotPos(end+1:end+numPts,:) = [xInter, yInter, zInter];
    end
end
end