function [toolTipSpace, toolAxisSpace] = calculateToolTip(SScrews,M,toolLength,theta)

TTheta = FK_space(SScrews,M,theta);

attachementPoint = TTheta(1:3,4); % assume attachment point at center of end effector frame
toolAxisBody = [0;0;1]; % assume along body frame z axis
toolAxisSpace = TTheta(1:3,1:3)*toolAxisBody;
toolTipSpace = attachementPoint+toolAxisSpace*toolLength; 

% toolTipSpace = TTheta*[0;0;toolLength;1];
% toolTipSpace = toolTipSpace(1:3);
end