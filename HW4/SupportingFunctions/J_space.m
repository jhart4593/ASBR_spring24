function Jacobian = J_space(Screws,theta)
%+ Given the screws of the serial chain robot and
% the joint angles, find the Jacobian using the space representation
%+ Reference: Section 5.1.1 Modern Robotics Mechanics, Planning and
% Control (Equation 5.11)
%+ Revision List: 
%+ Rev 1.0: Initial Release

thetaColVec = isequal(size(theta,1), 1); 
thetaRowVec = isequal(size(theta,2), 1); 

if ~(thetaColVec || thetaRowVec)
    error('Error: Theta must be a row or column vector');
end

numJoints = max(size(Screws)); 
if max(size(theta)) ~= numJoints
   error('Error: Number of screws and number of angles dont match');
end

Jacobian = Screws{1};
expProd = eye(4);
for i = 2:numJoints
    expProd = expProd*ScrewAxisDist2MatExp(Screws{i-1},theta(i-1));
    Jacobian(:,i) = T2Adj(expProd)*Screws{i};

end

end