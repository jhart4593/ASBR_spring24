function Jacobian = J_body(BScrews,theta)
%+ Given the screws of the serial chain robot and
% the joint angles, find the Jacobian using the body representation
%+ Reference: Section 5.1.2 Modern Robotics Mechanics, Planning and
% Control (Equation 5.18)
%+ Revision List: 
%+ Rev 1.0: Initial Release

thetaColVec = isequal(size(theta,1), 1); 
thetaRowVec = isequal(size(theta,2), 1); 

if ~(thetaColVec || thetaRowVec)
    error('Error: Theta must be a row or column vector');
end

numJoints = max(size(BScrews)); 
if max(size(theta)) ~= numJoints
   error('Error: Number of screws and number of angles dont match');
end
Jacobian(:,numJoints) = BScrews{numJoints};
expProd = eye(4);

for i = numJoints-1:-1:1
    expProd = expProd*ScrewAxisDist2MatExp(-BScrews{i+1},theta(i+1));
    Jacobian(:,i) = T2Adj(expProd)*BScrews{i};

end

end