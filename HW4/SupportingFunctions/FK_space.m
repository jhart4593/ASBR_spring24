function TTheta = FK_space(Screws,M,theta)
%+ Given the screws of the serial chain robot, the zero configuration, and
% the joint angles, find the pose (or intermediate pose) aka the foreward
% kinematics using the space representation
%+ Reference: Section 4.1.1 Modern Robotics Mechanics, Planning and
% Control (Equation 4.13-4.14)
%+ Revision List: 
%+ Rev 1.0: Initial Release

if ~isequal(size(M), [4, 4])
    error('Error: Home configuration matrix must be 4x4')
end

thetaColVec = isequal(size(theta,1), 1); 
thetaRowVec = isequal(size(theta,2), 1); 

if ~(thetaColVec || thetaRowVec)
    error('Error: Theta must be a row or column vector');
end

numJoints = max(size(Screws)); 
if max(size(theta)) ~= numJoints
   error('Error: Number of screws and number of angles dont match');
end

TTheta = M; 
for i = numJoints:-1:1
    eStheta = ScrewAxisDist2MatExp(Screws{i},theta(i));
    TTheta = eStheta*TTheta;
end

end