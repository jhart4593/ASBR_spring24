function [angular_volume, linear_volume] = J_ellipsoid_volume(Jacobian)
%+ Given the geometric Jacobian (either Jb in the end-effector frame or 
% Js in the fixed frame)find the volume of the manipulability ellipsoid
% + Note when volume is larger then the manipulability
% ellipsoid is larger meaning that there are more directions where it is
% easy to move
%+ Reference: Section 5.4 Modern Robotics Mechanics, Planning and
%+ Revision List: 
%+ Rev 1.0: Initial Release

if size(Jacobian,1) ~= 6
   error('Error: Jacobian must be a 6 x n matrix');
end

if rank(Jacobian) < size(Jacobian,1)
    warning('Jacobian is not full rank, singularity is present');
end

Jw = Jacobian(1:3,:); 
Aw = Jw*Jw'; 

Jv = Jacobian(4:6,:); 
Av = Jv*Jv'; 

% Since J is full rank, A is square, symmetric, and positive definite;
% therefore eig is more computation efficient than svd

[~,eigvalw] = eig(Aw,'vector'); 
angular_volume = sqrt(prod(eigvalw));

[~,eigvalv] = eig(Av,'vector'); 
linear_volume= sqrt(prod(eigvalv));
end