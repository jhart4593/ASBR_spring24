function [angular_isotropy, linear_isotropy] = J_isotropy(Jacobian)
%+ Given the geometric Jacobian (either Jb in the end-effector frame or 
% Js in the fixed frame)find the isotropy of the manipulability ellipsoid
% + Note when isotropy (>= 1) is low (close to 1) then the manipulability
% ellipsoid is nearly spherical or isotropic meaning that it is equally
% easy to move in any direction
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
angular_isotropy = sqrt(max(eigvalw)/min(eigvalw));

[~,eigvalv] = eig(Av,'vector'); 
linear_isotropy = sqrt(max(eigvalv)/min(eigvalv));
end