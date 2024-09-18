function [eigenvalues, eigenvectors] = ellipsoid_plot_angular(Jacobian)
%+ Given the geometric Jacobian (either Jb in the end-effector frame or 
% Js in the fixed frame) plot the angular manipulability ellipsoid
%+ Reference: Section 5.4 Modern Robotics Mechanics, Planning and
%+ Revision List: 
%+ Rev 1.0: Initial Release

if size(Jacobian,1) ~= 6
   error('Error: Jacobian must be a 6 x n matrix');
end

Jw = Jacobian(1:3,:); 

if rank(Jw) < size(Jacobian,1)/2
     warning('Angular Jacobian is not full rank, singularity is present');
end

A = Jw*Jw'; 

% Since J is full rank, A is square, symmetric, and positive definite;
% therefore eig is more computation efficient than svd

[eigenvectors,eigenvalues] = eig(A,'vector'); 

xc = 0; % x coordinate of center
yc = 0; % y coordinate of center
zc = 0; % z coordinate of center

xr = sqrt(eigenvalues(1)); % first semi axis length
yr = sqrt(eigenvalues(2)); % second semi axis length
zr = sqrt(eigenvalues(3)); % third semi axis length

[Xe,Ye,Ze] = ellipsoid(xc,yc,zc,xr,yr,zr);
exten = .5;  % Extension for plotting
pa1 = eigenvectors(:,1).*[-(xr+exten) xr+exten]; % Primary axis 1
pa2 = eigenvectors(:,2).*[-(yr+exten) yr+exten]; % Primary axis 2
pa3 = eigenvectors(:,3).*[-(zr+exten) zr+exten]; % Primary axis 3


ellipsoidPoints = eigenvectors*[Xe(:)';Ye(:)';Ze(:)']; % Rotate axes
Xe = reshape(ellipsoidPoints(1,:),size(Xe)); 
Ye = reshape(ellipsoidPoints(2,:),size(Ye));
Ze = reshape(ellipsoidPoints(3,:),size(Ze));

figure; 
surf(Xe,Ye,Ze)
colormap gray
hold on
plot3([pa1(1,1) pa1(1,2)],[pa1(2,1) pa1(2,2)], [pa1(3,1) pa1(3,2)],'r','LineWidth',5, 'DisplayName','Primary Axis 1')
plot3([pa2(1,1) pa2(1,2)],[pa2(2,1) pa2(2,2)], [pa2(3,1) pa2(3,2)],'g','LineWidth',5, 'DisplayName','Primary Axis 2')
plot3([pa3(1,1) pa3(1,2)],[pa3(2,1) pa3(2,2)], [pa3(3,1) pa3(3,2)],'b','LineWidth',5, 'DisplayName','Primary Axis 3')
xlabel('X Axis')
ylabel('Y Axis')
zlabel('Z Axis')
title('Angular Manipulability Ellipsoid')
legend('','Primary Axis 1','Primary Axis 2','Primary Axis 3')
end