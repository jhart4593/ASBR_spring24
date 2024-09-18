function w = manipulabilityMeasure(BScrews,theta)
% Compute manipulability objective from W8 L1 notes
Jb = J_body(BScrews,theta);
w =sqrt((det(Jb*Jb')));
end