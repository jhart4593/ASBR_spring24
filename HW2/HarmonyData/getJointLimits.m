function bnds = getJointLimits(jointIndx,side)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
switch jointIndx
    case 1
        bnds = [-.1 .73];
    case 2
        bnds = [-.36 .8];
    case 3 
        bnds = [-1.15 1.06];
    case 4
        bnds = [-1.75 1.41];
    case 5
        bnds = [-.66 2.47];
    case 6
        bnds = [-2.28 -.09];
    case 7
        bnds = [-1.27 1.27];
end

if strcmpi(side,'L')
    bnds = -flip(bnds);

end