function [Tinv] = Tinv(T)
% Takes inverse of transformation matrix

R = T(1:3,1:3);
P = T(1:3,4);

Tinv= [R' -R'*P
    zeros(1,3) 1]; 
end