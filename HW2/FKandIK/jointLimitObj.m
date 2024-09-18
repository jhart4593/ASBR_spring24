function w = jointLimitObj(theta,upperBnd,lowerBnd)

n = numel(theta); 
w = zeros(1,n);
for i = 1:n
    range = upperBnd(i)-lowerBnd(i);
    w(i) = ((theta(i)-range/2)/range)^2;

end

w = -sum(w)/(2*n);
end