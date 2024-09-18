function virtualTheta = getVirtualTheta(theta)

joint_list = {'SE','SP','SA','SR','SF','EF','WP'}; 

SE = strcmpi(joint_list,'se'); 
SP = strcmpi(joint_list,'sp');
SA = strcmpi(joint_list,'sa');
SR = strcmpi(joint_list,'sr');
SF = strcmpi(joint_list,'sf');
EF = strcmpi(joint_list,'ef');
WP = strcmpi(joint_list,'wp');

virtualTheta = [ 0; 0; theta(SE); theta(SP); -theta(SP); % //Additional virtual joint due to clavicle linkage actuatoed through parallelogram
            theta(SA); theta(SR); theta(SF); theta(EF); theta(WP)];

end