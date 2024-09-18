function [J_m] = J_manipulability(J_b)
%+ Given the body Jacobian of the serial chain robot, compute the
% manipulability Jacobian using the fast manipulator hessian.
%+ Returns:
% Manipulability Jacobian
%+ Reference: Manipulator Differential Kinematics (MDK) parts I and II.
%+ Revision List: 
%+ Rev 1.0: Initial Release

%Compute Hessian using fast manipulator Hessian method in MDK II (eqns
%25,27)
[~,col] = size(J_b);
for jj = 1:col
    H_j = [];
    for kk = 1:col
        Jv_j = J_b(4:6,jj);
        Jv_k = J_b(4:6,kk);
        Jw_j = J_b(1:3,jj);
        Jw_k = J_b(1:3,kk);
        
        if jj >= kk
            H_a_jk = cross(Jw_k,Jv_j);
        else
            H_a_jk = cross(Jw_j,Jv_k);
        end

        H_alpha_jk = cross(Jw_k,Jw_j);
        H_jk = [H_alpha_jk;H_a_jk];
        H_j(:,kk) = H_jk;
    end
    H{jj} = H_j;
end

%Compute the manipulability
m = sqrt(det(J_b*J_b'));

%Calculate manipulability Jacobian using eqn 43 from MDK II
if m == 0
    J_m = zeros(col,1);
else
    J_m = [];
    for ii = 1:col
        A = J_b*H{ii}';
        B = inv(J_b*J_b');
        J_m(ii) = A(:)'*B(:);
    end
    J_m = m*J_m';
end

end