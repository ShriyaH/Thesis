function [a_3BP_I]   = F_3BP(r_i,r_d,mu_s,Switch)
%% This function generates third body perturbance on the SC in the body and inertial frame of reference
%[a_3BP_I]   = F_3BP(r_i,r_d,mu_s,q_BI,m,Switch)
    if Switch == 1
        r_id = r_d - r_i;
        a_3BP_I = -mu_s*(r_id/norm(r_id)^3 - r_d/norm(r_d)^3); 

%         F_3BP_I = a_3BP_I * m;

%         F_3BP_B = quat_trans(q_BI,F_3BP_I,'n');

    else
%         F_3BP_B = [0;0;0;0];
        a_3BP_I = [0;0;0;0];
    end

end
