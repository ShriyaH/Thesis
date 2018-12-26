function [F_SRP_B,a_SRP_I]   = F_SRP(q_BI,flux,pos,normalsf,A_facet,a_r,a_d,AU,c,m,Switch)
%% This function generates SRP force per facet in the body and inertial frame of reference
    if Switch == 1
        e = (pos(1:3,1)/norm(pos(1:3,1)))';   %presently just postion vector of sun from asteroid in the inertial frame of reference can be resolved per facet 
        P = flux/(norm(pos(1:3,1)/AU)^2*c);   %radiation pressure at the distance of the asteroid

        F_SRP_B = zeros(size(normalsf,1),3);  
        for i=1:1:size(normalsf,1)
            n_B(i,1:3) = quat_trans(q_BI,normalsf(i,1:3),'vect');  %normals in the body frame
            co = dot(n_B(i,1:3),e(1,1:3),2);
     
            if i<=12 %change as per number of normals representing the main body and solar arrays of SC
                F = -P*A_facet(i,:)*co.*((2*(a_d(1,1)/3 + a_r(1,1)*co))'.*normalsf(i,1:3) + (1-a_r(1,1))'.*e);
                F_SRP_B(i,:) = F;
            else
                F = -P*A_facet(i,:)*co.*((2*(a_d(1,2)/3 + a_r(1,2)*co))'.*normalsf(i,1:3) + (1-a_r(1,2))'.*e);
                F_SRP_B(i,:) = F;
            end

        F_SRP_I(i,:) = quat_trans(conj_quat(q_BI),[F_SRP_B(i,:) 0],'n');
        end
        % 
       F_SRP_T = sum(F_SRP_B);
       a_SRP_I = sum(F_SRP_I)'/m;
    else
        F_SRP_B = [0;0;0;0];
        a_SRP_I = [0;0;0;0];
    end
end
