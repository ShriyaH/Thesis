function [dr_dq] = q_pde(q,r)
%get partial differential of a rotated vector (q.r.q*) with respect to the
%quaternion (q)
 q1 = q(1);
 q2 = q(2);
 q3 = q(3);
 q4 = q(4);
 
 r1 = r(1);
 r2 = r(2);
 r3 = r(3);
 
 a = q4.*r;
 b = cross(q(1:3),r);
 c = -r*q(1:3)';
 d = dot(r,q(1:3))*eye(3);
 e = q(1:3)*r';
 f = q4*omega_tensor(r,1);
 dr_dq = 2.*[q4.*r + cross(q(1:3),r), -r*q(1:3)' + dot(r,q(1:3))*eye(3) + q(1:3)*r' - q4*omega_tensor(r,1)];
end

 