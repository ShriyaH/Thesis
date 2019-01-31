function [omega_pde,R_pde] = quat_rot_pde(p,q)
%pde with respect to quaternion (p) for rotation of vector(q)
%for dual quaternion(p), valid for dual vector rotation(q)
%for relative ref frame omega_dot, get pde for position vector in coriolis
%term
    if length(p)== 4 && length(q)== 4
        pq = cross_quat(p,q);
        omega_pde = [-pq(4) -pq(3) pq(2) pq(1)  
                     pq(3) -pq(4) -pq(1) pq(2)  
                     -pq(2) pq(1) -pq(4) pq(3) 
                     0 0 0 0];
         omega_pde = 2.*omega_pde;
    
    elseif length(p)== 8 && length(q)== 8
        p1=p(1); p2=p(2); p3=p(3); p4=p(4);
        p5=p(5); p6=p(6); p7=p(7); p8=p(8);
        
        pq1 = cross_quat(p(1:4),q(1:4));
        pq2 = cross_quat(p(1:4),q(5:8));
        omega_pde = [-pq1(4) -pq1(3) pq1(2) pq1(1) 0 0 0 0 
                     pq1(3) -pq1(4) -pq1(1) pq1(2) 0 0 0 0 
                     -pq1(2) pq1(1) -pq1(4) pq1(3) 0 0 0 0
                     0 0 0 0 0 0 0 0
                     -pq2(4) -pq2(3) pq2(2) pq2(1) 0 0 0 0 
                     pq2(3) -pq2(4) -pq2(1) pq2(2) 0 0 0 0 
                     -pq2(2) pq2(1) -pq2(4) pq2(3) 0 0 0 0
                     0 0 0 0 0 0 0 0];
                 
        omega_pde =2.*omega_pde;
        
        R_pde = [0 0 0 0 0 0 0 0 
                 0 0 0 0 0 0 0 0 
                 0 0 0 0 0 0 0 0
                 0 0 0 0 0 0 0 0
                 -p8 -p7 p6 p5 p4 p3 -p2 -p1 
                 p7 -p8 -p5 p6 -p3 p4 p1 -p2
                 -p6 p5 -p8 p7 p2 -p1 p4 -p3
                 p5 p6 p7 p8 p1 p2 p3 p4];
        
        R_pde = 2.*R_pde;
    end
end

