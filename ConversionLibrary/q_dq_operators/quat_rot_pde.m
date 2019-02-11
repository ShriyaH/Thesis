function [pdw,pdW,pdR] = quat_rot_pde(p,w,W)
%pde with respect to quaternion (p) for rotation of vector(q)
%for dual quaternion(p), valid for dual vector rotation(q)
%for relative ref frame omega_dot, get pde for position vector in coriolis
%term
%     if length(p)== 4 && length(W)== 4
%         pq = cross_quat(p,W);
%         omega_pde = [-pq(4) -pq(3) pq(2) pq(1)  
%                      pq(3) -pq(4) -pq(1) pq(2)  
%                      -pq(2) pq(1) -pq(4) pq(3) 
%                      0 0 0 0];
%          pdW = 2.*omega_pde;
%     
%     elseif length(p)== 8 && length(W)== 8
    p1=p(1); p2=p(2); p3=p(3); p4=p(4);
    p5=p(5); p6=p(6); p7=p(7); p8=p(8);
    
    pw = cross_quat(p(1:4),w(5:8));
    pdw = [0 0 0 0 0 0 0 0 
             0 0 0 0 0 0 0 0 
             0 0 0 0 0 0 0 0
             0 0 0 0 0 0 0 0
             -pw(4) -pw(3) pw(2) pw(1) 0 0 0 0 
             pw(3) -pw(4) -pw(1) pw(2) 0 0 0 0 
             -pw(2) pw(1) -pw(4) pw(3) 0 0 0 0
              0 0 0 0 0 0 0 0];

    pW1 = cross_quat(p(1:4),W(1:4));
    pW2 = cross_quat(p(1:4),W(5:8));
    omega_pde = [-pW1(4) -pW1(3) pW1(2) pW1(1) 0 0 0 0 
                 pW1(3) -pW1(4) -pW1(1) pW1(2) 0 0 0 0 
                 -pW1(2) pW1(1) -pW1(4) pW1(3) 0 0 0 0
                 0 0 0 0 0 0 0 0
                 -pW2(4) -pW2(3) pW2(2) pW2(1) 0 0 0 0 
                 pW2(3) -pW2(4) -pW2(1) pW2(2) 0 0 0 0 
                 -pW2(2) pW2(1) -pW2(4) pW2(3) 0 0 0 0
                 0 0 0 0 0 0 0 0];

    pdW =2.*omega_pde;

    pdR = [0 0 0 0 0 0 0 0 
             0 0 0 0 0 0 0 0 
             0 0 0 0 0 0 0 0
             0 0 0 0 0 0 0 0
             -p8 -p7 p6 p5 p4 p3 -p2 -p1 
             p7 -p8 -p5 p6 -p3 p4 p1 -p2
             -p6 p5 -p8 p7 p2 -p1 p4 -p3
             p5 p6 p7 p8 p1 p2 p3 p4];

    pdR = 2.*pdR;
%     end
end

