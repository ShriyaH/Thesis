%%---------Unit Tests---------%%
clear all
clc

dq_op = 1;

%% DQ Operations 

if dq_op == 1
    q = [0.9; 0.75; 0.8; 0.3];
    q2 = [0.8; 0.5; 0.99; 0.5];
    r = [2; -4; -5; 0];

    q_unit = q/norm(q);
    dq = Q2DQ(q,r,1);

    dq1 = [dq(1:4); 9; 10; -4; 0];
    dq_unit = dq_norm(dq1,1);
    
   
    p1 = [1; 0; 0];
    p_trans1 = dq_trans(dq,p1,1);
    
    p2 = [0; 1; 0];
    p_trans2 = dq_trans(dq,p2,1);
    
    p3 = [0; 0; 1];
    p_trans3 = dq_trans(dq,p3,1);
    
    cross(p_trans1(5:7),p_trans2(5:7)) 
    
    figure()
    
    quiver3(0,0,0,p1(1),p1(2),p1(3),'Color','red');
    hold on
    quiver3(0,0,0,p2(1),p2(2),p2(3),'Color','green');
    quiver3(0,0,0,p3(1),p3(2),p3(3),'Color','blue');
    quiver3(r(1),r(2),r(3),p_trans1(5),p_trans1(6),p_trans1(7),'Color','red');
    quiver3(r(1),r(2),r(3),p_trans2(5),p_trans2(6),p_trans2(7),'Color','green');
    quiver3(r(1),r(2),r(3),p_trans3(5),p_trans3(6),p_trans3(7),'Color','blue');
    axis equal
end

