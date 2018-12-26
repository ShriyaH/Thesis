function [g_A,g_I,g_B,U,Wf,W] = F_G(r_A,q_BI,q_AI,rho,V,F,E,F_tilde,E_tilde,G)
%% Function to calculate Cube gravity from Asteroid structure (from Get_Asteroid function)
%
% Inputs: r_A - position of a field point in Asteroid frame
%         Asteroid - Asteroid structure
%
% Outputs: g_A - gravitational acceleration in Asteroid frame
%          U - gravitational potential in Asteroid frame
%          Wf - THE solid angle, which shows whether the point is in or out
%               It can be used for stopping simulation if SC crashes
% codegen -config:mex Poly_g 
% -args {zeros(3,1,'double'),zeros(4,1,'double'),zeros(4,1,'double'), zeros(1,1,'double'), zeros(2048,3,'double'), zeros(4092,3,'double'), zeros(6138,4,'double'),zeros(3,3,4092,'double'), zeros(3,3,6138,'double')}
%% Initialization
Wf = 0;

Facets = [0 0 0]';
Edges = [0 0 0]';

U_edg = 0;
U_fac = 0;

S = r_A;

for j=1:1:size(E,1)
    
    % Calculate gravity due to edge potential
    
    R1 = V(E(j,1),:);    
    R2 = V(E(j,2),:);    

    Ri = R1'-S;
    ri = norm(Ri);
    
    Rj = R2'-S;
    rj = norm(Rj);
        
    Rij = Rj-Ri;
    eij = norm(Rij);

    Re1 =(Ri+Rj)/2;
   
    Le = log((ri+rj+eij)/(ri+rj-eij));
    
    if Le == Inf
     
        Le = 0;
     
    end
    
    Edges = (Le*E_tilde(:,:,j)*Re1)+Edges;
    U_edg = (Le*Re1'*E_tilde(:,:,j)*Re1)+U_edg;
    
end

 for i=1:1:(size(F,1))
    
    % Calculate gravity due to face potential
    
    R1 = V(F(i,1),:);    
    R2 = V(F(i,2),:);    
    R3 = V(F(i,3),:);
    

    Ri = R1'-S;
    ri = norm(Ri);
    
    Rj = R2'-S;
    rj = norm(Rj);
    
    Rk = R3'-S;
    rk = norm(Rk);
    
    
    wf = 2*atan2(dot(Ri, cross(Rj,Rk)),(ri*rj*rk+ri*dot(Rj,Rk)+rj*dot(Rk,Ri)+rk*dot(Ri,Rj)));
    
    Rf =(Ri+Rj+Rk)/3;

    
    Facets = wf*F_tilde(:,:,i)*Rf+Facets;
    U_fac = wf*Rf'*F_tilde(:,:,i)*Rf+U_fac;
    Wf = wf+Wf;
  
 end

g_A = G*rho*(-Edges+Facets);
g_I = quat_trans(conj_quat(q_AI),g_A,'n');
g_B = quat_trans(q_BI,g_I,'n');

U = 0.5*G*rho*(U_edg-U_fac);

if Wf>10
    W=1;
else
    W=0;
end
end


