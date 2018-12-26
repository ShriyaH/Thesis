function [g_A, U, Wf] = Poly_gravity(r_A,G,Object)
%% Function to calculate Object gravity from Asteroid structure (from Get_Asteroid function)
%
% Inputs: r_A - position of a field point in Asteroid frame
%         Asteroid - Asteroid structure
%
% Outputs: g_A - gravitational acceleration in Asteroid frame
%          U - gravitational potential in Asteroid frame
%          Wf - THE solid angle, which shows whether the point is in or out
%               It can be used for stopping simulation if SC crashes

%% Initialization

rho = Object.rho;

V = Object.Vertices;

F = Object.Facets;
E = Object.Edges;
F_tilde = Object.F_tilde;
E_tilde = Object.E_tilde;

Wf = 0;

Facets = [0 0 0]';
Edges = [0 0 0]';

U_edg = 0;
U_fac = 0;

S = [r_A(1) r_A(2) r_A(3)];

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

    U_fac = wf*Rf'*F_tilde(:,:,i)*Rf+U_fac;
    Facets = wf*F_tilde(:,:,i)*Rf+Facets;
    
    Wf = wf+Wf;
  
 end

g_A = G*rho*(-Edges+Facets);

U = 0.5*G*rho*(U_edg-U_fac);

end

