function [g_A, g_I, Wf, U] = Poly_g(r_A, q_AI, rho, Vert, Fac, Edg, F_tilde, E_tilde)

G=6.67408e-11;

V=Vert;

F=Fac;

E=Edg;

Wf=0;

Facets=[0 0 0]';
Edges=[0 0 0]';


U_edg=0;
U_fac=0;

S=r_A;

for j=1:1:size(E,1)
    
    R1=V(E(j,1),:);    
    R2=V(E(j,2),:);    

    Ri=R1'-S;
    ri=norm(Ri);
    
    Rj=R2'-S;
    rj=norm(Rj);
        
    Rij=Rj-Ri;
    eij=norm(Rij);
%     Re1=Ri;
    Re1=(Ri+Rj)/2;
   
    Le=log((ri+rj+eij)/(ri+rj-eij));
    if Le==Inf
     
        Le=0;
     
    end
    
    U_edg=(Le*Re1'*E_tilde(:,:,j)*Re1)+U_edg;

    Edges=(Le*E_tilde(:,:,j)*Re1)+Edges;

end

 for i=1:1:(size(F,1))
      
    R1=V(F(i,1),:);    
    R2=V(F(i,2),:);    
    R3=V(F(i,3),:);
    

    Ri=R1'-S;
    ri=norm(Ri);
    
    Rj=R2'-S;
    rj=norm(Rj);
    
    Rk=R3'-S;
    rk=norm(Rk);
    
    
    wf=2*atan2(dot(Ri, cross(Rj,Rk)),(ri*rj*rk+ri*dot(Rj,Rk)+rj*dot(Rk,Ri)+rk*dot(Ri,Rj)));
    
    Rf=(Ri+Rj+Rk)/3;

    U_fac=wf*Rf'*F_tilde(:,:,i)*Rf+U_fac;

    Facets=wf*F_tilde(:,:,i)*Rf+Facets;
    
    Wf=wf+Wf;
  
 end

g_A = G*rho*(-Edges+Facets);
g_I = quat_trans(conj_quat(q_AI),g_A,'n');
% g_B = quat_trans(q_BI,g_I,'n');

U = 0.5*G*rho*(U_edg-U_fac);

end

