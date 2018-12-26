function [g_A,g_I,Wf,W] = poly_g(r_A,q_AI,rho,V,F,E,F_tilde,E_tilde,G)
Wf = 0;
R1e=V(E(:,1),:);    
R2e=V(E(:,2),:);    

Ri=R1e'-r_A;
ri=(Ri(1,:).^2+Ri(2,:).^2+Ri(3,:).^2).^0.5;

Rj=R2e'-r_A;
rj=(Rj(1,:).^2+Rj(2,:).^2+Rj(3,:).^2).^0.5;

Rij=Rj-Ri;
eij=(Rij(1,:).^2+Rij(2,:).^2+Rij(3,:).^2).^0.5;

Le=log((ri+rj+eij)./(ri+rj-eij));

Le(Le==Inf) = 0;
Re1=(Ri+Rj)/2.*Le;
Edges = zeros(3, size(E,1));
 for j=1:1:size(E,1)
   
    Edges(:,j)=(E_tilde(:,:,j)*Re1(:,j));
%     U_edg = (Le*Re1'*E_tilde(:,:,j)*Re1)+U_edg;
end

clear R1e R2e Ri Rj Rk ri rj rk Re1 Le I eij
      
R1f=V(F(:,1),:);    
R2f=V(F(:,2),:);    
R3f=V(F(:,3),:);


Ri=R1f'-r_A;
ri=(Ri(1,:).^2+Ri(2,:).^2+Ri(3,:).^2).^0.5;

Rj=R2f'-r_A;
rj=(Rj(1,:).^2+Rj(2,:).^2+Rj(3,:).^2).^0.5;

Rk=R3f'-r_A;
rk=(Rk(1,:).^2+Rk(2,:).^2+Rk(3,:).^2).^0.5;


wf=2*atan2(dot(Ri, cross(Rj,Rk)),(ri.*rj.*rk+ri.*dot(Rj,Rk)+rj.*dot(Rk,Ri)+rk.*dot(Ri,Rj)));

Rf=(Ri+Rj+Rk)/3.*wf;

Facets = zeros(3,size(F,1));

for i=1:1:size(F,1)

    Facets(:,i)=F_tilde(:,:,i)*Rf(:,i);
%      U_fac = wf*Rf'*F_tilde(:,:,i)*Rf+U_fac;
    Wf = wf+Wf;
end

g_A=G*rho*(-sum(Edges,2)+sum(Facets,2));
g_I = quat_trans(conj_quat(q_AI),g_A,'n');
% g_B = quat_trans(q_BI,g_I,'n');

% U = 0.5*G*rho*(U_edg-U_fac);

if Wf>10
    W=1;
else
    W=0;
end
end