% function [g_B_pdc,g_B_pdf,TG_B_pdc,TG_B_pdf] = dg_complex_diff(q_BA,m)

global CONSTANTS Kleopatra 

G=CONSTANTS.G;

V=Kleopatra.Polyhedron.Vertices;

F=Kleopatra.Polyhedron.Facets;

E=Kleopatra.Polyhedron.Edges;

E_tilde = Kleopatra.Polyhedron.E_tilde;
F_tilde = Kleopatra.Polyhedron.F_tilde;

rho = Kleopatra.rho;
%% Compute gravitational acceleration and gravity gradient torque
%Initialise function values for loop
Wf=@(q1,q2,q3,q4,q5,q6,q7,q8) 0;
Facets=@(q1,q2,q3,q4,q5,q6,q7,q8) [0 0 0]';
Edges=@(q1,q2,q3,q4,q5,q6,q7,q8) [0 0 0]';
U_edg=@(q1,q2,q3,q4,q5,q6,q7,q8) 0;
U_fac=@(q1,q2,q3,q4,q5,q6,q7,q8) 0;

%function for r_A (2.q_d.q_r^*)
S =@(q1,q2,q3,q4,q5,q6,q7,q8)  2.*cross_quat([q5;q6;q7;q8],[-q1;-q2;-q3;q4]);
S =@(q1,q2,q3,q4,q5,q6,q7,q8)  S(1:3);

for j=1:1:size(E,1)
    
    R1=V(E(j,1),:);    
    R2=V(E(j,2),:);    

    Ri=@(q1,q2,q3,q4,q5,q6,q7,q8) R1'-S(q1,q2,q3,q4,q5,q6,q7,q8);
    ri=@(q1,q2,q3,q4,q5,q6,q7,q8) norm(Ri(q1,q2,q3,q4,q5,q6,q7,q8));
    
    Rj=@(q1,q2,q3,q4,q5,q6,q7,q8) R2'-S(q1,q2,q3,q4,q5,q6,q7,q8);
    rj=@(q1,q2,q3,q4,q5,q6,q7,q8) norm(Rj(q1,q2,q3,q4,q5,q6,q7,q8));
        
    Rij=@(q1,q2,q3,q4,q5,q6,q7,q8) Rj(q1,q2,q3,q4,q5,q6,q7,q8)-Ri(q1,q2,q3,q4,q5,q6,q7,q8);
    eij=@(q1,q2,q3,q4,q5,q6,q7,q8) norm(Rij(q1,q2,q3,q4,q5,q6,q7,q8));

    Re1=@(q1,q2,q3,q4,q5,q6,q7,q8) (Ri(q1,q2,q3,q4,q5,q6,q7,q8)+Rj(q1,q2,q3,q4,q5,q6,q7,q8))/2;
   
    Le=@(q1,q2,q3,q4,q5,q6,q7,q8) log((ri(q1,q2,q3,q4,q5,q6,q7,q8)+rj(q1,q2,q3,q4,q5,q6,q7,q8)+eij(q1,q2,q3,q4,q5,q6,q7,q8))/...
            (ri(q1,q2,q3,q4,q5,q6,q7,q8)+rj(q1,q2,q3,q4,q5,q6,q7,q8)-eij(q1,q2,q3,q4,q5,q6,q7,q8)));

    U_edg=@(q1,q2,q3,q4,q5,q6,q7,q8) (Le(q1,q2,q3,q4,q5,q6,q7,q8)*Re1(q1,q2,q3,q4,q5,q6,q7,q8)'*E_tilde(:,:,j)*Re1(q1,q2,q3,q4,q5,q6,q7,q8))+U_edg(q1,q2,q3,q4,q5,q6,q7,q8);

    Edges=@(q1,q2,q3,q4,q5,q6,q7,q8) (Le(q1,q2,q3,q4,q5,q6,q7,q8)*E_tilde(:,:,j)*Re1(q1,q2,q3,q4,q5,q6,q7,q8))+Edges(q1,q2,q3,q4,q5,q6,q7,q8);

end

 for i=1:1:(size(F,1))
      
    R1=V(F(i,1),:);    
    R2=V(F(i,2),:);    
    R3=V(F(i,3),:);
    

    Ri=@(q1,q2,q3,q4,q5,q6,q7,q8) R1'-S(q1,q2,q3,q4,q5,q6,q7,q8);
    ri=@(q1,q2,q3,q4,q5,q6,q7,q8) norm(Ri(q1,q2,q3,q4,q5,q6,q7,q8));
    
    Rj=@(q1,q2,q3,q4,q5,q6,q7,q8) R2'-S(q1,q2,q3,q4,q5,q6,q7,q8);
    rj=@(q1,q2,q3,q4,q5,q6,q7,q8) norm(Rj(q1,q2,q3,q4,q5,q6,q7,q8));
    
    Rk=@(q1,q2,q3,q4,q5,q6,q7,q8) R3'-S(q1,q2,q3,q4,q5,q6,q7,q8);
    rk=@(q1,q2,q3,q4,q5,q6,q7,q8) norm(Rk(q1,q2,q3,q4,q5,q6,q7,q8));
    
    
    wf=@(q1,q2,q3,q4,q5,q6,q7,q8) 2*atan2(dot(Ri(q1,q2,q3,q4,q5,q6,q7,q8), cross(Rj(q1,q2,q3,q4,q5,q6,q7,q8),Rk(q1,q2,q3,q4,q5,q6,q7,q8))),...
        (ri(q1,q2,q3,q4,q5,q6,q7,q8)*rj(q1,q2,q3,q4,q5,q6,q7,q8)*rk(q1,q2,q3,q4,q5,q6,q7,q8)+ri(q1,q2,q3,q4,q5,q6,q7,q8)*...
        dot(Rj(q1,q2,q3,q4,q5,q6,q7,q8),Rk(q1,q2,q3,q4,q5,q6,q7,q8))+rj(q1,q2,q3,q4,q5,q6,q7,q8)*dot(Rk(q1,q2,q3,q4,q5,q6,q7,q8),Ri(q1,q2,q3,q4,q5,q6,q7,q8))+...
        rk(q1,q2,q3,q4,q5,q6,q7,q8)*dot(Ri(q1,q2,q3,q4,q5,q6,q7,q8),Rj(q1,q2,q3,q4,q5,q6,q7,q8))));
    
    Rf=@(q1,q2,q3,q4,q5,q6,q7,q8) (Ri(q1,q2,q3,q4,q5,q6,q7,q8)+Rj(q1,q2,q3,q4,q5,q6,q7,q8)+Rk(q1,q2,q3,q4,q5,q6,q7,q8))/3;

    U_fac=@(q1,q2,q3,q4,q5,q6,q7,q8) wf(q1,q2,q3,q4,q5,q6,q7,q8)*Rf(q1,q2,q3,q4,q5,q6,q7,q8)'*F_tilde(:,:,i)*Rf(q1,q2,q3,q4,q5,q6,q7,q8)+U_fac(q1,q2,q3,q4,q5,q6,q7,q8);

    Facets=@(q1,q2,q3,q4,q5,q6,q7,q8) wf(q1,q2,q3,q4,q5,q6,q7,q8)*F_tilde(:,:,i)*Rf(q1,q2,q3,q4,q5,q6,q7,q8)+Facets(q1,q2,q3,q4,q5,q6,q7,q8);
    
    Wf=@(q1,q2,q3,q4,q5,q6,q7,q8) wf(q1,q2,q3,q4,q5,q6,q7,q8)+Wf(q1,q2,q3,q4,q5,q6,q7,q8);
  
 end
 
%polyhedral gravity field in the A-frame
g_A =@(q1,q2,q3,q4,q5,q6,q7,q8) G*rho*(-Edges(q1,q2,q3,q4,q5,q6,q7,q8)+Facets(q1,q2,q3,q4,q5,q6,q7,q8));

%polyhedral gravity field in the B-frame
g_B =@(q1,q2,q3,q4,q5,q6,q7,q8) quat_trans([q1;q2;q3;q4;q5;q6;q7;q8],g_A(q1,q2,q3,q4,q5,q6,q7,q8),'n');

%gravitational potential
U =@(q1,q2,q3,q4,q5,q6,q7,q8) 0.5*G*rho*(U_edg(q1,q2,q3,q4,q5,q6,q7,q8)-U_fac(q1,q2,q3,q4,q5,q6,q7,q8));

%gravity gradient torque
% TG_B =@(q1,q2,q3,q4,q5,q6,q7,q8) 

%% Comparison Complex Differentiation and Finite Difference Methods
%Complex Differentiation
g_B_pdc = @(q1,q2,q3,q4,q5,q6,q7,q8,h) imag(g_B(q1+i*h,q2+i*h,q3+i*h,q4+i*h,q5+i*h,q6+i*h,q7+i*h,q8+i*h))./h;
% TG_B_pdc = @(q1,q2,q3,q4,q5,q6,q7,q8,h) imag(TG_B(q1+i*h,q2+i*h,q3+i*h,q4+i*h,q5+i*h,q6+i*h,q7+i*h,q8+i*h))./h;

%Finite Difference
g_B_pdf = @(q1,q2,q3,q4,q5,q6,q7,q8,h) (g_B(q1+h,q2+h,q3+h,q4+h,q5+h,q6+h,q7+h,q8+h) ...
        - g_B(q1-h,q2-h,q3-h,q4-h,q5-h,q6-h,q7-h,q8-h))./(2*h);
% TG_B_pdf = @(q1,q2,q3,q4,q5,q6,q7,q8,h) (TG_B(q1+h,q2+h,q3+h,q4+h,q5+h,q6+h,q7+h,q8+h) ...
%         - TG_B(q1-h,q2-h,q3-h,q4-h,q5-h,q6-h,q7-h,q8-h))./(2*h);

%print results with increasing h    
format long
count = 1;
for h = 10.^(-1:16)
    count = count+1;
%     disp([ g_B_pdc(0.182574185835055,0.365148371670111,0.547722557505166,0.730296743340221,0,18257.4185835055,36514.8371670111,-36514.8371670111,h)...
%            g_B_pdf(0.182574185835055,0.365148371670111,0.547722557505166,0.730296743340221,0,18257.4185835055,36514.8371670111,-36514.8371670111,h)])

    pde_comp(:,count) = g_B_pdc(0.182574185835055,0.365148371670111,0.547722557505166,0.730296743340221,0,18257.4185835055,36514.8371670111,-36514.8371670111,h);
    pde_fin(:,count) = g_B_pdf(0.182574185835055,0.365148371670111,0.547722557505166,0.730296743340221,0,18257.4185835055,36514.8371670111,-36514.8371670111,h);
end
% 
% for h = 10.^(-1:16)
%     disp([ TG_B_pdc(0.182574185835055,0.365148371670111,0.547722557505166,0.730296743340221,0,18257.4185835055,36514.8371670111,-36514.8371670111,h)...
%            TG_B_pdf(0.182574185835055,0.365148371670111,0.547722557505166,0.730296743340221,0,18257.4185835055,36514.8371670111,-36514.8371670111,h)])
% end

% end

