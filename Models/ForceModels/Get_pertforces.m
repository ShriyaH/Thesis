function [F_D, T_D, g_B] = Get_pertforces(m,C_BA,r_A,r_B,rs_B,e_B,mu,Asteroid)
%%Get all accelerations and torques in the body reference frame
global CONSTANTS Switch SC Sun 


c = CONSTANTS.c;
AU = CONSTANTS.AU;
mu_s = CONSTANTS.mu_s;
flux = CONSTANTS.flux;
% pos_s = Sun.pos;

normalsf = SC.Polyhedron.normalsf;
A = SC.Polyhedron.A_facet;
a_r = SC.a_r;
a_d = SC.a_d;
% m = SC.mass.m_i;
C = SC.Polyhedron.C;
V = SC.Polyhedron.Vertices;
rs_I = Sun.rs_I(1:3);
r_A = r_A(1:3);
r_B = r_B(1:3);
%% Solar Radiation Pressure
if Switch.SRP
    
   P = flux/((Asteroid.kep_orbit(1)/AU)^2*c);   %radiation pressure at the distance of the Asteroid
     
%    e = (pos_s(1:3,1)/norm(pos_s(1:3,1)))';  %presently just unit postion vector of sun from Asteroid in the inertial or Asteroid frame since sun pos is fixed
%    e_B = quat_trans(q_BA,e_A,'vect');     %sun position vector in the body frame 
        
   F_SRP_B = zeros(size(normalsf,1),3); 
   T_SRP_B = zeros(size(normalsf,1),3);

   for i=1:1:size(normalsf,1)
       co(i,1) = dot(normalsf(i,1:3),e_B',2); %cos of angle between sun vector and the SC surface normals
            
       if co(i,1) < 0
          F_SRP_B(i,:) = 0;
       else
          F_SRP_B(i,:) = -A(i)*co(i,1)*((2*(a_d(i,1)/3 + a_r(i,1)*co(i,1))).*normalsf(i,1:3) + (1-a_r(i,1)).*e_B');            
       end
          T_SRP_B(i,:) = cross(C(i,:),F_SRP_B(i,:),2);
   
    end
       
	F_SRP_T = P*sum(F_SRP_B);
%     a_SRP_T = norm(F_SRP_T)/m;
	F_SRP_B = [F_SRP_T'; 0];
    T_SRP_B = [sum(T_SRP_B)'; 0];  
else  
	F_SRP_B = [0;0;0;0];
	T_SRP_B = [0;0;0;0];
end

%% 3rd Body Perturbations
if Switch.TBP
    r_bs_B = rs_B(1:3) - r_B;          %position of sun wrt to SC
    a_3BP_B = -mu_s.*(r_bs_B./norm(r_bs_B)^3 - rs_B./norm(rs_B)^3);

    F_3BP_B = [m.*a_3BP_B;0];
else     
    F_3BP_B = [0;0;0;0];
end

%% Gravitational acceleration and torques
if Switch.poly_grav 
    q_BA = DCM2Q(C_BA);
    [g_A] = Poly_g_new(r_A,q_BA,Asteroid);
    F_B = [m.*(C_BA*g_A); 0];
elseif Switch.GG ~=1
    r = norm(r_B);
    F_B = [-(m*mu).*r_B(1:3)./r^3; 0];
elseif Switch.GG
    F_B = [0;0;0;0];
end

if Switch.GG
    %Number of point masses: 80(20 block, 60 solar arrays)
    n_block = 8 + 12; %8 vertices, 12 facet centres
    n_SA = 40 + 20; % 4 vertices and 2 facet centres each on 10 panels

    %Mass of point masses
    m_pm = [((m-150)/n_block).*(ones(n_block,1)); (150/n_SA).*(ones(n_SA,1))]; %mass of SA: 150kg 

    %Position of point masses wrt body frame
    r_pm = [ V(1:8,:); C(1:12,:); V(9:48,:); C(13:32,:) ];
    r_pm_B = -r_pm + r_B';

    %Grav accl, gradient torque  of point masses wrt body frame
    for i = 1:length(m_pm)
        r_pm_A(:,i) = C_BA'*r_pm_B(i,:)';
        q_BA = DCM2Q(C_BA);
        if Switch.sp_grav == 1
           r = norm(r_pm_B(i,:));
           g_pm_B(:,i) = (-(mu).*r_pm_B(i,:)./r^3)'; 
        else
           [g_pm_A(:,i),g_pm_B(:,i)] = Poly_g_new(r_pm_A(:,i),q_BA,Asteroid); 
        end
        g(i,:) = g_pm_B(1:3,i)';
        F_GG(i,:) = (m_pm(i) .* g(i,:));
        T_GG(i,:) = cross(r_pm(i,:),F_GG(i,:),2);
    end
    [g_A,g_B] = Poly_g_new(r_A,q_BA,Asteroid); 
    g_B = g_B(1:3);
    F_GG_B = [sum(F_GG)';0];
    T_GG_B = [sum(T_GG)';0];
else
    T_GG_B = [0;0;0;0];
    F_GG_B = [0;0;0;0];
end 

F_D = F_B + F_SRP_B + F_3BP_B + F_GG_B;
T_D = T_GG_B + T_SRP_B;

end
    
 