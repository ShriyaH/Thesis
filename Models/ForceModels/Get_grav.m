function [g_B, T_GG_B] = Get_grav(q_BA,Asteroid)
global CONSTANTS Switch SC 

m = SC.mass.m_i;
C = SC.Polyhedron.C;
V = SC.Polyhedron.Vertices;
m_n = SC.mass.m_i;

C_BA = Q2DCM(q_BA);
r_B = 2.*cross_quat(q_BA(5:8),q_BA(1:4));

if Switch.poly_grav == 1
    r_A = C_BA'*r_B;
    [g_A, U, Wf] = Poly_gravity(r_A,Asteroid.rho,Asteroid.Polyhedron.Vertices,Asteroid.Polyhedron.Facets,Asteroid.Polyhedron.Edges,Asteroid.Polyhedron.F_tilde,Asteroid.Polyhedron.E_tilde,CONSTANTS.G);
    g_B = C_BA*g_A;
else
    r = norm(r_B);
    g_B = -(m*Asteroid.mu).*r_B(1:3)./r^3;
end

if Switch.GG == 1
    %Number of point masses: 80(20 block, 60 solar arrays)
    n_block = 8 + 12; %8 vertices, 12 facet centres
    n_SA = 40 + 20; % 4 vertices and 2 facet centres each on 10 panels

    %Mass of point masses
    m = [((m_n-150)/n_block).*(ones(n_block,1)); (150/n_SA).*(ones(n_SA,1))]; %mass of SA: 150kg 

    %Position of point masses wrt inertial frame
    r_pm = [ V(1:8,:); C(1:12,:); V(9:48,:); C(13:32,:) ];
    r_pm_B = r_pm + r_B';

    %Grav accl, gradient torque  of point masses wrt body frame
    for i = 1:length(m)
        r_pm_A(:,i) = C_BA'*r_pm_B(i,:)';
        if Switch.poly_grav == 1
            g_pm_A(:,i) = Poly_gravity_mex(r_pm_A(:,i),Asteroid.rho,Asteroid.Polyhedron.Vertices,Asteroid.Polyhedron.Facets,Asteroid.Polyhedron.Edges,Asteroid.Polyhedron.F_tilde,Asteroid.Polyhedron.E_tilde,CONSTANTS.G);
            g_pm_B(:,i) = C_BA*g_pm_A(:,i);
        else
            r = norm(r_pm_B(i,:));
            g_pm_B(:,i) = (-(m*mu).*r_pm_B(i,:)./r^3)';
        end
        F_g(i,:) = (m(i) .* g_pm_B(:,i))';
        T_GG(i,:) = cross(r_pm_B(i,:),F_g(i,:),2);
    end
    
    T_GG_B = sum(T_GG)';

else
    T_GG_B = [0;0;0;0];
end

end