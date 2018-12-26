clc
clear data
% % tic
% % Generate asteroid data file 
[CG] = Get_Asteroid('CG',1);
% G = 6.67408e-11;
% mu_s = 1.32712440018e20; %standard gravitational parameter of the sun
% T.tf = 2*86400;    % Propagate for two days
% T.nsteps = T.tf/60;  % Integration per minute
% T.t0 = 0;
% x = [0.866025403784439; 0.866025403784439; 0.866025403784439; 0.5];
% y = [0.5; 0.5; 0.5; 0.866025403784439];
% z = cross_quat(y,conj_quat(x))
% z = cross_quat(x,conj_quat(y))
% [g_A, U, Wf] = Poly_gravity([300e3,300e3,300e3], Kleopatra);
% [Frames]= Ref_frames(Kleopatra);
% [Sun] = Get_Sun(Kleopatra,Frames);
% n = 4000;
% [Kleopatra.lm_coord,Kleopatra.lm_n,Kleopatra.lm_r] = Get_landmarks(Kleopatra,n);
% [Imp_Ellipse] = MinVolEllipse(Kleopatra.Polyhedron.Vertices', 0.001);
% figure()
% hold on
% surf(Imp_Ellipse.XX,Imp_Ellipse.YY,Imp_Ellipse.ZZ,'edgecolor', 'w');  %Plot impact ellipsoid
%  plot_Ast(Kleopatra.Polyhedron);
% for i=1:1:n
% plotCircle3D(Kleopatra.lm_coord(i,:), Kleopatra.lm_n(i,:), Kleopatra.lm_r(i)) 
% end
% hold off
%Plot landmarks on asteroid
% % [x, y, z] = ellipsoid(0,0,0,Kleopatra.Polyhedron.ellipsoid(1,1)+15e3,Kleopatra.Polyhedron.ellipsoid(1,2)+20e3,Kleopatra.Polyhedron.ellipsoid(1,3)+15e3,30);
% SL = [226e3;97.5e3;87.8e3];  
% CV = [0;0;0];
% figure()
% % hold on
% [CuboidHandle, verts, facs] = DrawCuboid(SL,CV);
% axis equal
% plot_Ast(Kleopatra.Polyhedron,Sun.gradient);
% quiver3(Kleopatra.Polyhedron.x(:,1),Kleopatra.Polyhedron.x(:,2),Kleopatra.Polyhedron.x(:,3),Kleopatra.Polyhedron.normalsf(:,1),Kleopatra.Polyhedron.normalsf(:,2),Kleopatra.Polyhedron.normalsf(:,3));
% % surf(x, y, z)
% % alpha 0.3
% hold off
% % figure('units','normalized','outerposition',[0 0 1 1])
% % spin(Kleopatra,Sun);
% % toc



% view(3)
% axis vis3d