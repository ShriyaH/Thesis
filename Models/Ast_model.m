function [] = Ast_model(Asteroid)
%% Asteroid Model plot
%plots asteroid with landmarks, their normals and the SC position.

% [Asteroid] = Get_Asteroid('Asteroid',1);   %Get the asteroid polyhedron and properties
% Asteroid.n_lm = 20;                                %number of landmarks
% [Asteroid.lm_coord,Asteroid.lm_n,Asteroid.lm_r] = Get_landmarks(Asteroid,Asteroid.n_lm);
SC_pos = [105e3;40e3; 50e3];
scale = 1;
scale2 = 500e3;

figure()
patch('Vertices',Asteroid.Polyhedron.Vertices,'Faces',Asteroid.Polyhedron.Facets,'FaceVertexCData',[0.99 0.99 0.99],'FaceAlpha',0.8, 'EdgeColor', [0.5 0.5 0.5]);
goodplot
hold on
for i = 1:Asteroid.n_lm
plotCircle3D(Asteroid.lm_coord (i,:),Asteroid.lm_n (i,:),Asteroid.lm_r(1,i))
end
for i = 1:Asteroid.n_lm
quiver3(Asteroid.lm_coord(i,1),Asteroid.lm_coord(i,2),Asteroid.lm_coord(i,3),Asteroid.lm_n(i,1),Asteroid.lm_n(i,2),Asteroid.lm_n(i,3),scale2,'Color','red','Linewidth',1);
end
quiver3(0,0,0,150e3,0,0,scale,'Color','red','Linewidth',2);
quiver3(0,0,0,0,50e3,0,scale,'Color','green','Linewidth',2);
quiver3(0,0,0,0,0,50e3,scale,'Color','blue','Linewidth',2);
quiver3(0,0,0,SC_pos(1),SC_pos(2),SC_pos(3),scale,'Color','yellow','Linewidth',2);
grid on
axis equal
xlabel('X-axis (m)');
ylabel('Y-axis (m)');
zlabel('Z-axis (m)');
end
