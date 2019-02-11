function [] = Ast_model(Asteroid,SC,dqi,dqn,dq_form)
%% Asteroid Model plot
%plots asteroid with landmarks, their normals and the SC position.

% [Asteroid] = Get_Asteroid('Asteroid',1);   %Get the asteroid polyhedron and properties
% Asteroid.n_lm = 20;                                %number of landmarks
% [Asteroid.lm_coord,Asteroid.lm_n,Asteroid.lm_r] = Get_landmarks(Asteroid,Asteroid.n_lm);

SC_posi = DQ2R(dqi,dq_form);
SC_posn = DQ2R(dqn,dq_form);
v = SC.Polyhedron.Vertices;
f = SC.Polyhedron.Facets;

scale2 = 50e3;

for i = 1:length(v)
    qi = dqi(1:4);
    qn = dqn(1:4);
    vi(i,:) = quat_trans(qi,v(i,:),'vect');
    vi(i,:) = vi(i,:) + SC_posi(1:3)'; 
    vn(i,:) = quat_trans(qn,v(i,:),'vect');  
    vn(i,:) = vn(i,:) + SC_posn(1:3)'; 
end

figure()
patch('Vertices',Asteroid.Polyhedron.Vertices,'Faces',Asteroid.Polyhedron.Facets,'FaceVertexCData',[0.99 0.99 0.99],'FaceAlpha',0.8, 'EdgeColor', [0.5 0.5 0.5]);
hold on
patch('Vertices',vi,'Faces',f,'FaceVertexCData',[0.99 0.99 0.99],'FaceAlpha',0.8, 'EdgeColor', [0.5 0.5 0.5]);
patch('Vertices',vn,'Faces',f,'FaceVertexCData',[0.99 0.99 0.99],'FaceAlpha',0.8, 'EdgeColor', [0.5 0.5 0.5]);
% for i = 1:Asteroid.n_lm
plotCircle3D(Asteroid.lm_coord (1,:),Asteroid.lm_n (1,:),Asteroid.lm_r(1,1))
% end
% for i = 1:Asteroid.n_lm
quiver3(Asteroid.lm_coord(1,1),Asteroid.lm_coord(1,2),Asteroid.lm_coord(1,3),Asteroid.lm_n(1,1),Asteroid.lm_n(1,2),Asteroid.lm_n(1,3),scale2,'Color','red','Linewidth',1);
% end
grid on
axis equal
xlabel('X-axis (m)');
ylabel('Y-axis (m)');
zlabel('Z-axis (m)');
hold off
end
