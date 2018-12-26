% v = Cube.Polyhedron.Vertices;
% f = Cube.Polyhedron.Facets;
% s = sign(v);
% t = s*100e3;
% q = s*10e3;
% v2 = v+q; 
% v3 = v+t;
% G = 6.67408e-11;
% [Cube] = Get_Cube(60e3,30e3,40e3,G);
% x = randi([-80e3 80e3],1,1000);
% y = randi([-50e3 50e3],1,1000);
% z = randi([-60e3 60e3],1,1000);
% r = [x;y;z];
% 
% % figure() 
% % hold on
% % axis equal
% % set(gca, 'Box','off', 'Projection','perspective','color','black', 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w')
% %     set(gcf,'color','black');
% %     axis off
% %  axis equal   
% % for t = 1:1:50
% %     y = -rad2deg(Cube.w_AI(3)*t*60);
% % %     x.new(:,t) = cross_quat(Frames.quat.Q_AI(1:4,t),cross_quat(x.start,conj_quat(Frames.quat.Q_AI(1:4,t))));
% % %     c(:,:,t) = [zeros(length(Cube.Polyhedron.Facets),1) Sun.gradient(:,t) Sun.gradient(:,t)];
% %     
% %     patch('Vertices',v,'Faces',f,'FaceVertexCData',[0 1 1],'FaceColor','flat');
% %     patch('Vertices',v2,'Faces',f,'FaceColor',[1 1 1],'FaceAlpha',0.5,'EdgeColor','none');
% %     patch('Vertices',v3,'Faces',f,'FaceColor',[0 1 0],'FaceAlpha',0.1,'EdgeColor','none');
% % plot3(U_sph(:,2)+60e3+20e3,U_sph(:,3)+30e3+20e3,U_sph(:,4)+40e3+20e3,'.');
% %     for i=1:500
% %     comet3(x_A(1,1:50,i),x_A(2,1:50,i),x_A(3,1:50,i))
% %     end
% % scatter3(x,y,z)
% % 
% % hold off
% 
% for i =1:1000
% 
% [g_A(:,i), U(:,i), Wf(:,i)] = Poly_gravity(r(:,i),Cube.rho,Cube.Polyhedron.Vertices,....
%     Cube.Polyhedron.Facets,Cube.Polyhedron.Edges,Cube.Polyhedron.F_tilde,Cube.Polyhedron.E_tilde,G);
% end
% figure()
% hold on
% patch('Vertices',v,'Faces',f,'FaceVertexCData',[0 1 1],'FaceColor','flat');
% quiver3(r(1,:),r(2,:),r(3,:),g_A(1,:),g_A(2,:),g_A(3,:))


N=3;  
E1=60e3;
E2=30e3;
E3 = 40e3;
n=1000; 

d=randi(N,1,n);  
s =randi(2,1,n)-1;  
v=rand(N,n);  

for i=1:n  
  v(d(i),i)=s(i);  
end 
 
v(1,:) = E1*v(1,:);
v(2,:) = E2*v(2,:);
v(3,:) = E3*v(3,:);
figure()
plot3(v(1,:),v(2,:),v(3,:),'.','Color','r'); 


