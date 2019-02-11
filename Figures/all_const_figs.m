%Generate figures for all constraints

% %Glide-slope
[ Kleopatra ] = Get_Asteroid( 'Kleopatra', 1 );
[F,V]=stlread('Rosetta.stl');
r=[10e3,50e3,50e3];
V = 500.*V+r;
% t = [0;1];
% [X,Y,Z] = cylinder(t);
% x= 50.*X;
% y=50.*Y;
% z=50.*Z; 

% figure();
% clf;
% hold on
% patch('Vertices',Kleopatra.Polyhedron.Vertices,'Faces',Kleopatra.Polyhedron.Facets,'FaceVertexCData',[0.7 0.7 0.7],'FaceColor','flat','FaceAlpha',0.9);
% % patch('Vertices',V,'Faces',F,'FaceVertexCData',[0.7 0.7 0.7],'FaceColor','flat','FaceAlpha',0.9);
% quiver3(0,0,0,200e3,0,0,'Color','r','Linewidth',2);
% quiver3(0,0,0,0,200e3,0,'Color','g','Linewidth',2);
% quiver3(0,0,0,0,0,200e3,'Color','b','Linewidth',2);
% quiver3(0,0,0,10e3,70e3,120e3,'Color','k','Linewidth',2);
% % surf(x,y,z);
% % alpha(.5)
% xlabel('X-axis (m)')
% ylabel('Y-axis (m)')
% zlabel('Z-axis (m)')
% axis equal
% grid on
% hold on

% %LOS

% [F,V]=stlread('Rosetta.stl');
% V(:,:)=V(:,:)./2;
% X1=[1.2 -1.5 2];
% X2=[0 -5 10];
% r=[0 3];
% n=20;
% cyl_color='g';
% closed=0;
% Cone(-X1,-X2,r,n,cyl_color,closed,lines)
% alpha(0.5)
% hold on
% patch('Vertices',V,'Faces',F,'FaceVertexCData',[0.7 0.7 0.7],'FaceColor','flat','FaceAlpha',0.9);
% quiver3(0,0,0,25,0,0,'Color','r','Linewidth',2);
% quiver3(0,0,0,0,10,0,'Color','g','Linewidth',2);
% quiver3(0,0,0,0,0,5,'Color','b','Linewidth',2);
% xlabel('X-axis (m)')
% ylabel('Y-axis (m)')
% zlabel('Z-axis (m)')
% axis equal
% grid on

%Gimbal
% t = [0;1];
% [X,Y,Z] = cylinder(t);
% x= 500*(3.*X)+r(1);
% y=500*(3.*Y)+r(2);
% z=100*(10.*Z)+r(3); 
% % V(:,:)=V(:,:)./2;
% 
% figure();
% % clf;
% hold on
% surf(-x,-y,-z);
% hold on
% alpha(.5)
% hold on
% patch('Vertices',V,'Faces',F,'FaceVertexCData',[0.7 0.7 0.7],'FaceColor','flat','FaceAlpha',0.9);
% hold on
% quiver3(0,0,0,25,0,0,'Color','r','Linewidth',2);
% quiver3(0,0,0,0,5,0,'Color','g','Linewidth',2);
% quiver3(0,0,0,0,0,5,'Color','b','Linewidth',2);
% xlabel('X-axis (m)')
% ylabel('Y-axis (m)')
% zlabel('Z-axis (m)')
% axis equal
% grid on

