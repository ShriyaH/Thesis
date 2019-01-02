%plot SC
figure()
patch('Vertices',SC.Polyhedron.Vertices,'Faces',SC.Polyhedron.Facets,'FaceVertexCData',[0.9 0.9 0.9],'FaceColor','flat');
hold on
quiver3(0,0,0,4,0,0,'Color', 'r','Linewidth',1.5)
quiver3(0,0,0,0,15,0,'Color', 'g','Linewidth',1.5)
quiver3(0,0,0,0,0,4,'Color', 'b','Linewidth',1.5)
xlabel('X-axis (m)')
ylabel('Y-axis (m)')
zlabel('Z-axis (m)')
axis equal