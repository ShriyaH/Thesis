function [] = spin(Asteroid,Sun)
% spin.m: Rotates a view around the equator one revolution
% in 5-degree steps. Negative step makes it rotate normally
% (west-to-east).

for j = 1: length (Sun.t)
    y = -rad2deg(Asteroid.w_AI(3)*Sun.t (j));
 
     plot_Ast(Asteroid.Polyhedron,Sun.grad(:,:,j))
% %     comet3(Frames.Orbit.x(:,1),Frames.Orbit.x(:,2),Frames.Orbit.x(:,3))
  	view(y,Asteroid.axis(2));     %  Asteroid's axial tilt
 	drawnow
     set(gca, 'Box','off', 'Projection','perspective')
     set(gcf,'color','black');
     axis off;
     axis vis3d
% end



% plot_Ast(Kleopatra.Polyhedron,Out(:,:,1))
% a1 = nf(:,1,1);
% a2 = nf(:,2,1);
% a3 = nf(:,3,1);
% b1 = R1(:,1,1);
% b2 = R1(:,2,1);
% b3 = R1(:,3,1);
% scale=5;
% quiver3(b1,b2,b3,a1,a2,a3,scale)

end