function [] = spin(Object,Sun,nsteps)
% spin.m: Rotates a view around the equator one revolution
% in 5-degree steps. Negative step makes it rotate normally
% (west-to-east).

for t = 1:1:nsteps
    y = -rad2deg(Object.w_AI(3)*t*60);
    f(:,:,t) = [zeros(length(Object.Facets),1) Sun.gradient(:,t) Sun.gradient(:,t)];
    patch('Vertices',Object.Vertices,'Faces',Object.Facets,...
          'FaceVertexCData',f(:,:,t),'FaceColor','flat');
%      plot_Ast(Object.Polyhedron,Sun.grad(:,:,j))
    view(y,50);     %  Asteroid's axial tilt
 	drawnow
    set(gca, 'Box','off', 'Projection','perspective')
    set(gcf,'color','black');
    axis off;
    axis vis3d
 end


% Object figure
% figure()
% hold on
% patch('Vertices',Object.Vertices,'Faces',Object.Facets,...
%       'FaceVertexCData',[0.9 1 0.9],'FaceColor','flat')
% quiver3([0,0,0],[0,0,0],[0,0,0],[Object.dim(1)+30e3,0,0],[0,Object.dim(2)+30e3,0],[0,0,Object.dim(3)+30e3]);
% for i =1:12
% line([0 x_fC(i,1)],[0,x_fC(i,2)],[0,x_fC(i,3)],'color','r');
% end
% 
% xlabel('X')
% ylabel('Y')
% zlabel('Z')
% view(3)
% axis equal
% hold off

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