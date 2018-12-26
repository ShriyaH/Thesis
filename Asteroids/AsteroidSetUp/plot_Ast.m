function [] = plot_Ast(Polyhedron)

% for i=1:length(normalsf)
%     obs_angle = acos(dot(sun_direction,normalsf));
% end
% light('Position',[X(i,1) X(i,2) X(i,3)]);
grey=[0.9 0.9 0.9];
AST.Vertices = Polyhedron.Vertices;
AST.Faces = Polyhedron.Facets;
%  f = [zeros(length(obs_ratio),1) obs_ratio(:,:,1) obs_ratio(:,:,1)];
%  AST.FaceVertexCData = f;
AST.FaceVertexCData = [1 1 1];
 AST.FaceColor = 'flat';
AST.EdgeColor = grey*0.9;
AST.LineWidth = 0.5;

patch(AST)
axis equal
% patch(AST,'FaceLighting','gouraud','FaceColor','interp',...
%       'AmbientStrength',0.5);
hold on
end

