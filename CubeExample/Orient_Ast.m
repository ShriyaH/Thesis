% function []= Orient_Ast(Asteroid,Traj)
% 
t = Traj.t;
v = Cube.Polyhedron.Vertices;
v = [v zeros(length(v),1)];
f = Cube.Polyhedron.Facets;

for i = 1:n_samp
    for j = 1:length(t{i})
        q{i}(1:4,j) = [0; 0; sin(Cube.w_AI(3)*t{i}(j)/2); cos(Cube.w_AI(3)*t{i}(j)/2)];
        for k = 1:length(v)
            v_new{i}{j}(k,:) = cross_quat(q{i}(:,j),cross_quat(v(k,:)',conj_quat(q{i}(:,j))));
        end
    end
end

    
 

% v = SC.Polyhedron.Vertices;
% v = [v zeros(length(v),1)];
% q = Q_rot;
% 
% for i = 1:50
%     for j = 1:length(q{i})
%         for k = 1:length(v)
%             v_new{i}{j}(k,:) = cross_quat(q{i}(j,:)',cross_quat(v(k,:)',conj_quat(q{i}(j,:)')));
%         end
%         v_new{i}{j} = 1000*v_new{i}{j}(:,1:3) + y{i}(j,1:3);
%     end
% end
% end