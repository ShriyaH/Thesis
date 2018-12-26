% function [Domain_update] = Eligible_traj(Cube,Sun,SC,Traj,Orient)
t = State.t;
y = State.y;
q_AI = State.q_AI;
q_BI = State.q_BI;
r_HI = Sun.pos;
nf = Cube.Polyhedron.normalsf;
r1 = Cube.Polyhedron.x;
r2 = Cube.Polyhedron.y;
r3 = Cube.Polyhedron.z;
r_fA = (r1 + r2 + r3)/3;  %Position vec of centroid of facet

%% Asteroid Facet Illumination and SC Phase Angle Scores

for i = 1:length(q_AI)
    for j = 1:length(q_AI{i})
        r_HA{i}(j,1:3) = quat_trans(q_AI{i}(:,j),r_HI(:,1),'vect');
        r_A{i}(j,1:3) = quat_trans(q_AI{i}(:,j),[y{i}(j,1:3)';0],'vect'); 
        
        phase{i}(j,:) = dot(r_A{i}(j,1:3),r_HA{i}(j,1:3))./(norm(r_A{i}(j,1:3))*norm(r_HA{i}(j,1:3)));    %Phase angle
                
        for k = 1:length(r_fA)
            r_Hf{i}{j}(k,:) = r_HA{i}(j,:) - r_fA(k,1:3);                 %Vector to sun from facet
            dir{i}{j}(k,:) = r_Hf{i}{j}(k,1:3)./norm(r_Hf{i}{j}(k,1:3));
            obs{i}{j}(k,:) = dot(nf(k,1:3),dir{i}{j}(k,:));               %angle between facet normal and sun vec from facet            
        end
        
        %% Normalize score to 0.25
        o{i}{j}(:,:) = obs{i}{j}(:,:) + 1; %change mean to 1
        p{i}{j} = max(o{i}{j}(:,:));
        factor{i}{j} = 0.25/p{i}{j};    
        facet_score{i}{j}(:,:) = o{i}{j}(:,:).*factor{i}{j};  %facet illumination score
    end
%         n{i}(:,:) = phase{i}(:,:) + 1; %change mean to 1
%         m{i} = max(n{i}(:,:));
%         factor{i} = 0.25/m{i};            
end
% phase_score = phase.*0.25;     %phase angle score
%% SC Phase Angle Score

% r_BH = Sun.x_BH;
% r_BH = Orient.r_HB
% r_IH = Sun.pos;
% 
% for j = 1:length(r_HA)
%     phase(j,1) = dot(r_BH(j,:),r_IH(j,:))/(norm(r_BH(j,:))*norm(r_IH(j,:)));
%     phase_angle(j,1) = rad2deg(acos(phase(j,1)));
%     h(j,1) = (phase(j,1) + 1);
%     p_score(j,1) = h(j,1)/8;
%     graph2(j,:) = p_score(j,1)';
% end

% Sun.pos_rel = r_HA;
% Sun.angle = view_angle;
% Sun.dir_f =  dir;
% 
% Scores.obs_score = o_score;
% 
% 
% % figure(1)
% % hold on
% % plot(1:2880,graph1(1:2880,1:12))
% % xlabel('Time (mins)')
% % ylabel('Observation Score')
% % legend('Facet 1','Facet 2','Facet 3','Facet 4','Facet 5','Facet 6','Facet 7','Facet 8','Facet 9','Facet 10','Facet 11','Facet 12')
% % hold off 

% Sun.posAU=Sun.pos/AU;
% Sun.flux=Sun.flux/Sun.posAU^2;
% Scores.phase_score = p_score;
% 
% figure(2)
% hold on
% plot(1:2880,graph2(1:2880,1:12))
% hold off 
% 
% 
% %% Observing Facets
% 
% c = 3e8; % Speed of light
% dist_nominal = 15e3; % Altitude from surface, m
% t_min_travel = 2*dist_nominal/c;
% patch_area = 10; % LR patch area, m^2
% time_patch = (t_min_travel*Polyhedron.A_facet/patch_area)/60;
% T_min_obs = 20*60; % Minimum observation time of each facet
% [r c] = size(Polyhedron.Facets);