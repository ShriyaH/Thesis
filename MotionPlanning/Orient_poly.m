function [Orient] = Orient_poly(Asteroid,SC,Traj,Sun)
%% Get the SC body reference frame

y = Traj.y;
t = Traj.t;
q_AI = Traj.q_AI;
r_HI = Sun.pos';
ang_s = SC.th_sa_forb;

for i = 1:length(y)
    for j = 1:length(y{i})
            rho{i}(j,1) = norm(y{i}(j,1:3));                %range of SC
            z_B{i}(j,:) = -y{i}(j,1:3)./rho{i}(j,1);        %nadir pointing
            
            vel{i}(j,1) = norm(y{i}(j,4:6));                %velocity magnitude of SC
            v_B{i}(j,:) = y{i}(j,4:6)./vel{i}(j,1);  
        
            y_B{i}(j,:) = cross(z_B{i}(j,:),v_B{i}(j,:),2); %axis perpendicular to orbital plane
            y_B{i}(j,:) = y_B{i}(j,:)./norm(y_B{i}(j,:));
                  
            x_B{i}(j,:) = cross(y_B{i}(j,:),z_B{i}(j,:),2); %right handed x axis (y cross z)
            x_B{i}(j,:) = x_B{i}(j,:)./norm(x_B{i}(j,:)); 
            
             %% Check if the solar panels are aligned to the sun within certain angle
 
            r_HB{i}(j,:) = r_HI(1,1:3) - y{i}(j,1:3);             % position of sun wrt SC
            sun_angle{i}(j,:) = acos(dot(x_B{i}(j,:),r_HB{i}(j,:))./norm(r_HB{i}(j,:)));
            
            if sun_angle{i}(j,:) < ang_s || sun_angle{i}(j,:) > pi-ang_s 
               a = ang_s + 0.0349;  % 2 degrees more than needed to be safe
               
               q_sun = [0;0;sin(a/2);cos(a/2)]; % rotate about z axis to get sun on solar array
               
               y_B{i}(j,:) = quat_trans(q_sun,y_B{i}(j,:),'vect');
               x_B{i}(j,:) = quat_trans(q_sun,x_B{i}(j,:),'vect'); 
            else
                x_B{i}(j,:) = x_B{i}(j,:);
                y_B{i}(j,:) = y_B{i}(j,:);
            end
            
            DCM_BI{i}{j} = [x_B{i}(j,:);y_B{i}(j,:);z_B{i}(j,:)];  % DCM from inertial to body frame
            q_BI{i}(j,:) = DCM2Q(DCM_BI{i}{j});                    % quaternion for body wrt inertial
            qc_BI{i}(j,:) = conj_quat(q_BI{i}(j,:)');
 
            
    end
end

%% Orient SC and Asteroid polyhedrons

v = SC.Polyhedron.Vertices;
v = [v zeros(length(v),1)];
v1 = Asteroid.Polyhedron.Vertices;
v1 = [v1 zeros(length(v1),1)];


for i = 1:1:length(t)
    for j = 1:length(t{i})
               
        for k = 1:length(v)
            v_new{i}{j}(k,:) = quat_trans(qc_BI{i}(j,:)',v(k,:)','vect');    
        end
      
        for l = 1:length(v1)
            v1_new{i}{j}(l,:) = quat_trans(q_AI{i}(:,j),v1(l,:)','vect');
        end
        
        q_BA{i}(:,j) = cross_quat(q_BI{i}(j,:)',conj_quat(q_AI{i}(:,j)));
        
        v_new{i}{j} = 1000*v_new{i}{j}(:,1:3) + y{i}(j,1:3);       
        v1_new{i}{j} = v1_new{i}{j}(:,1:3);
    end
end


Orient.rho = rho;
Orient.vel = vel;
Orient.DCM = DCM_BI;
Orient.q_BI = q_BI;
Orient.q_BA = q_BA;
Orient.q_AI = q_AI;
Orient.r_HB = r_HB;
Orient.SC_V_new = v_new;
Orient.Ast_V_new = v1_new;


end