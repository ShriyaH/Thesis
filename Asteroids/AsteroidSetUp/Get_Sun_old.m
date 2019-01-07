function [Sun] = Get_Sun(Asteroid,Frames)

mu_s = 1.32712440018e20;

tspan = [0 15*3600];
[X,theta_rad,t] = Propagate_Orbit( Asteroid.kep_orbit, Frames.Orbit.state, mu_s,tspan);

F = Asteroid.Polyhedron.Facets;
t_I = [Asteroid.Polyhedron.Vertices';zeros(1,length(Asteroid.Polyhedron.Vertices'))];

 for l = 1:length(t) 
    Q_RI(:,l) = [0; 0; sin(Asteroid.w_AI(3)*t(l)/2); cos(Asteroid.w_AI(3)*t(l)/2)];
    
    for j = 1:length(t_I)
        new_Vertices(:,j,l) = cross_quat(Q_RI(:,l),cross_quat(t_I(:,j),q_conjugate(Q_RI(:,l))));
    end
    
    V(:,:,l) = permute(new_Vertices(1:3,:,l),[2 1 3]);  
     
    for i=1:1:(size(F,1))
      
    R1(i,:,l)=V(F(i,1),:,l);    
    R2(i,:,l)=V(F(i,2),:,l);    
    R3(i,:,l)=V(F(i,3),:,l); 

    R12(i,:,l)=R2(i,:,l)-R1(i,:,l);
    R23(i,:,l)=R3(i,:,l)-R2(i,:,l);  
    R31(i,:,l)=R1(i,:,l)-R3(i,:,l);

    nf(i,:,l)=cross(R12(i,:,l),R23(i,:,l));
    nf(i,:,l)=nf(i,:,l)/norm(nf(i,:,l)); 
    
%     if dot(nf(i,:,l), R1(i,:,l))<0 && dot(nf(i,:,l), R3(i,:,l))<0&& dot(nf(i,:,l), R2(i,:,l))<0
%         nf(i,:,l)=-nf(i,:,l);
%     end
    
    C(i,1,l) = (R1(i,1,l)+R2(i,1,l)+R3(i,1,l))/3;
    C(i,2,l) = (R1(i,2,l)+R2(i,1,l)+R3(i,1,l))/3;
    C(i,3,l) = (R1(i,3,l)+R2(i,1,l)+R3(i,1,l))/3;
    
    S_vec(i,:,l) = C(i,:,l)-X(l,:);
    o(i,:,l) = dot(nf(i,:,l),S_vec(i,:,l));
    o_norm(i,:,l) = norm(nf(i,:,l))*norm(S_vec(i,:,l));
    sun_angle(i,:,l) = rad2deg(acos(o(i,:,l)/o_norm(i,:,l)));
    
    obs_ratio(i,:,l) = 1-(sun_angle(i,:,l)/180);
    grad(i,:,l)=fix(obs_ratio(i,:,l)*10^2)/10^2;
    end
end
    Sun.centroids = C;
    Sun.sun_angle = sun_angle;
    Sun.gradient = grad;
    Sun.time = t;
end