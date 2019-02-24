%%SBMPO Asteroid Mapping

%% Initialise
Mc = 0;
i = 1;
n_mesh = 50;
dv_max = 10;

%% SBMPO Algorithm
while Mc<100
    for j = 1:n_mesh
        if j ==1
            DeltaV(j) = Ini_mesh(dv_max,n(0));
            C = 0;
        else
            DeltaV(j) = Refine_mesh(DeltaV,C,dv_max,nv(j));
        end
        for k = 1:nv
            T(k) = Propagate_traj(dv(k),t(i),t(i+1));
            C(k) = Compute_score(T(k));
        end
    end
    deltav_opt(i) = Compute_Opt_Man(C,DeltaV);
    Mc = Mission_Completion(n_f,t_obs(i),t_min);
    i=i+1
end
