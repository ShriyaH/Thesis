function[tc,xc,uc,xdotc,cpu_time,status] = SCvx_transcription()
% Asteroid descent problem for ECOS with Successive Convexification
% Shriya Hazra, 31-Jul-2018 
global CONSTANTS MAIN PARAMS ITR;

rho0 = CONSTANTS.rho0;
rho1 = CONSTANTS.rho1;
rho2 = CONSTANTS.rho2;
Alpha = CONSTANTS.Alpha;
Beta = CONSTANTS.Beta;
eta = CONSTANTS.eta0;

[A,b,G,h,C,dims] = SCvx_initialise_parameters();

G = sparse(G);
A = sparse(A);
opts = ecosoptimset;
ns = PARAMS.n_state;
nc = PARAMS.n_control;
nv = PARAMS.n_virt;
nsl = PARAMS.n_slack;
K = CONSTANTS.nodes;
eta = CONSTANTS.eta0; 
E_nu = zeros(20,3);
E_nu(18:20,1:3) = eye(3);

%% TRANSCRIPTION - Successive Convexification
 for i = 1:10

        [x, y, info_, s, z] = ecos(C, G, h, dims, A, b, opts);
         y =  -y(ns+(1:(K-1)*ns));

        if strcmp(info_.infostring,'Optimal solution found')
            status = 1;
        else
            status = 2;
        end

        cpu_time = info_.timing.runtime;

        Xc = reshape(x,(ns+nc+nv+nsl),[]);
        xc = Xc(1:ns,:);
        uc = Xc(ns+1:ns+nc,:);
        vc = Xc(ns+nc+1:ns+nc+nv,:);
        sc = Xc(ns+nc+nv+1:ns+nc+nv+nsl,:);
        for ii = 1:K-1
             xdotc{i}(:,ii) = ITR.Ac_k{i}{ii}*xc(:,ii) + ITR.Bc_k{i}{ii}*uc(:,ii) + ITR.zc_k{i}{ii};
        end
    
        ITR.x_k{i+1} = xc;
        ITR.u_k{i+1} = uc;
        ITR.v_k{i+1} = vc;
        ITR.xdot_k{i+1} = xdotc;
        
%         ITR.m_spent(1,i) = sum(ITR.x_k{i+1}{}-ITR.x_k{i+1}(1,end));
%         disp(['mass spent: ',num2str(ITR.m_spent),' kg'])
%         
        %% Compute the cost ratio
        d = ITR.x_k{i+1} - ITR.x_k{i};
        w = ITR.u_k{i+1} - ITR.u_k{i};
        
        acik_plots;
        % Non linear penalty cost
        
        J = ITR.x_k{i+1}(1,end) - ITR.x_k{i+1}(1,1);
        % Non linear 
        % Actual change in the penalty cost
        
        % Predicted change by convex cost
        
        % Ratio
%         rho = C/C; 
%       
%     %% Update trust region radius 
%         if rho < rho0
%             eta = eta./Alpha;  
%             [A,b,G,h,C,dims] = SCvx_reevaluate_parameters(i,eta,ITR.x_k{i-1},ITR.u_k{i-1});
%             return
%         elseif rho < rho1
%             eta = eta./Alpha;
%         elseif rho2 <= rho
%             eta = Beta.*eta;
%         else
%             break
%         end  
% 
%     % Revaluate all matrices with the new solution and trust region
%         [A,b,G,h,C,dims] = SCvx_reevaluate_parameters(i,eta,xc,uc);
%         
%         G = sparse(G);
%         A = sparse(A);
    end


