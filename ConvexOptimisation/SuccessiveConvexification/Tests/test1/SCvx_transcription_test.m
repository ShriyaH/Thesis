function[xc,uc,xdotc,cpu_time,status] = SCvx_transcription_test()
% Asteroid descent problem for ECOS with Successive Convexification
% Shriya Hazra, 31-Jul-2018 
global CONSTANTS PARAMS ITR;

K = CONSTANTS.nodes;
rho0 = CONSTANTS.rho0;
rho1 = CONSTANTS.rho1;
rho2 = CONSTANTS.rho2;
Alpha = CONSTANTS.Alpha;
Beta = CONSTANTS.Beta;
w_vc = CONSTANTS.w_vc;

ns = PARAMS.n_state;
nc = PARAMS.n_control;
nv = PARAMS.n_virt;
nsl = PARAMS.n_slack;

ITR.v_k{1} = zeros(21,30);

%% TRANSCRIPTION - Successive Convexification
opts = ecosoptimset;
i = 1;
count = 1;

 while count < 10
    count = i;
    
    [A,b,G,h,C,dims] = SCvx_ini_para_test(i);
    
    [x, y, info_, s, z] = ecos(C, G, h, dims, A, b, opts);
    y =  -y(ns+(1:(K-1)*ns));

    if strcmp(info_.infostring,'Optimal solution found')
       status(i) = 1;
    else
       status(i) = 2;
    end

    cpu_time(i) = info_.timing.runtime;

    Xc = reshape(x,(ns+nc+nv+nsl),[]);
    xc = Xc(1:ns,:);
    for k = 1:K
        xc(8:11,k) = xc(8:11,k)./norm(xc(8:11,k));
    end
    uc = Xc(ns+1:ns+nc,:);
    vc = Xc(ns+nc+1:ns+nc+nv+nsl,:);
   
    for ii = 1:K-1
        xdotc(:,ii) = ITR.Ac_k{i}{ii}*xc(:,ii) + ITR.Bc_k{i}{ii}*uc(:,ii) + ITR.zc_k{i}{ii};
    end
    
    ITR.x_k{i+1} = xc;
    ITR.u_k{i+1} = uc;
    ITR.v_k{i+1} = vc;
    ITR.xdot_k{i+1} = xdotc;
    ITR.cost{i} = info_.pcost;
        
    ITR.m_spent(1,i) = ITR.x_k{i+1}(1,1)-ITR.x_k{i+1}(1,end);
        
    %% Trust region radius update
%     for ii = 1:K
%         norm_v1(ii) = w_vc*(ITR.v_k{i}(1:nv,ii)'*ITR.v_k{i}(1:nv,ii));
%         norm_v2(ii) = w_vc*(ITR.v_k{i+1}(1:nv,ii)'*ITR.v_k{i+1}(1:nv,ii));
%     end
%         
%     % Non linear penalty costs
%     J1 = -ITR.x_k{i}(1,end) + sum(norm_v1);
%     J2 = -ITR.x_k{i+1}(1,end) + sum(norm_v2);
%         
%     % Linear penalty cost
%     L = info_.pcost/10;
%         
%     % Actual change in the penalty cost
%     dN = J1 - J2; 
% 
%     % Predicted change by convex cost
%     dL = J1 - L;
%         
%     if dL~= 0
%        % Ratio
%        rho = dN/dL; 
%        ITR.rho(i) = rho;
% 
%        % Update trust region radius 
%        if rho <= rho0
%           ITR.eta_k{i} = ITR.eta_k{i}./Alpha; 
%             continue;
%        elseif rho > rho0 || rho < rho1
%           ITR.eta_k{i+1} = ITR.eta_k{i}./Alpha;
%        elseif rho1 <= rho || rho < rho2
%           ITR.eta_k{i+1} = ITR.eta_k{i};
%        elseif rho2 <= rho
%           ITR.eta_k{i+1} = Beta.*ITR.eta_k{i};
%        end  
% 	  else
%        ITR.eta_k{i+1} = ITR.eta_k{i};
%         ITR.eta_k{i+1} = ITR.eta_k{i};
%     end
    i = i+1           

 end
 acik_plots(i);
    
end


