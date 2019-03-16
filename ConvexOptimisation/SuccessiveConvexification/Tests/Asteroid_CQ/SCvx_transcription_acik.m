function[xc,uc,xdotc,cpu_time,status] = SCvx_transcription_acik()
% Asteroid descent problem for ECOS with Successive Convexification
% Shriya Hazra, 31-Jul-2018 
global CONSTANTS PARAMS ITR;

K = CONSTANTS.nodes;
rho0 = CONSTANTS.rho0;
rho1 = CONSTANTS.rho1;
rho2 = CONSTANTS.rho2;
Alpha = CONSTANTS.Alpha;
Beta = CONSTANTS.Beta;
tol = CONSTANTS.tol;
g = CONSTANTS.g;

ns = PARAMS.n_state;
nc = PARAMS.n_control;
nv = PARAMS.n_virt;
nsl = PARAMS.n_slack;
nt = PARAMS.n_tr;
n = ns+nc+nv+nsl+nt;

ITR.v_k{1} = zeros(nv+nsl,K);
ITR.eta_k{1} = zeros(1,K);
ITR.S{1} = 0;
ITR.ETA{1} = 0;

%% TRANSCRIPTION - Successive Convexification
opts = ecosoptimset('MAXIT',200, 'FEASTOL',1e-6,'RELTOL',1e-6); 
% opts = ecosoptimset();
i = 1;
count = 1;

 while count < CONSTANTS.i_max
    count = i;
    
    [A,b,G,h,C,dims] = SCvx_ini_para_acik(i);
    
    [x, y, info_, s, z] = ecos(C, G, h, dims, A, b, opts);
    y =  -y(ns+(1:(K-1)*ns));

    if strcmp(info_.infostring,'Optimal solution found')
       status(i) = 1;
    else
       status(i) = 2;
    end

    cpu_time(i) = info_.timing.runtime;
    
    ITR.S{i+1} = x(end-1);
    ITR.ETA{i+1} = x(end);
    x = x(1:end-2);
    Xc = reshape(x,(ns+nc+nv+nsl+nt),[]);
    xc = Xc(1:ns,:);
    % quaternion normalisation
    for k = 1:K
        xc(8:11,k) = xc(8:11,k)./norm(xc(8:11,k));
    end
    uc = Xc(ns+1:ns+nc,:);
    vc = Xc(ns+nc+1:ns+nc+nv+nsl,:);
    eta = Xc(ns+nc+nv+nsl+1:ns+nc+nv+nsl+nt,:);
   
    for ii = 1:K-1
        xdotc(:,ii) = ITR.Ac_k{i}{ii}*xc(:,ii) + ITR.Bc_k{i}{ii}*uc(:,ii) + ITR.zc_k{i}{ii};
        xdotc_prev(:,ii) = ITR.Ac_k{i}{ii}*ITR.x_k{i}(:,ii) + ITR.Bc_k{i}{ii}*ITR.u_k{i}(:,ii) + ITR.zc_k{i}{ii};
    end
    
    ITR.x_k{i+1} = xc;
    ITR.u_k{i+1} = uc;
    ITR.v_k{i+1} = vc;
    ITR.eta_k{i+1} = eta;
    ITR.xdot_k{i+1} = xdotc;
    ITR.cost{i} = info_.pcost;
        
    ITR.m_spent(1,i+1) = ITR.x_k{i+1}(1,1)-ITR.x_k{i+1}(1,end);
        
    %% Trust region penalty weight update
%     ITR.state_conv{i} = sum(sum(abs((ITR.x_k{i+1}-ITR.x_k{i})))) + sum(sum(abs((ITR.u_k{i+1}-ITR.u_k{i}))));
%     ITR.virt_cont{i} = sum(abs(ITR.v_k{i+1}(end,:)));
    for ii = 1:K 
        ITR.st_conv{i}(ii) = sqrt((ITR.x_k{i+1}(:,ii)-ITR.x_k{i}(:,ii))'*(ITR.x_k{i+1}(:,ii)-ITR.x_k{i}(:,ii)) + (ITR.u_k{i+1}(:,ii)-ITR.u_k{i}(:,ii))'*(ITR.u_k{i+1}(:,ii)-ITR.u_k{i}(:,ii)));   
        
    end
    ITR.state_conv{i} = norm(ITR.st_conv{i});
    ITR.virt_cont{i} = norm(ITR.v_k{i+1}(end,:));

     if ITR.state_conv{i} < tol && ITR.virt_cont{i} < tol
          ITR.w_tr{i+1} = ITR.w_tr{i};
     else
        % Non linear penalty costs
        J1 = -ITR.x_k{i+1}(1,end);
        J2 = -ITR.x_k{i}(1,end) + ITR.w_tr{i}* sum(sum(abs(xdotc_prev-xdotc)));

        % Linear penalty cost
        L = info_.pcost;

        % Actual change in the penalty cost
        dN = J1 - J2; 

        % Predicted change by convex cost
        dL = J1 - L;

        if dL== 0
            ITR.w_tr{i+1} = ITR.w_tr{i};
        else
           % Ratio
           rho = dN/dL; 
           ITR.rho(i) = rho;
            
           ITR.w_tr{i+1} = Alpha*ITR.w_tr{i};
           % Update trust region radius 
%            if rho <= rho0
%               ITR.w_tr{i} = Alpha*ITR.w_tr{i}; 
%                 continue;
%            elseif rho > rho0 || rho < rho1
%               ITR.w_tr{i+1} = Alpha*ITR.w_tr{i};
%            elseif rho1 <= rho || rho < rho2
%               ITR.w_tr{i+1} = ITR.w_tr{i};
%            elseif rho2 <= rho
%               ITR.w_tr{i+1} = ITR.w_tr{i}/Beta;
%            end  
        end
     end
    i = i+1           

 end
acik_plots(i); 
end


