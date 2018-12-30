phi = {};
        for ii = 1:K-2
            phi{ii}{1} = ITR.Ad_k{i}{ii+1};
            for jj = 1:(K-2-ii)
                phi{ii}{jj+1} = ITR.Ad_k{i}{ii+jj+1}*phi{ii}{jj};
            end
        end

        for ii = 0:K
            jdx = ii*n + ns + (1:nc);    
            for jj = 1:(K-3-ii) 
            idx = (1+jj+ii)*ns + (1:ns);    
            A(idx,jdx) = -phi{ii+1}{jj}*ITR.Bd_k{i}{ii+1};
            end
        end

        Z = zeros(ns*K,ns*(K-1));
        Z(ns+1:end,1:end) = eye(ns*(K-1)); 
        for ii = 0:K-2
            jdx = ii*ns + (1:ns);    
            for jj = 1:(K-2-ii) 
            idx = (1+jj+ii)*ns + (1:ns);
            Z(idx,jdx)= phi{ii+1}{jj};
            end
            idx = ii*ns +(1:ns);
            z(idx,1) = ITR.zd_k{i}{ii+1};
        end
        
        b1 = Z*z;
        b = zeros(K*ns,1);
        b(1:ns,1) = x0; %initial state
        b = b + b1;