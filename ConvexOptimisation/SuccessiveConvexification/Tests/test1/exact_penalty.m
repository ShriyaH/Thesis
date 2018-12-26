%exact penalty constraints
if Switch.exact_penalty_on %||.||_1 in cost function
    if Switch.virtual_control_on % nu_k <= s_k
        Gt = zeros(K,K*n);
        ht = zeros(K,1);

        for ii = 1:K        
            Gt(ii,(ii-1)*n+ns+nc+(1:nv)) = 1;
            Gt(ii,(ii-1)*n+ns+nc+nv+(1:nsl)) = -1;
        end

        G = [G;Gt];
        h = [h;ht];
        dims.l =  size(G,1);

    end

    if Switch.virtual_control_on % -nu_k <= s_k
        Gt = zeros(K,K*n);
        ht = zeros(K,1);

        for ii = 1:K
            Gt(ii,(ii-1)*n+ns+nc+(1:nv)) = -1;
            Gt(ii,(ii-1)*n+ns+nc+nv+(1:nsl)) = -1;
        end

        G = [G;Gt];
        h = [h;ht];
        dims.l =  size(G,1);

    end
end 

if Switch.exact_penalty_on %||.||_1 in cost function
    if Switch.virtual_control_on % ||s_k||_2 <= kappa_k
    S = zeros(nv,K*n);
    s = zeros(1,K*n);
    bc = zeros(nv,1);
        for ii = 1:K
            Ac = S;
            Ac(1:nv,(ii-1)*n+ns+nc+(1:nsl)) = eye(nv);

            cc = s;
            cc(1,(ii-1)*n+ns+nc+nv+(1:nsl)) = 1;

            dc = 0;
            Gt = -[cc;Ac];
            ht = [dc;bc];

            G = [G;Gt];
            h = [h;ht];

            dims.q = [dims.q (nv+1)];

        end
    end
end


