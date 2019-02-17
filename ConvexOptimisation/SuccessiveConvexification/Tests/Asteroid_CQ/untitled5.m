ITR.Ad_k = {}
for i = 1:2
    for ii = 1:30
        ITR.Ad_k{i}{ii} = randn(20,20);
        ITR.Bd_k{i}{ii} = randn(20,3);
        ITR.zd_k{i}{ii} = randn(20,1);
    end
end
K =30;
phi = {};
  for k = 1:K-1
       for ii = 1:K-k
           if ii == 1 
               phi{k}{ii} = ITR.Ad_k{1}{k}; 
           else
               phi{k}{ii} = ITR.Ad_k{1}{k}*phi{k}{ii-1};
           end
       end
   end
   ns =20;
   nc = 3;
   n = 20+3+20+1+1;
   A = zeros(K*ns+ns-1,K*n+2);
           
for ii = 1:K-2
   jdx = (ii-1)*n + ns + (1:nc);    
   for jj = 1:(K-1-ii) 
   idx = (jj+ii)*ns + (1:ns);    
   A(idx,jdx) = -phi{ii+1}{jj}*ITR.Bd_k{i}{ii};   
   end
end

Z = zeros(ns*K,ns*(K-1));
for ii = 1:K-1
   idx = (ii)*ns + (1:ns);
   jdx = (ii-1)*ns + (1:ns);
   Z(idx,jdx) = eye(ns); 
   
   idx = (ii-1)*ns +(1:ns);
   z(idx,1) = ITR.zd_k{i}{ii};
end

for ii = 1:K-2
   jdx = (ii-1)*ns + (1:ns);    
   for jj = 1:(K-1-ii) 
   idx = (jj+ii)*ns + (1:ns);    
   Z(idx,jdx) = phi{ii+1}{jj};   
   end
end

z_new = Z*z;
z_new = z_new(ns+1:end,1);

b(ns+1:K*ns,1) = z_new;    