function [F_tilde,normalsf,R_1,R_2,R_3,A_facet,C] = F_tild(Polyhedron)
%% Function to calculate diadic matrix for faces
%  It does not depend on a field point, so it can be
%  calculated once during initialization.
%
%%
F=Polyhedron.Facets;
V=Polyhedron.Vertices;


for i=1:1:(size(F,1))
      
    R1=V(F(i,1),:);    
    R2=V(F(i,2),:);    
    R3=V(F(i,3),:); 

    R12=R2-R1;

    R23=R3-R2;
    
    R31=R1-R3;

    nf = cross(R12,R23);
   
    nf = nf/norm(nf);
%     
%     cth1 = dot(nf,R1)/norm(R1);
%     cth2 = dot(nf,R2)/norm(R2);
%     cth3 = dot(nf,R3)/norm(R3);
    
    a = norm(R12);
    b = norm(R23);
    c = norm(R31);

    p = (a+b+c)/2;
    
    area = sqrt(p*(p-a)*(p-b)*(p-c));
%     if dot(nf, R1)<0 && dot(nf, R2)<0 && dot(nf, R3)<0
%         nf=-nf;
%     end
    
    c = (R1+R2+R3)/3;
    
    F_tilde(:,:,i)=nf'*nf;
    normalsf(i,:)= nf;   
    R_1(i,:) = R1;
    R_2(i,:) = R2;
    R_3(i,:) = R3;
    C(i,:) = c;
    A_facet(i,:) = area;
%     ctheta(i,:) = [cth1 cth2 cth3];
end

