function [normalsf,R_1,R_2,R_3,C,A_facet] = SC_prop(Polyhedron)
%% Function to calculate area and centroid of faces

F=Polyhedron.Facets;
V=Polyhedron.Vertices;

for i=1:1:(size(F,1))
      
    R1=V(F(i,1),:);    
    R2=V(F(i,2),:);    
    R3=V(F(i,3),:); 

    R12=R2-R1;

    R23=R3-R2;
    
    R31=R1-R3;

    nf=cross(R12,R23);
   
    nf=nf/norm(nf);
    
    normalsf(i,:)=nf;  
    
    a = norm(R12);
    b = norm(R23);
    c = norm(R31);

    p = (a+b+c)/2;
    
    area = sqrt(p*(p-a)*(p-b)*(p-c));
    
    A_facet(i,:) = area;
    
    c = (R1+R2+R3)./3;
    
    R_1(i,:) = R1;
    R_2(i,:) = R2;
    R_3(i,:) = R3;
    C(i,:) = c;
end

