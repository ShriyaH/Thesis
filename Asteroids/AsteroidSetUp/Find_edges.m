function [ Edges,E_tilde,normalsa,normalsb,R1,R2 ] = Find_edges(Polyhedron)

%% Function that finds edges and calculates the diadic edge matrix
%INPUTS:
%
%Polyhedron - structure containing vertices and facets
%
%OUTPUTS:
%
%Edges - array which defines all edges
%        first and second column - number of vertices
%        third and fourth column - which facets share the same edge
%
%E_tilde - diadic matrix for an edge
%
%
error=0;
V=Polyhedron.Vertices;
F=Polyhedron.Facets;

j=1;
z=1;

s=size(F,1);

for i=1:1:s
    
   a=F(i,1);
   b=F(i,2);
   
   E(z,:)=[a b i];
   z=z+1;
   
   a=F(i,1);
   b=F(i,3);
   
   E(z,:)=[a b i];
   z=z+1;
   
   a=F(i,2);
   b=F(i,3);
   
   E(z,:)=[a b i];
   z=z+1;
end

E2=[sort(E(:,1:2),2) E(:,3)];
E3=sortrows(E2);

for i=1:1:size(E3,1)/2
    
    if E3(2*i-1,1)==E3(2*i,1) && E3(2*i-1,2)==E3(2*i,2)
        
        Edges(i,:)=[E3(2*i,1:2) E3(2*i-1,3) E3(2*i,3)];
            
    else
    
    error=1;
    
    end
    
end

for i=1:1:size(Edges,1)
    % Plane a
    f1=Edges(i,3);
    
    v1=F(f1,1);
    v2=F(f1,2);
    v3=F(f1,3);
    
    R1_a=V(v1,:);    
    R2_a=V(v2,:);    
    R3_a=V(v3,:);
    
    R12_a=R2_a-R1_a;

    R23_a=R3_a-R2_a;

   
    n_a=cross(R12_a,R23_a);
    
    n_a=n_a/norm(n_a);
    
    if dot(n_a, R1_a)<0
        n_a=-n_a;
    end
    
    if Edges(i,1)==v1 || Edges(i,2)==v1
        
      if Edges(i,1)==v2 || Edges(i,2)==v2
          r_1_a=v1;
          r_2_a=v2;
      end
      
      if Edges(i,1)==v3 || Edges(i,2)==v3
          r_1_a=v3;
          r_2_a=v1;
      end
    end
    
    if Edges(i,1)==v2 || Edges(i,2)==v2
        
        if Edges(i,1)==v3 || Edges(i,2)==v3
            r_1_a=v2;
            r_2_a=v3;
        end
    end
          
    
    % Plane b
    
    f1=Edges(i,4);
    
    v1=F(f1,1);
    v2=F(f1,2);
    v3=F(f1,3);
    
    R1_b=V(v1,:);    
    R2_b=V(v2,:);    
    R3_b=V(v3,:);
    
    R12_b=R2_b-R1_b;

    R23_b=R3_b-R2_b;

   
    n_b=cross(R12_b,R23_b);
    
    n_b=n_b/norm(n_b);
    
    if dot(n_b, R1_b)<0
        n_b=-n_b;
    end
    
    if Edges(i,1)==v1 || Edges(i,2)==v1
        
      if Edges(i,1)==v2 || Edges(i,2)==v2
          r_1_b=v1;
          r_2_b=v2;
      end
      
      if Edges(i,1)==v3 || Edges(i,2)==v3
          r_1_b=v3;
          r_2_b=v1;
      end
    end
    
    if Edges(i,1)==v2 || Edges(i,2)==v2
        
        if Edges(i,1)==v3 || Edges(i,2)==v3
            r_1_b=v2;
            r_2_b=v3;
        end
    end
    
    
    n12_a=cross(V(r_2_a,:)-V(r_1_a,:),n_a);
    n12_a=n12_a/norm(n12_a);
    n12_b=cross(V(r_2_b,:)-V(r_1_b,:),n_b);
    n12_b=n12_b/norm(n12_b);    

    E_tilde(:,:,i)=n_a'*n12_a + n_b'*n12_b;
    normalsa(i,:)= n_a;
    normalsb(i,:)= n_b;
end

E=Edges;
for j=1:1:size(E,1)
    
    R1(j,:)=V(E(j,1),:);    
    R2(j,:)=V(E(j,2),:); 
    
end
if error==1
msg = 'Error occurred.';
error(msg);
end
end

