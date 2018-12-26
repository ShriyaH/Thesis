function [ Wf ] = Ast_in_out( R_A, Object)

%% Function that shows if a point is inside an asteroid (polyhedron) or outside

% Inputs:
% R_A - position in Asteroid fixed frame
% Polyhedron - structure containing polyhedron vertices, facets
% 
% Outputs:
% Wf - solid angle(0 if the point is outside, 4pi if it is inside)
% NOTE: the output is never exactly zeros due to numerical errors
% 
% Please read the paper of D. Scheeres: Exterior gravitation of a polyhedron
% derived and compared with harmonic and mascon gravitation representations 
% of asteroid 4769 Castalia

V=Object.Vertices;
F=Object.Facets;

Wf=0;
%% 
 for i=1:1:(size(F,1))
      
    R1=V(F(i,1),:);    
    R2=V(F(i,2),:);    
    R3=V(F(i,3),:);
    

    Ri=R1'-R_A;
    ri=norm(Ri);
    
    Rj=R2'-R_A;
    rj=norm(Rj);
    
    Rk=R3'-R_A;
    rk=norm(Rk);
    
    
    wf=2*atan2(dot(Ri, cross(Rj,Rk)),(ri*rj*rk+ri*dot(Rj,Rk)+rj*dot(Rk,Ri)+rk*dot(Ri,Rj)));
    
    Rf=(Ri+Rj+Rk)/3;
    
    Wf=wf+Wf;
  
 end

    


end

