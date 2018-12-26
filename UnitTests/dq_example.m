%% Dual Quaternion Example

q_AI = [0;0;0;1];
DCM_AI = Q2DCM(q_AI);

R = [0;0;0];
dq_AI = [q_AI; 0; 0; 0; 0];

q_BA = [0.38268; 0.38268; 0.38268; 0.9239];
%q_BA = [1; 0; 0; 0];
q_BA = q_BA./norm(q_BA);
DCM_BA = Q2DCM(q_BA);

t_A = [10; -15; 20; 0];

qd = 0.5.*cross_quat(t_A,q_BA);

dq_BA = [q_BA; qd];

q_BI = cross_quat(q_BA,q_AI);
q_BI = q_BI./norm(q_BI);

%% SC Design 

SC_block = [2.1 2 2.8]; %SC dimensions, m
SC_SA = [0 14 2.28]; %Solar array dimensions, m

x = SC_block(1)/2;
y = SC_block(2)/2;
z = SC_block(3)/2;

sy = SC_SA(2)/5;
sz = SC_SA(3)/2;
d = 0.06;
d1 = 1+y;

block = [x -y -z
      x y -z
     -x y -z
     -x -y -z
     x -y z
      x y z
      -x y z
     -x -y z];
 
SA1 = [0 d1 -sz 
      0 d1 sz 
      0 d1+sy -sz  
      0 d1+sy sz 
      0 d1+sy+d -sz  
      0 d1+sy+d sz  
      0 d1+2*sy+d -sz 
      0 d1+2*sy+d sz 
      0 d1+2*sy+2*d -sz 
      0 d1+2*sy+2*d sz 
      0 d1+3*sy+2*d -sz 
      0 d1+3*sy+2*d sz 
      0 d1+3*sy+3*d -sz 
      0 d1+3*sy+3*d sz 
      0 d1+4*sy+3*d -sz 
      0 d1+4*sy+3*d sz 
      0 d1+4*sy+4*d -sz 
      0 d1+4*sy+4*d sz 
      0 d1+5*sy+4*d -sz 
      0 d1+5*sy+4*d sz]; 
SA2 = -SA1;
Polyhedron.Vertices = [block;SA1;SA2];
       
Polyhedron.Facets = [1 2 6
    1 6 5
    2 3 7
    2 7 6
    3 4 8
    3 8 7
    4 1 5
    4 5 8
    4 3 2
    4 2 1
    5 6 7
    5 7 8
    9 11 12
    9 12 10
    13 15 16
    13 16 14
    17 19 20
    17 20 18
    21 23 24
    21 24 22
    25 27 28
    25 28 26
    29 32 30
    29 31 32
    33 36 34
    33 35 36
    37 40 38
    37 39 40
    41 44 42
    41 43 44
    45 48 46
    45 47 48];

% [Polyhedron.Facets, Polyhedron.Vertices] = stlread('Rosetta.stl');
% [nf,nv] = reducepatch(SC.Polyhedron.Facets,SC.Polyhedron.Vertices,0.02);
[Polyhedron.normalsf,Polyhedron.R_1,Polyhedron.R_2,Polyhedron.R_3,Polyhedron.C,Polyhedron.A_facet] = SC_prop(Polyhedron);

%SC.Polyhedron = Polyhedron;

patch('Vertices',Polyhedron.Vertices,'Faces',Polyhedron.Facets,'FaceVertexCData',[1 1 1],'FaceColor','flat');
hold on
Ref_frame_plot(DCM_AI, R)
axis equal

%% DQ Transformation

for i = 1:3 
    dq_BI(:,i) = dq_trans(dq_BA,DCM_AI(i,:));
    r_BI(i,:) = dq_BI(5:7,i)';
    DCM_BI(i,:) = r_BI(i,:)-t_A(1:3)';
end


for j = 1:length(Polyhedron.Vertices)
    Polyhedron.Vert_new(j,:) = (DCM_BA * Polyhedron.Vertices(j,:)')' + t_A(1:3)';
end

Ref_frame_plot(DCM_BI, t_A(1:3))
hold on
patch('Vertices',Polyhedron.Vert_new,'Faces',Polyhedron.Facets,'FaceVertexCData',[1 1 1],'FaceColor','flat');
grid on

%% Screw interpolation (ScLERP)

theta_rad = 2*acos(q_BA(4));
l = q_BA(1:3);
d = dot(t_A(1:3),l);
m = 0.5*(cross(t_A(1:3),l)+ cross(l,cot(theta_rad/2)*cross(t_A(1:3),l)));
p = cross(l,m);

t = acos(dot(p,[1; 1; 0])./norm(p));
scale = 50;
quiver3(p(1),p(2),p(3), l(1), l(2), l(3), scale)
hold on
quiver3(0,0,0,p(1),p(2),p(3))
plotCircle3D(p,l,norm(p))

xlabel('X');
ylabel('Y');
zlabel('Z');