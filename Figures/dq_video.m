%DQ video
global CONSTANTS
vert ={};
vert{1} = [0 0 0;1 0 0;1 1 0;0 1 0;0 0 1;1 0 1;1 1 1;0 1 1];
fac = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];

CONSTANTS.dq_form = 2;
ri = [0.5,0.5,0.5];
qi = [0;0;0;1];
dqi = Q2DQ(qi,ri,CONSTANTS.dq_form);
rf = [5,6,8];
qf = [0.8;0.9;0.7;0.8];
qf = qf./norm(qf);
dqf = Q2DQ(qf,rf,CONSTANTS.dq_form);

[dqm,r,r_norm,ang,l,m,d,theta,t] = ScLERP(dqi, dqf, linspace(0,10,30));

for i = 2:length(dqm)
    vert{i} = vert{1}+r(1:3,i)';
end
figure()
hold on
quiver3(0.5,0.5,0.5,5,6,8,'Color','k','Linewidth',2);
for i = 1:length(dqm)
    quiver3(0.5,0.5,0.5,1.5,0,0,'Color','r','Linewidth',2);
    quiver3(0.5,0.5,0.5,0,1.5,0,'Color','g','Linewidth',2);
    quiver3(0.5,0.5,0.5,0,0,1.5,'Color','b','Linewidth',2);
    patch('Vertices',vert{i},'Faces',fac,...
      'FaceVertexCData',(1:6)','FaceColor','flat','FaceAlpha',0.3)
end
view(3)
axis vis3d
xlabel('X-axis')
ylabel('Y-axis')
zlabel('Z-axis')
axis equal