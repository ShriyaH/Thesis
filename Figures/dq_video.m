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

for i = 1:length(dqm)
    for j = 1:8
    vert{i}(j,:) = quat_trans(dqm(1:4,i),vert{1}(j,:),'vect');
    end
    vert{i} = vert{i}+r(1:3,i)'-r(1:3,1)';
end

for i = 1:length(dqm)
    figure()
     hold on
    a(i,1:3) =  quat_trans(dqm(1:4,i),[1.5,0,0],'vect');
    b(i,1:3) =  quat_trans(dqm(1:4,i),[0,1.5,0],'vect');
    c(i,1:3) =  quat_trans(dqm(1:4,i),[0,0,1.5],'vect');
    quiver3(r(1,i),r(2,i),r(3,i),a(i,1),a(i,2),a(i,3),'Color','r','Linewidth',2);
    quiver3(r(1,i),r(2,i),r(3,i),b(i,1),b(i,2),b(i,3),'Color','g','Linewidth',2);
    quiver3(r(1,i),r(2,i),r(3,i),c(i,1),c(i,2),c(i,3),'Color','b','Linewidth',2);
    patch('Vertices',vert{i},'Faces',fac,...
      'FaceVertexCData',(1:6)','FaceColor','flat','FaceAlpha',0.5) 
  quiver3(0.5,0.5,0.5,5,6,8,'Color','k','Linewidth',2);
  view(3)
    axis vis3d
    xlabel('X-axis')
    ylabel('Y-axis')
    zlabel('Z-axis')
    axis equal
    grid on
end



%%% video

% workingDir = tempname;
% mkdir(workingDir)
% mkdir(workingDir,'images')
% DQVideo = VideoReader('dq.avi');
% ii = 1;
% 
% while hasFrame(dqVideo)
%    img = readFrame(dqVideo);
%    filename = [sprintf('%03d',ii) '.jpg'];
%    fullname = fullfile(workingDir,'images',filename);
%    imwrite(img,fullname)    % Write out to a JPEG file (img1.jpg, img2.jpg, etc.)
%    ii = ii+1;
% end