%% Plots
ast = 0;
rot = 0; %Plot Rotating Asteroid, Impact ellipse and Escape sphere
map = 0;
orient = 0;
suncone = 0;
sc = 1;

figure()
hold on 
axis equal
grid on
% set(gca, 'Box','off', 'Projection','perspective','color','black', 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w')
% set(gcf,'color','black');

%% Plot SC

if sc == 1
    o = [0 0 0];
    x = [10 0 0; 0 0 0; 0 0 0];
    y = [0 0 0; 0 20 0; 0 0 0];
    z = [0 0 0; 0 0 0; 0 0 10];
    patch('Vertices',SC.Polyhedron.Vertices,'Faces',SC.Polyhedron.Facets,'FaceVertexCData',[0 1 1],'FaceColor','flat'); %Plot asteroid polyhedron
    plot_ref(o,x,'r')
    plot_ref(o,y,'g')
    plot_ref(o,z,'b')
    xlabel('X (m)')
    ylabel('Y (m)')
    zlabel('Z (m)')
end
%% Plot Asteroid

if ast == 1
    patch('Vertices',Kleopatra.Polyhedron.Vertices,'Faces',Kleopatra.Polyhedron.Facets,'FaceVertexCData',[0 1 1],'FaceColor','flat'); %Plot asteroid polyhedron
    patch('Vertices',nv,'Faces',nf,'FaceVertexCData',[0.9 0.9 0.9],'FaceAlpha',0.3); %Plot asteroid polyhedron
    % Plot Landmarks 
    for i=1:1:n_lm
    plotCircle3D(Kleopatra.lm_coord(i,:), Kleopatra.lm_n(i,:), Kleopatra.lm_r(i))  %Plot landmarks on asteroid
    end 
% a = [9.401598365967719e+04; 9.352002697396978e+04; 9.303163666666657e+04; 9.255146865820992e+04; 9.208009885615479e+04];
% b = [2.214156056211550e+04; 2.266056351234433e+04; 2.315769083437147e+04; 2.363379809302976e+04; 2.408970420449762e+04];
% c = [6.928203230275509e+04];
    % Plot trajectories
    hold on
    for i = 1:n_v
       plot3(Traj.y_A{i}(:,1),Traj.y_A{i}(:,2),Traj.y_A{i}(:,3)); %Plot all propagated sample trajectories
       %plot3(Traj.y_A{i}(end,1),Traj.y_A{i}(end,2),Traj.y_A{i}(end,3),'ro','MarkerSize',10); %Plot all propagated sample trajectories
%        [x y z] = ellipsoid(0,0,0,a(i),b(i),c,20);
%        XX = zeros(NN+1,NN+1);
%        YY = zeros(NN+1,NN+1);
%        ZZ = zeros(NN+1,NN+1);
%        for k = 1:length(X)
%             for j = 1:length(X)
%                 point = [X(k,j) Y(k,j) Z(k,j)]';
%                 P = M * point;
%                 XX(k,j) = P(1)+C(1);
%                 YY(k,j) = P(2)+C(2);
%                 ZZ(k,j) = P(3)+C(3);
%                 semi = [a;b; c];
%                 Q = M * semi;
%             end
%         end
    
    end
    
 %surf(Imp_Ellipse.XX,Imp_Ellipse.YY,Imp_Ellipse.ZZ,'edgecolor', 'w', 'FaceAlpha', 0.3);  %Plot impact ellipsoid
end

%% Plot Rotating Asteroid

if rot == 1 
    t = cell2mat(Traj.te);

    for n = 1:length(t)
        y = -rad2deg(Kleopatra.w_AI(3)*t);
        x.new(:,t) = cross_quat(Frames.quat.Q_AI(1:4,t),cross_quat(x.start,conj_quat(Frames.quat.Q_AI(1:4,t))));
        c(:,:,t) = [zeros(length(Kleopatra.Facets),1) Sun.gradient(:,t) Sun.gradient(:,t)];

       patch('Vertices',Kleopatra.Polyhedron.Vertices,'Faces',Kleopatra.Polyhedron.Facets,'FaceVertexCData',[0 1 1],'FaceColor','flat'); %Plot asteroid polyhedron
        patch('Vertices',Esc_Kleopatra.Vertices,'Faces',Kleopatra.Polyhedron.Facets,'FaceColor',[0 1 0],'FaceAlpha',0.1,'EdgeColor','none'); %Plot escape volume polyhedron

        alpha 0.3
        patch('Vertices',Imp_Kleopatra.Vertices,'Faces',Kleopatra.Polyhedron.Facets,'FaceColor',[1 1 1],'FaceAlpha',0.5,'EdgeColor','none');

        view(y,50);     %  Asteroid's axial tilt
        drawnow
    end
end

%% Plot Traj vs Vel vs Time
 
if map == 1
    for i = 1:n
        for j = 1:length(y{i}) 
        r(j,i) = norm([y{i}(j,1) y{i}(j,2) y{i}(j,3)]);
        v(j,i) = norm([y{i}(j,4) y{i}(j,5) y{i}(j,6)]);    

        end
       plot3(r(1:j,i),t{i},v(1:j,i)) 
    end
    xlabel('Radius Vector')
    ylabel('Time')
    zlabel('Velocity')
end

%% Plot SC axis orientation
% vv = 1000*SC.Polyhedron.Vertices + Traj.y{1}(2,1:3);

if orient == 1
    o = [0 0 0];
    i = [1e5 0 0; 0 1e5 0; 0 0 1e5];
    j = [Sun.pos(1,1)/5e6 Sun.pos(2,1)/5e6 Sun.pos(3,1)/5e6];
    y = [Traj.y{5}(2,1) Traj.y{5}(2,2) Traj.y{5}(2,3); Traj.y{5}(30,1) Traj.y{5}(30,2) Traj.y{5}(30,3); Traj.y{5}(100,1) Traj.y{5}(100,2) Traj.y{5}(100,3)];

    q1=Orient.q_BI{5}(2,:)';
    q2=Orient.q_BI{5}(30,:)';
    q3=Orient.q_BI{5}(100,:)';
    a=10000.*[quat_trans(q1,[1;0;0;0],'vect')' quat_trans(q1,[0;1;0;0],'vect')' quat_trans(q1,[0;0;1;0],'vect')'];
    b=10000.*[quat_trans(q2,[1;0;0;0],'vect')' quat_trans(q2,[0;1;0;0],'vect')' quat_trans(q2,[0;0;1;0],'vect')'];
    c=10000.*[quat_trans(q3,[1;0;0;0],'vect')' quat_trans(q3,[0;1;0;0],'vect')' quat_trans(q3,[0;0;1;0],'vect')'];
    
%     r = Orient.r_HB{5}(2,:)./5e6;
%     r1 = Orient.r_HB{5}(150,:)./5e6;

    xlabel('x')
    ylabel('y')
    zlabel('z')
    plot3(Traj.y{5}(2:end,1),Traj.y{5}(2:end,2),Traj.y{5}(2:end,3));   %Plot all propagated sample trajectories
    plot_ref(o,i,'g')
    plot_ref(o,y,'r')
    plot_ref(o,j,'k')

%     m = 10000.*[Orient.x{1}(2,1:3); Orient.y{1}(2,1:3); Orient.z{1}(2,1:3)];
%     n = 10000.*[Orient.x{1}(30,1:3); Orient.y{1}(30,1:3);Orient.z{1}(30,1:3) ];
%     o = 10000.*[Orient.x{1}(70,1:3); Orient.y{1}(70,1:3); Orient.z{1}(70,1:3)];
%     plot_ref(y(1,1:3),m,'r')
%     plot_ref(y(2,1:3),n,'r')
%     plot_ref(y(3,1:3),o,'r')

    plot_ref(y(1,1:3),a,'b')
%     plot_ref(y(1,1:3),r,'b')
%     plot_ref(y(3,1:3),r1,'b')
    plot_ref([0 0 0],b,'b')
%     plot_ref([0 0 0],c,'b')
%     plot3(Traj.y{1}(2,1),Traj.y{1}(2,2),Traj.y{1}(2,3),'bo','MarkerSize',30)



    for i=1:1:n_lm
        plotCircle3D(Kleopatra.lm_coord(i,:)/5, Kleopatra.lm_n(i,:), Kleopatra.lm_r(i)/5)  %Plot landmarks on asteroid
    end

    for i=[2,100]
        patch('Vertices',Orient.SC_V_new{5}{i},'Faces',SC.Polyhedron.Facets,'FaceVertexCData',[1 1 1],'FaceColor','flat');
        patch('Vertices',Orient.Ast_V_new{5}{i}/5,'Faces',Kleopatra.Polyhedron.Facets,'FaceVertexCData',[0 1 1],'FaceColor','flat')
    end
end

hold off    

%% Plot SC sun cone

if suncone == 1
th_forb = atan(sz/(d1+5*sx+4*d));
xx = (d1+5*sx+4*d)/(cos(th_forb));
ss = 3e11*tan(th_forb);

figure()
hold on
% plot3([0 d1+5*sx+4*d],[0 0],[0 sz])
patch('Vertices',Vertices,'Faces',Facets,'FaceVertexCData',[1 1 1],'FaceColor','flat');
axis equal


% 
% r = linspace(0,sz) ;
% th = linspace(0,2*pi) ;
% [R,T] = meshgrid(r,th) ;
% Z = R.*cos(T) ;
% Y = R.*sin(T) ;
% X = linspace(0,d1+5*sx+4*d) ;
% surf(X,Y,Z,'FaceAlpha', 0.3, 'FaceColor', 'w')
% surf(-X,Y,Z,'FaceAlpha', 0.3, 'FaceColor', 'w')
end