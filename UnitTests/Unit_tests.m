eb = [-0.216846275957216	0.972739379179911	0.0821936299001812];
eb = eb./norm(eb);
p = atan2(eb(2),eb(1));
t = atan2(eb(3),norm(eb));
% C = Eul2DCM([-p,t+pi/2,-p],'ZYZ');
% q = DCM2Q(C);
q1 = Eul2Q([-p,t+pi/2,-p],'ZYZ');
x1 = quat_trans(q1,[1,0,0],'vect');
x1 = x1./norm(x1);
q2 = [x1';0];
q =cross_quat(q2,q1);
zz = quat_trans(q,[0,0,1],'vect');
zz1 = quat_trans(q,[0,0,-1],'vect');
yy = quat_trans(q,[0,1,0],'vect');
xx = quat_trans(q,[1,0,0],'vect');

figure()
quiver3(0,0,0,eb(1),eb(2),eb(3),'Color','y');
hold on
quiver3(0,0,0,zz(1),zz(2),zz(3),'Color','b');
quiver3(0,0,0,zz1(1),zz1(2),zz1(3),'Color','k');
quiver3(0,0,0,yy(1),yy(2),yy(3),'Color','g');
quiver3(0,0,0,xx(1),xx(2),xx(3),'Color','r');
quiver3(0,0,0,0,0,1,'Color','b');
quiver3(0,0,0,0,0,-1,'Color','k');
quiver3(0,0,0,0,1,0,'Color','g');
quiver3(0,0,0,1,0,0,'Color','r');
grid on

