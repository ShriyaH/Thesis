A = [-0.5,-1.5,-0.7]; 
B = [1,0.5,0.3]; 
C = [1,1.5,2];
figure()
hold on
patch(A,B,C,'w','FaceAlpha',0.9) 
quiver3(0,0,0,2,0,0,'Color','r');
quiver3(0,0,0,0,2,0,'Color','g');
quiver3(0,0,0,0,0,2,'Color','b');
plot3([0,-0.5],[0,1],[0,1],'Color','k','linestyle','--');
plot3([0,-1.5],[0,0.5],[0,1.5],'Color','k','linestyle','--');
plot3([0,-0.7],[0,0.3],[0,2],'Color','k','linestyle','--');
plot3([-1,-0.5],[2,1],[3,1],'Color','k','linestyle','--');
plot3([-1,-1.5],[2,0.5],[3,1.5],'Color','k','linestyle','--');
plot3([-1,-0.7],[2,0.3],[3,2],'Color','k','linestyle','--');