clear all
close all
% state=zeros(100);
% LT=zeros(1);

cspice_furnsh( 'SPICE/standard_R.txt' )


epoch1    = 'November 27, 2014 11:00 PM PST';
epoch2    = 'September 12, 2014 11:00 PM PST';
et1 = cspice_str2et( epoch1 );
et2 = cspice_str2et( epoch2 );

ett=et1:1:et2;
    
 [a,b,c,d]=cspice_frinfo (-1000012000); 
[state, lt] = cspice_spkezr('-1000012000', et1, 'J2000', 'NONE', 'EARTH');


radii = cspice_bodvrd( '-1000012000', 'RADII', 3 )
% C = cspice_pxform('ITOKAWA_FIXED' , 'J2000', ett );
% 
cspice_unload( 'standard_R.txt' )
% 
R_I=state(1:3,:);
V_I=state(4:6,:);
% % for i=1:1:size(R_A,2)
% % R_I(:,i)=C(:,:,i)*R_A(:,i);
% % end
% Time=ett-et1;
 %% Plotting
fsz=11;
ppsz=[8 8]; 
mrg=0.1;
% 
figure()

plot3(R_I(1,:),R_I(2,:),R_I(3,:));
% 
% figure
% 
% plot(Time, V_I(1,:));
% hold on
% plot(Time, V_I(2,:));
% hold on
% plot(Time, V_I(3,:));
% 
% % N=(et2-et1)/5;
% % 
% % for a=1:N:size(state,2)-1
% % 
% % plot3(state(1,a:a+N),state(2,a:a+N),state(3,a:a+N),'Linewidth',2);
% % hold on
% % 
% % end
% % legend t1 t2 t3 t4 t5
% % plot(R(:,1),R(:,3),'Linewidth',2)
% % hold on
% % plot(R(:,1),R(:,4),'Linewidth',2)
% 
% % grid on
% % % title('Position vector')
% % % legend('r_x','r_y','r_z','Location','NorthEast')
xlabel('X axis')
ylabel('Y axis')
zlabel('Z axis')
% % 
% % 
% % [v,f]=Find_index('itokawa.txt');
% % 
% % Data=importdata('itokawa.txt');
% % V=Data.data(1:v,:);
% % F=Data.data(v+1:f,:);
% % 
% % plot3(V(:,1),V(:,2),V(:,3),'.','Color','k');
% % axis equal
% % for i=1:size(F,1)
% % 
% % line([V(F(i,1),1) V(F(i,2),1)],[V(F(i,1),2) V(F(i,2),2)],[V(F(i,1),3) V(F(i,2),3)], 'Color', 'k')
% % hold on
% % line([V(F(i,2),1) V(F(i,3),1)],[V(F(i,2),2) V(F(i,3),2)],[V(F(i,2),3) V(F(i,3),3)], 'Color', 'k')
% % hold on
% % line([V(F(i,1),1) V(F(i,3),1)],[V(F(i,1),2) V(F(i,3),2)],[V(F(i,1),3) V(F(i,3),3)], 'Color', 'k')
% % hold on
% % 
% % end
% 
% 
goodplot(ppsz,mrg,fsz)
print -dpdf ROSA.pdf