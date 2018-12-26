function [ output_args ] = goodplot(aspectratio,papersize, margin, fontsize)
% function which produces a nice-looking plot
% and sets up the page for nice printing
% papersize in inches
if nargin == 0
    aspectratio = [1 0.357 0.357];
    papersize(1) = 8;
    papersize(2) = 8;
    margin = 0.5;
    fontsize = 18;

elseif nargin == 1
    papersize(1) = 8;
    papersize(2) = 8;    
    margin = 0.5;
    fontsize = 18;

elseif nargin == 2
    margin = 0.5;
    fontsize = 18;

elseif nargin == 3
    fontsize = 18;  
end

set(get(gca,'xlabel'),'FontSize', fontsize, 'FontWeight', 'normal');
set(get(gca,'ylabel'),'FontSize', fontsize, 'FontWeight', 'normal');
set(get(gca,'title'),'FontSize', fontsize, 'FontWeight', 'normal');
grid on
set(gca,'LineWidth',1);
set(gca,'FontSize',fontsize);
set(gca,'FontWeight','normal');
set(gca,'PlotBoxAspectRatio',aspectratio);
set(gcf,'color','w');
set(gcf,'PaperUnits','inches');
set(gcf,'PaperSize', [papersize(1) papersize(2)]);
set(gcf,'PaperPosition',[margin margin papersize(1)-2*margin papersize(2)-2*margin]);
set(gcf,'PaperPositionMode','Manual');



end

