function[]=wxlabel(string,fontsize)
% This function simply puts the settings for a proper visualization of the
% labels, titles and so on;
%
% Marco Sagliano, DLR, 10-Apr-2013

if nargin==1
    xlabel(string,'FontSize',12,'FontName','Arial');
else    
    xlabel(string,'FontSize',fontsize,'FontName','Arial');
end