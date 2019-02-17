%%----Ast4_descent_script----%%
% Asteroid descent problem for ECOS with Successive Convexification
% Shriya Hazra, 31-Jul-2018 
clear all; 
clc; 
close all; 

global ITR PARAMS CONSTANTS;
ini_models_acik;

[xc1,uc1,xdotc1,cpu_time1,status1] = SCvx_transcription_acik();

m_spent = xc1(1,1)-xc1(1,end);
disp(['mass spent: ',num2str(m_spent),' kg'])

total_cpu_time1 = sum(cpu_time1);

save('acik_CQ')
