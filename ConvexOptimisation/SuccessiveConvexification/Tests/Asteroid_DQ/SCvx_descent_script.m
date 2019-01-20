%%----Ast_descent_script----%%
% Asteroid descent problem for ECOS with Successive Convexification
% Shriya Hazra, 31-Jul-2018 
clear all; 
clc; 
close all; 

global ITR CONSTANTS;
tic
Initialize_models();
toc
tic
[xc,uc,xdotc,cpu_time,status] = SCvx_transcription();
toc
m_spent = xc(1,1)-xc(1,end);
disp(['mass spent: ',num2str(m_spent),' kg'])

