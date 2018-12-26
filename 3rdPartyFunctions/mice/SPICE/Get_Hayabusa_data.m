function [ R,V,TIME ] = Get_Hayabusa_data( )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
cspice_furnsh( 'SPICE/standard.txt' )


epoch1    = 'October 07, 2005 11:00 PM PST';
epoch2    = 'October 10, 2005 11:00 PM PST';
et1 = cspice_str2et( epoch1 );
et2 = cspice_str2et( epoch2 );

ett=et1:1:et2;
    
[a,b,c,d]=cspice_frinfo ( 2025143); 
[state, lt] = cspice_spkezr('HAYABUSA', ett, 'ITOKAWA_FIXED', 'NONE', 'ITOKAWA');


cspice_unload( 'standard.tm' )

R=1000*state(1:3,:);
V=1000*state(4:6,:);
TIME=ett-et1;
end

