% Define the number of divisions of the time interval.
STEP = 1000;
% Construct a meta kernel, "standard.tm”, which will be used to load the needed
% generic kernels: "naif0009.tls," "de421.bsp,” and "pck00009.tpc.”
% Load the generic kernels using the meta kernel, and a Cassini spk.
cspice_furnsh( { 'standard.tm', 'https://naif.jpl.nasa.gov/pub/naif/pds/data/co-s_j_e_v-spice-6-v1.0/cosp_1000/data/spk/'} )
et = cspice_str2et( {'Jun 30, 2005', 'Sep 26, 2007'} );
times = (0:STEP-1) * ( et(2) - et(1) )/STEP + et(1);
ptarg = mice_spkpos( 'Cassini', times, 'J2000', 'NONE', 'SATURN BARYCENTER' );
pos = [ptarg.pos];
% Plot the resulting trajectory.
x = pos(1,:);
y = pos(2,:);
z = pos(3,:);
plot3(x,y,z)
cspice_kclear