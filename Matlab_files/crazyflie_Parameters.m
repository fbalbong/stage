%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Drone dynamic model parameters %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% clear all; close all; clc

%% geometry
grav   = 9.81; %m/s²
masse   = 29e-3; %kg 36.5g version CEA
a_side   = sqrt(3.1e-5*6/masse); %80e-3; % drone side dimension in [m]
height   = 0.5e-2; %15e-3 ; % 0.5e-2;  % drone efficient height in [m] (estimate for inertia
L= sqrt(a_side^2+a_side^2)/2; % 30e-3; %m 

% Inertia (estimate) as if it was a rectangle
% Area density  :
% normal plane to z-body :
rho_z = masse/a_side^2;
% normal plane to x-body or y-body :
rho_x_y = masse/(a_side*height);

% Moment of inertia (based on efficient rectangle)
Jxx = rho_x_y*a_side*height*(a_side^2+height^2)/12; % AK 2.4e-5 Modélisation CAO
Jyy = rho_x_y*a_side*height*(a_side^2+height^2)/12; % AK 2.4e-5 Modélisation CAO
Jzz = rho_z*a_side^4/6; % AK 3.2e-5 Modélisation CAO

J= [Jxx  0    0;
    0   Jyy  0;
    0   0    Jzz];
J_inv   = inv(J);

%% aerodynamical coefficients
haero   = 0 ; % we neglect aerodynamic effect on speed %10e-3;%22e-3; %m
% mu  = 1.2e-2; %kg/s
muz  = 5e-2; %kg/s
muxy  =0.4*masse; %kg/s

%% Motors

% fit 4th degree polynomial based on the bitcraze internet page
% https://wiki.bitcraze.io/misc:investigations:thrust
% PWM to RPM function (consider identical motors)
% coefs:
p4 = -0.00082934;
p3 = 0.19157;
p2 = -15.689;
p1 = 719.86;
p0 = 280.93;


Mc2u = [1  -0.5  0.5 -1;
         1  -0.5 -0.5  1;
         1   0.5 -0.5 -1;
         1   0.5  0.5  1]; %external inputs to motor input voltage Matrix

kconst=1.95e-8;%2.2e-8; %kgm/rad² thrust constant
b=1e-9; % 2e-9; %kgm²/rad² torque constant 

%% Simulation parameters
% Low pass filtering cutoff frequancy on gyro data
cutoff_freq=80; %Hz

MAXINT16=2^16-1; % Maximum control signal value

ATTITUDE_RATE = 500; % 500 Hz
dt=1/ATTITUDE_RATE; % Sampling time in sec

