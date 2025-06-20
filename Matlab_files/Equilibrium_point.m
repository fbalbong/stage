%% Computation of equilibrium point
% We first select the desired Euler angles of the drone then compute the corresponding
% linear speeds and finally define the corresponding inputs of Drone model

w_eq0 = [0;0;0]; % angular rates are always zero at equilibrium point

eta_eq0 = [0;0;0];  % [deg2rad(10);deg2rad(20);deg2rad(30)];% % the desired Euler angles at the output
pdot_eq0 = [-masse*grav/muxy*sin(eta_eq0(2));masse*grav/muxy*cos(eta_eq0(2))*sin(eta_eq0(1));0]; % linear speed, vz is free and is fixed to zero
% The dron is not mooving and thrust and torks compensate for gravity and other forces

% initialization of quadDynamics to equilibrium point
% q_init      = [1;0;0;0];
pdot_init   = pdot_eq0;
w_init      = w_eq0;
eta_init    = eta_eq0;

% inptus of dynamical model bloc quadDynamics
Teq0 = masse*grav*cos(eta_eq0(1))*cos(eta_eq0(2))-muz*pdot_eq0(3);
tau_x_eq0 = -masse*grav*haero*cos(eta_eq0(2))*sin(eta_eq0(1));
tau_y_eq0 = -masse*grav*haero*sin(eta_eq0(2));
tau_z_eq0 = 0;

% inputs for actuation bloc
Omagas = sqrt([1  1  1  1;
         -1 -1  1  1;
         -1  1  1 -1;
         -1  1 -1  1]^-1*[Teq0/kconst;sqrt(2)*tau_x_eq0/kconst/L;-sqrt(2)*tau_y_eq0/kconst/L;tau_z_eq0/b]);

uis = roots(2*pi/60*[p4*(100/2^16)^4 p3*(100/2^16)^3 p2*(100/2^16)^2 p1*(100/2^16) p0-60/2/pi*Omagas(1)]);
uis = uis(imag(uis)==0);
u1 = uis(uis>=0);

uis = roots(2*pi/60*[p4*(100/2^16)^4 p3*(100/2^16)^3 p2*(100/2^16)^2 p1*(100/2^16) p0-60/2/pi*Omagas(2)]);
uis = uis(imag(uis)==0);
u2 = uis(uis>=0);

uis = roots(2*pi/60*[p4*(100/2^16)^4 p3*(100/2^16)^3 p2*(100/2^16)^2 p1*(100/2^16) p0-60/2/pi*Omagas(3)]);
uis = uis(imag(uis)==0);
u3 = uis(uis>=0);

uis = roots(2*pi/60*[p4*(100/2^16)^4 p3*(100/2^16)^3 p2*(100/2^16)^2 p1*(100/2^16) p0-60/2/pi*Omagas(4)]);
uis = uis(imag(uis)==0);
u4 = uis(uis>=0);

inpts = [1  -0.5  0.5 -1;
         1  -0.5 -0.5  1;
         1   0.5 -0.5 -1;
         1   0.5  0.5  1]^-1*[u1(2);u2(2);u3(2);u4(2)]; % Inputs to actuation bloc

Tc = inpts(1); % 41415; % 44100; % Trust setpoint to compansate for drone weight
ur = inpts(2); %0; % roll
up = inpts(3); %0; % pitch
uy = inpts(4); %0; % yaw

% Drone Simulink Model operator setpoint definition
thurstSP = Tc;

rollSP=4;
rollSPtime = 2.5;
pitchSP=2;
pitchSPtime = 3;
yawSP=-3;
yawSPtime = 2.1;
    
Tend = 20; % Simulation time



% % non Zero equilibrium point, input thrust T = 0.5, 10 degree for each
% % euler angle
% w_eq = [0;0;0];
% eta_eq = [deg2rad(5);deg2rad(0);deg2rad(0)];
% pdot_eq = [-m*g/muxy*sin(eta_eq(2));m*g/muxy*cos(eta_eq(2))*sin(eta_eq(1));m/muz*(-0.5/m+g*cos(eta_eq(1))*cos(eta_eq(2)))];
% % inptus
% Teq = m*g*cos(eta_eq(1))*cos(eta_eq(2))-muz*pdot_eq(3);
% deltaT = 1;
% tau_x_eq = -m*g*h*cos(eta_eq(2))*sin(eta_eq(1));
% tau_y_eq = -m*g*h*sin(eta_eq(2));
% tau_z_eq = 0;
% delta_tau = 1e-4;

