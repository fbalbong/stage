% Actuation gain

zeta = [Tc;ur;up;uy]; %actuation bloc input
u_M = Mc2u*zeta ; % Motor inputs
% Motor control to Omega NL function
cont2omega = @(u)2*pi/60*(p4*(100/MAXINT16)^4*u.^4+p3*(100/MAXINT16)^3*u.^3+p2*(100/MAXINT16)^2*u.^2+p1*(100/MAXINT16)*u+p0);
Omega0 = cont2omega(u_M); % equiulibrium Omega vector

% linear gain function
omega_lin_gain = @(u)2*pi/60*100/MAXINT16*(4*p4*(100/MAXINT16)^3*u.^3+3*p3*(100/MAXINT16)^2*u.^2+2*p2*(100/MAXINT16)*u+p1);
FFall = diag(omega_lin_gain(u_M));% Linear gain diagonal matrix

% equilibrium point at output
%Teq = k*ones(1,4)*Omega0.^2;
%tau_x_eq = k*L/sqrt(2)*[-1 -1 1 1]*Omega0.^2;
%tau_y_eq = -k*L/sqrt(2)*[-1 1 1 -1]*Omega0.^2; 
%tau_z_eq = b*[-1 1 -1 1]*Omega0.^2 ;

% Linear transformation matrix
LAM = [2*kconst*Omega0'*FFall*Mc2u;
       sqrt(2)*kconst*L*Omega0'*diag([-1 -1 1 1])*FFall*Mc2u;
       -sqrt(2)*kconst*L*Omega0'*diag([-1 1 1 -1])*FFall*Mc2u;
       2*b*Omega0'*diag([-1 1 -1 1])*FFall*Mc2u]; 

% xeq = [w_eq0;pdot_eq0;eta_eq0];
A13 = [     0     haero*muxy/Jxx     0;
      -haero*muxy/Jyy      0         0;
            0          0         0  ]; % Feedbakc bloc linear speed vx and vy to angular rate omega_x/y due to h, application point of aerodynamic force
A21 = [     1      sin(eta_eq0(1))*tan(eta_eq0(2))     cos(eta_eq0(1))*tan(eta_eq0(2));
            0              cos(eta_eq0(1))             -sin(eta_eq0(1));
            0      sin(eta_eq0(1))/cos(eta_eq0(2))     cos(eta_eq0(1))/cos(eta_eq0(2))   ]; % Bloc coupling angular speed to euler angels
A31 = [     0       -pdot_eq0(3)     pdot_eq0(2);
        pdot_eq0(3)       0         -pdot_eq0(1);
       -pdot_eq0(2)  pdot_eq0(1)          0       ]; % Coriolis block for linear speed dynamics
A32 = [                 0                       -grav*cos(eta_eq0(2))                 0;
       grav*cos(eta_eq0(1))*cos(eta_eq0(2))  -grav*sin(eta_eq0(1))*sin(eta_eq0(2))       0;
      -grav*sin(eta_eq0(1))*cos(eta_eq0(2))  -grav*cos(eta_eq0(1))*sin(eta_eq0(2))       0  ];
A33 = blkdiag(-muxy/masse,-muxy/masse,-muz/masse); % 

A_LIN = [zeros(3,3), zeros(3,3),    A13;
            A21,     zeros(3,3), zeros(3,3);
            A31,        A32,        A33];
B_LIN = [zeros(3,1), blkdiag(1/Jxx,1/Jyy,1/Jzz);
                    zeros(5,4);
            -1/masse, zeros(1,3)];
C_LIN = eye(9);
D_LIN = zeros(9,4);

%G_LIN = ss(A_LIN,B_LIN,[C_LIN],[D_LIN])*LAM;
%zpk(G_LIN(1,2))

G_FULL_LIN = ss(A_LIN,B_LIN*LAM,[C_LIN; zeros(4,9)],[D_LIN*LAM;LAM]);
G_FULL_LIN.OutputName={'dwr';'dwp';'dwy';'dphi';'dtheta';'dpsi';'dvx';'dvy';'dvz';'dT';'dtaux';'dtauy';'dtauz'};
G_FULL_LIN.InputName={'d_Trust';'d_roll';'d_pitch';'d_yaw'};
G_FULL_LIN.StateName={'dwr';'dwp';'dwy';'dphi';'dtheta';'dpsi';'dvx';'dvy';'dvz'};

% zpk(G_FULL_LIN('dwx','d_roll'))
% zpk(G_FULL_LIN('dwy','d_pitch'))
% zpk(G_FULL_LIN('dwz','d_yaw'))