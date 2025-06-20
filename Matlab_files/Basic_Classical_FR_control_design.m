% eig(A_LIN)

%% Rate Control

% three independent plant tranfer functions for each angle
Gwr = minreal(180/pi*(G_FULL_LIN('dwr','d_roll'))); zpk(Gwr)
Gwp = minreal(180/pi*(G_FULL_LIN('dwp','d_pitch'))); zpk(Gwp)
Gwy = minreal(180/pi*(G_FULL_LIN('dwy','d_yaw'))); zpk(Gwy)

% Low pass filter applied to measure
[FN, FD] = butter(2,2*pi*cutoff_freq,"low",'s');
F = tf(FN,FD); 
FN = 252700% 252661.8726678876;
FD = [1, 710.9, FN];
F2 = tf(FN,FD);

% Three independent rate controller computation (inner loop) as a simple proportionnal
% gain
Kwr = round(100/(180/pi/Jxx*LAM(2,2)),-1);
Kwp = Kwr;
% sisotool(Gwr,Kwr,F)
Kwy = round(100/(180/pi/Jzz*LAM(4,4)),-1);
% sisotool(Gwy,Kwy,F)

PID_ROLL_RATE_KP = Kwr;
PID_ROLL_RATE_KI = 0;
PID_ROLL_RATE_KD = 0;

PID_PITCH_RATE_KP = Kwp;
PID_PITCH_RATE_KI = 0;
PID_PITCH_RATE_KD = 0;

PID_YAW_RATE_KP = Kwy;
PID_YAW_RATE_KI = 0;
PID_YAW_RATE_KD = 0;

% Verification of modulus margin for inner loop
Lwr = Gwr*Kwr*F;
Swr = feedback(1,Lwr),
MMr = 1/norm(Swr,'inf')
Lwp = Gwp*Kwp*F;
Swp = feedback(1,Lwp);
MMp = 1/norm(Swp,'inf')
Lwy = Gwy*Kwy*F;
Swy = feedback(1,Lwy);
MMy = 1/norm(Swy,'inf')

% plant model seen be angle controller
Getar = tf(180/pi,[1 0])*zpk(feedback(Gwr*pi/180*Kwr, 180/pi*F))
Getap = tf(180/pi,[1 0])*zpk(feedback(Gwp*pi/180*Kwp, 180/pi*F))
Getay = tf(180/pi,[1 0])*zpk(feedback(Gwy*pi/180*Kwy, 180/pi*F))

% Tunning of angle controllers
PID_ROLL_KP = 100/5
PID_ROLL_KI = 100;
PID_ROLL_KD = 0;

PID_PITCH_KP = 100/5
PID_PITCH_KI = 100;
PID_PITCH_KD = 0;

PID_YAW_KP = 100/5
PID_YAW_KI = 100;
PID_YAW_KD = 0;


Ketar = PID_ROLL_KP + tf(PID_ROLL_KI,[1 0]);
Ketap = PID_ROLL_KP + tf(PID_ROLL_KI,[1 0]);
Ketay = PID_ROLL_KP + tf(PID_ROLL_KI,[1 0]);

% sisotool(Getar,Ketar,F)
% sisotool(Getap,Ketap,F)
% sisotool(Getay,Ketay,F)

Setar = feedback(1,Getar*Ketar*F);
Setap = feedback(1,Getap*Ketap*F);
Setay = feedback(1,Getay*Ketay*F);
% figure, bode(Setar)
MMetar = 1/norm(Setar,'inf')
MMetap = 1/norm(Setap,'inf')
MMetay = 1/norm(Setay,'inf')

Kroll = Kwr*[Ketar -1];
Kpitch = Kwp*[Ketap -1]; 
Kyaw = Kwy*[Ketay -1];

% definition of overal controller with input permutation
K = blkdiag(Kroll,Kpitch,Kyaw)*[1 0 0 0 0 0;
                                0 0 0 1 0 0;
                                0 1 0 0 0 0;
                                0 0 0 0 1 0;
                                0 0 1 0 0 0;
                                0 0 0 0 0 1];
