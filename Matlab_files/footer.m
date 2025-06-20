    % rate and attitude PID controller parameters
    PID_JB=1; % 1 to use custom PID parameter, 0 to use Bitcraze's
    
    if(PID_JB)     
    PID_ROLL_RATE_KP                    = 150;%350;
    PID_ROLL_RATE_KI                    = 0;
    PID_ROLL_RATE_KD                    = 0;
    PID_ROLL_RATE_INTEGRATION_LIMIT     = 33.3; 
    
    PID_PITCH_RATE_KP                   = PID_ROLL_RATE_KP; 
    PID_PITCH_RATE_KI                   = PID_ROLL_RATE_KI;
    PID_PITCH_RATE_KD                   = PID_ROLL_RATE_KD;
    PID_PITCH_RATE_INTEGRATION_LIMIT    = 33.3; 
    
    PID_YAW_RATE_KP                     = 115;%150;
    PID_YAW_RATE_KI                     = 0;
    PID_YAW_RATE_KD                     = 0;
    PID_YAW_RATE_INTEGRATION_LIMIT      = 166.7;
    
    PID_ROLL_KP                         = 32;%15;
    PID_ROLL_KI                         = 10;%5;
    PID_ROLL_KD                         = 0;
    PID_ROLL_INTEGRATION_LIMIT          = 20.0;
    
    PID_PITCH_KP                        = PID_ROLL_KP;
    PID_PITCH_KI                        = PID_ROLL_KI;
    PID_PITCH_KD                        = PID_ROLL_KD;
    PID_PITCH_INTEGRATION_LIMIT         = 20.0;
    
    PID_YAW_KP                          = 32;
    PID_YAW_KI                          = 10;%15;5;
    PID_YAW_KD                          = 0;
    PID_YAW_INTEGRATION_LIMIT           = 360.0; 
    
    else
    PID_ROLL_RATE_KP                    = 250.0;
    PID_ROLL_RATE_KI                    = 500.0;
    PID_ROLL_RATE_KD                    = 2.5;
    PID_ROLL_RATE_INTEGRATION_LIMIT     = 33.3; 
    
    PID_PITCH_RATE_KP                   = PID_ROLL_RATE_KP; 
    PID_PITCH_RATE_KI                   = PID_ROLL_RATE_KI;
    PID_PITCH_RATE_KD                   = PID_ROLL_RATE_KD;
    PID_PITCH_RATE_INTEGRATION_LIMIT    = 33.3; 
    
    PID_YAW_RATE_KP                     = 120.0;
    PID_YAW_RATE_KI                     = 16.7;
    PID_YAW_RATE_KD                     = 0;
    PID_YAW_RATE_INTEGRATION_LIMIT      = 166.7;
    
    PID_ROLL_KP                         = 6.0;
    PID_ROLL_KI                         = 3.0;
    PID_ROLL_KD                         = 0;
    PID_ROLL_INTEGRATION_LIMIT          = 20.0;
    
    PID_PITCH_KP                        = PID_ROLL_KP;
    PID_PITCH_KI                        = PID_ROLL_KI;
    PID_PITCH_KD                        = PID_ROLL_KD;
    PID_PITCH_INTEGRATION_LIMIT         = 20.0;
    
    PID_YAW_KP                          = 6.0;
    PID_YAW_KI                          = 1.0;
    PID_YAW_KD                          = 0.35;
    PID_YAW_INTEGRATION_LIMIT           = 360.0; 
    end


    % Integrator initialization
        PID_ROLL_RATE_INIT      = 0 ;
        PID_PITCH_RATE_INIT     = 0;
        PID_YAW_RATE_INIT       = 0;

        PID_ROLL_INIT           = 0;
        PID_PITCH_INIT          = 0;
        PID_YAW_INIT            = 0;
        PID_YAW_INITD           = 0;

% pidVZ.kp = 25;
% pidVZ.ki = 15;
% pidVZ.kd = 0;
