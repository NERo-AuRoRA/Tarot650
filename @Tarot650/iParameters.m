function iParameters(tarot)

tarot.pPar.Model = 'Tarot 650-Sport'; % robot model
tarot.pPar.ip = ['192.168.1.' num2str(tarot.pID)];

% Sample time
tarot.pPar.t   = tic; % Current Time
tarot.pPar.ts  = 0.01; % Tarot 650
tarot.pPar.ti  = 0; % Flag time 

% Reference for Tarot Parameter
% https://github.com/ctu-mrs/mrs_simulation/blob/master/models/mrs_robots_description/urdf/t650.xacro#L113-L119

% Dynamic Model Parameters 
tarot.pPar.g = 9.8;    % [kg.m/s^2] Gravitational acceleration
tarot.pPar.maxAlt = 5000; 

% [kg] Tarot 650 mass
tarot.pPar.m = 3.46; 

% [kg.m^2] Moments of Inertia
tarot.pPar.Ix = 0.0415; 
tarot.pPar.Iy = 0.0415;
tarot.pPar.Iz = 0.2782;

% Low level parameters - 1st order system
% Pitch rate
tarot.pPar.K(1,1) = 0.2255;
tarot.pPar.T(1,1) = 0.0580;

% Roll rate
tarot.pPar.K(2,1) = 0.9255;
tarot.pPar.T(2,1) = 0.0580;

% Yaw rate
tarot.pPar.K(3,1) = 1.0011;
tarot.pPar.T(3,1) = 0.1089;

% Parameter of the collective force
% Relationship between stick percentage and generated thrust (in g force)
% T = a*u^n + b
% 0.1 -> g
% 1.0 -> 2g
tarot.pPar.n = 0.5;
u_F = [0.1 -tarot.pPar.g; 1 2*tarot.pPar.g]; 

tarot.pPar.a = tarot.pPar.m*((u_F(2,2) - u_F(1,2))/(u_F(2,1)^tarot.pPar.n - u_F(1,1)^tarot.pPar.n));    % 0.21055; 
tarot.pPar.b = tarot.pPar.m*(u_F(2,2) + tarot.pPar.g) - tarot.pPar.a*u_F(2,1)^tarot.pPar.n;  % -0.15011; 

% Altitude rate - Collective thrust
% tarot.pPar.K(4,1) = 14.0100;
% tarot.pPar.T(4,1) = 0.1545;

% Pose reference
tarot.pPar.Xr  = zeros(12,1); 
tarot.pPar.Xra = zeros(12,1); 

% Rotor velocities
tarot.pPar.W = zeros(4,1); 

% Model disturbances
tarot.pPar.D = zeros(6,1); 
tarot.pPar.Q = zeros(3,1); 
