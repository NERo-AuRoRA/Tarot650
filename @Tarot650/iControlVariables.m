function iControlVariables(tarot)

% Tarot 650-Sport
% ========================================================================
% Robot pose
tarot.pPos.X    = zeros(12,1); % Current pose (point of control)
% tarot.pPos.X(3) = 0.2575;      % Posição de repouso 
tarot.pPos.Xa   = zeros(12,1); % Past pose

tarot.pPos.Xc   = zeros(12,1); % Current pose (center of the robot)
tarot.pPos.Xp   = zeros(12,1); % Current pose (computed by the robot)

tarot.pPos.Xd   = zeros(12,1); % Desired pose
tarot.pPos.Xda  = zeros(12,1); % Past desired pose

tarot.pPos.Xr   = zeros(12,1); % Reference pose
tarot.pPos.Xra  = zeros(12,1); % Past reference pose

% First time derivative 
tarot.pPos.dX   = zeros(12,1); % Current pose
tarot.pPos.dXd  = zeros(12,1); % Desired pose
tarot.pPos.dXr  = zeros(12,1); % Reference pose

% Pose error
tarot.pPos.Xtil = tarot.pPos.Xd - tarot.pPos.X; 


%% ---

tarot.pPos.Xo   = zeros(12,1);    % Bias pose: Calibration
 
tarot.pPos.dXd  = zeros(12,1);    % Desired first derivative Pose 

tarot.pPos.Xtil = zeros(12,1);    % Posture Error

tarot.pPos.dX   = zeros(12,1);    % First derivative

tarot.pSC.U     = zeros(4,1);     % Control Signal
tarot.pSC.Ur    = zeros(4,1);     % Reference control signal 
tarot.pSC.Ud    = zeros(4,1);     % Desired Control Signal (sent to robot)

tarot.pSC.Wd    = zeros(4,1);     % Desired rotor velocity;
tarot.pSC.Xr    = zeros(12,1);    % Reference pose

tarot.pSC.Or    = zeros(3,1);     % Angular velocities in the body-frame
tarot.pSC.dOr   = zeros(3,1);     
tarot.pSC.Thr   = zeros(1,1);     % Thrust in the body-frame

tarot.pSC.D     = zeros(6,1);     % Disturbance Vector


