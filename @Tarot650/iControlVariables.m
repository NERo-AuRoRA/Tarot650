function iControlVariables(tarot)

% Tarot 650-Sport
% ========================================================================
% Robot pose
tarot.pPos.X    = zeros(12,1); % Current pose (point of control)
tarot.pPos.X(3) = 0.2575;      % Posição de repouso 
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

