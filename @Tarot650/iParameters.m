function iParameters(tarot)

tarot.pPar.Model = 'Tarot 650-Sport'; % robot model

% Sample time
tarot.pPar.Ts = 0.1; % For numerical integration
tarot.pPar.ti = tic; % Flag time

% Dynamic Model Parameters 
tarot.pPar.g = 9.8;    % [kg.m/s^2] Gravitational acceleration

