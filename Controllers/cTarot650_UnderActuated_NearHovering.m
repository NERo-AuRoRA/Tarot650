function [drone,cGains,cPar] = cTarot650_UnderActuated_NearHovering(drone,cGains,cPar)

% Controllers Gains.
% The Gains must be given in the folowing order
% [kxL1  kyL1 kzL1 kPhiL1 kthetaL1 kPsiL1 ; kxL2 kyL2  kzL2 kPhiL2 kthetaL2 kPsiL2]
% cGains = [0.5 0.5 5 1 1 1; 2 2 2 20 20 2.5];

if nargin < 3
    % List of controller paramenters
    
    cPar.ts  = 1/100; % Time sample
    
    % Dynamic Model Parameters
    cPar.g = 9.8;    % [kg.m/s^2] Gravitational acceleration
    cPar.maxAlt = 5000;
    
    % [kg] Tarot 650 mass
    cPar.m = 3.46;
    
    % [kg.m^2] Moments of Inertia - whole body - concentrated mass   
    cPar.Ix = 0.0415;
    cPar.Iy = 0.0415;
    cPar.Iz = 0.2782;
    
    % Low level parameters - 1st order system
    % Pitch rate
    cPar.K(1,1) = 0.2255;
    cPar.T(1,1) = 0.0580;
    
    % Roll rate
    cPar.K(2,1) = 0.9255;
    cPar.T(2,1) = 0.0580;
    
    % Yaw rate
    cPar.K(3,1) = 1.0011;
    cPar.T(3,1) = 0.1089;
    
    % Parameter of the collective force
    % Relationship between stick percentage and generated thrust (in g force)
    % T = a*u^n + b
    cPar.n = 0.5;    
    cPar.a =  150.4887; % 0.21055;
    cPar.b = -47.5887;  % -0.15011;
end

if nargin < 2
    % Default Controller Gains
    %cGains = [1 1 2 5 5 1; 5 5 10 10 10 5];
    cGains = [2 4 3 4 4 8; 1 1 2 15 15 2];
    %     cGains = [2 4 3 5 5 1; .2 .2 2 15 15 2.5];
    % disp('Gains not given. Using standard ones.');
end

% -------------------------------------------------
% System dynamics
% Translational dynamics
MMt = [cPar.m*cos(drone.pPos.X(6)), cPar.m*sin(drone.pPos.X(6)), -cPar.m*drone.pPos.X(5);...
    cPar.m*(-sin(drone.pPos.X(6))+drone.pPos.X(4)*drone.pPos.X(5)*cos(drone.pPos.X(6))), cPar.m*(cos(drone.pPos.X(6))+drone.pPos.X(4)*drone.pPos.X(5)*sin(drone.pPos.X(6))), cPar.m*drone.pPos.X(4);...
    cPar.m*(drone.pPos.X(5)*cos(drone.pPos.X(6))+drone.pPos.X(4)*sin(drone.pPos.X(6))), cPar.m*(drone.pPos.X(5)*sin(drone.pPos.X(6))-drone.pPos.X(4)*cos(drone.pPos.X(6))), cPar.m];

CCt = zeros(3,3);

GGt = [-drone.pPos.X(5)*cPar.m*cPar.g;...
    drone.pPos.X(4)*cPar.m*cPar.g;...
    cPar.m*cPar.g];

% Rotational dynamics
MMr = [cPar.Ix, 0, -cPar.Ix*drone.pPos.X(5);...
    0, cPar.Iy+cPar.Iz*drone.pPos.X(4)^2, cPar.Iy*drone.pPos.X(4)-cPar.Iz*drone.pPos.X(4);...
    -cPar.Ix*drone.pPos.X(5), cPar.Iy*drone.pPos.X(4)-cPar.Iz*drone.pPos.X(4), cPar.Ix*drone.pPos.X(5)^2+cPar.Iy*drone.pPos.X(4)^2+cPar.Iz];

CCr = [0, -cPar.Iz*drone.pPos.X(4)*drone.pPos.X(11)+1/2*(-cPar.Ix-cPar.Iy+cPar.Iz)*drone.pPos.X(12), 1/2*(-cPar.Ix-cPar.Iy+cPar.Iz)*drone.pPos.X(11)-cPar.Iy*drone.pPos.X(4)*drone.pPos.X(12);...
    cPar.Iz*drone.pPos.X(4)*drone.pPos.X(11)+1/2*(cPar.Ix+cPar.Iy-cPar.Iz)*drone.pPos.X(12), cPar.Iz*drone.pPos.X(4)*drone.pPos.X(10), 1/2*(cPar.Ix+cPar.Iy-cPar.Iz)*drone.pPos.X(10)-cPar.Ix*drone.pPos.X(5)*drone.pPos.X(12);...
    1/2*(-cPar.Ix+cPar.Iy-cPar.Iz)*drone.pPos.X(11)+(cPar.Ix*drone.pPos.X(5)+cPar.Iy*drone.pPos.X(4))*drone.pPos.X(12), 1/2*(-cPar.Ix+cPar.Iy-cPar.Iz)*drone.pPos.X(10)+cPar.Ix*drone.pPos.X(5)*drone.pPos.X(12), cPar.Iy*drone.pPos.X(4)*drone.pPos.X(10)+cPar.Ix*drone.pPos.X(5)*drone.pPos.X(11)];

GGr = zeros(3,1);

MM = [MMt zeros(3,3); zeros(3,3) MMr];
CC = [CCt zeros(3,3); zeros(3,3) CCr];
GG = [GGt; GGr];

% -------------------------------------------------
% Describing the system as an under-actuaded one
MMpp = MM(1:2,1:2);
MMpa = MM(1:2,3:6);
MMap = MM(3:6,1:2);
MMaa = MM(3:6,3:6);

CCpp = CC(1:2,1:2);
CCpa = CC(1:2,3:6);
CCap = CC(3:6,1:2);
CCaa = CC(3:6,3:6);

GGpp = GG(1:2,1);
GGaa = GG(3:6,1);

EEa = CCaa*drone.pPos.dX(9:12) + GGaa + MMap*drone.pPos.dX(7:8) + CCap*drone.pPos.dX(1:2);

% -------------------------------------------------
% Under-actuated controller
% Gains

KKL1a  = diag(cGains(1,3:6));
KKL2a  = diag(cGains(2,3:6));
KKL3a  = 2*sqrt(KKL1a);
KKL4a  = 2*sqrt(KKL1a*KKL2a)/KKL3a;

KKL1p  = diag(cGains(1,1:2));
KKL2p  = diag(cGains(2,1:2));
KKL3p  = 2*sqrt(KKL1p);
KKL4p  = 2*sqrt(KKL1p*KKL2p)/KKL3p;


drone.pPos.Xtil = drone.pPos.Xd-drone.pPos.X;

mup = drone.pPos.dXd(1:2) + KKL1p*tanh(KKL2p*drone.pPos.Xtil(7:8))  + KKL3p*tanh(KKL4p*drone.pPos.Xtil(1:2));
mua = drone.pPos.dXd(3:6) + KKL1a*tanh(KKL2a*drone.pPos.Xtil(9:12)) + KKL3a*tanh(KKL4a*drone.pPos.Xtil(3:6));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 02/11/2021: ASBrandao
% Using absolute value of eta_z to compute the roll and pitch referecenes
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

drone.pPos.Xd(10) = (-drone.pPos.Xd(4)+(atan2((mup(1)*sin(drone.pPos.X(6))-mup(2)*cos(drone.pPos.X(6)))*cos(drone.pPos.X(5)),(abs(mua(1))+cPar.g))))/cPar.ts;
drone.pPos.Xd(11) = (-drone.pPos.Xd(5)+(atan2((mup(1)*cos(drone.pPos.X(6))+mup(2)*sin(drone.pPos.X(6))),(abs(mua(1))+cPar.g))))/cPar.ts;

drone.pPos.Xd(4) = atan2((mup(1)*sin(drone.pPos.X(6))-mup(2)*cos(drone.pPos.X(6)))*cos(drone.pPos.X(5)),(abs(mua(1))+cPar.g));
drone.pPos.Xd(5) = atan2((mup(1)*cos(drone.pPos.X(6))+mup(2)*sin(drone.pPos.X(6))),(abs(mua(1))+cPar.g));

drone.pPos.Xtil = drone.pPos.Xd-drone.pPos.X;
mua = drone.pPos.dXd(3:6) + KKL1a*tanh(KKL2a*drone.pPos.Xtil(9:12)) + KKL3a*tanh(KKL4a*drone.pPos.Xtil(3:6));

drone.pSC.Td = MMaa*mua + EEa;
% -------------------------------------------------
% Low-level controller
W  = [1 0 -drone.pPos.X(5);  0 1 drone.pPos.X(4);  0 -drone.pPos.X(4)  1];
dW = [0 0 -drone.pPos.X(11); 0 0 drone.pPos.X(10); 0 -drone.pPos.X(10) 0];

ddEta  = diag([cPar.Ix cPar.Iy cPar.Iz])\drone.pSC.Td(2:4);

Omega  = W*drone.pPos.X(10:12);
dOmega = dW*drone.pPos.X(10:12) + W*ddEta;

drone.pSC.Ud(1:3) = tanh(diag(cPar.K)\(diag(cPar.T)*dOmega + Omega));
drone.pSC.Ud(4)   = ((drone.pSC.Td(1) - cPar.b)/cPar.a)^(1/cPar.n);
if drone.pSC.Ud(4) < 0
    drone.pSC.Ud(4) = 0;
else
    if drone.pSC.Ud(4) > 1
        drone.pSC.Ud(4) = 1;
    end
end





