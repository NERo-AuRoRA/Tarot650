function sDynamicModel(tarot)

% Dynamic model from
% Brandão, A. S., M. Sarcinelli-Filho, and R. Carelli. 
% "High-level underactuated nonlinear control for rotorcraft machines." 
% Mechatronics (ICM), 2013 IEEE International Conference on. IEEE, 2013.
%
% Daniel, S. (2021). 
% Fyzická interakce malé vícerotorové bezpilotní helikoptéry s člověkem 
% Master's thesis, České vysoké učení technické v Praze. 
% Vypočetní a informační centrum.
%
% Simulate ArDrone dynamic model
%
%      +----------+  W   +--------+  F   +----------+  T   +-------+
% U -> | Actuator |  ->  | Rotary |  ->  | Forces & |  ->  | Rigid |  -> X
%      | Dynamics |      | Wing   |      | Torques  |      | Body  |
%      +----------+      +--------+      +----------+      +-------+
%


% 1: Receive input signal
%     pitch rate         | [-1,1] <==> [X,X] degrees
%     roll rate          | [-1,1] <==> [X,X] degrees
%     altitude rate      | [ 0,1] <==> [X,X] m/s
%     yaw rate           | [-1,1] <==> [X,X] degrees/s

tarot.pPar.Xra = tarot.pPar.Xr;

% Low-level relationship
% Reference values

W  = [1 0 -tarot.pPos.X(5);  0 1 tarot.pPos.X(4);  0 -tarot.pPos.X(4)  1];
dW = [0 0 -tarot.pPos.X(11); 0 0 tarot.pPos.X(10); 0 -tarot.pPos.X(10) 0];

tarot.pSC.Or = W*tarot.pPos.X(10:12);

% Angular rate in the body frame
tarot.pSC.dOr(1,1) = 1/tarot.pPar.T(1,1)*(tarot.pPar.K(1)*tarot.pSC.Ud(1) - tarot.pSC.Or(1));
tarot.pSC.dOr(2,1) = 1/tarot.pPar.T(2,1)*(tarot.pPar.K(2)*tarot.pSC.Ud(2) - tarot.pSC.Or(2));
tarot.pSC.dOr(3,1) = 1/tarot.pPar.T(3,1)*(tarot.pPar.K(3)*tarot.pSC.Ud(3) - tarot.pSC.Or(3));

% Thrust in the body frame
tarot.pSC.Thr = tarot.pPar.a*tarot.pSC.Ud(4)^tarot.pPar.n + tarot.pPar.b;
% REMOVE after discussion --------------------------
% tarot.pSC.Ur = ((tarot.pSC.Ud(3) - tarot.pPar.b)/tarot.pPar.a)^2;
% tarot.pSC.dThr = 1/tarot.pPar.T(3,1)*(tarot.pPar.K(3)*tarot.pSC.Ur - tarot.pSC.Thr);
% tarot.pSC.Thr = tarot.pSC.Thr + tarot.pSC.dThr*tarot.pPar.ts;
% --------------------------------------------------

% Matrix of velocities transformation from Body-frame to Inertial-frame
% Matrix simplified for navigation with small attitude angles
tarot.pPos.dXr(10:12) = W\(tarot.pSC.dOr - dW*tarot.pPos.X(10:12));

% Transformation of angles' acceleration to torques
% Arbitrary gains (Replace then by the moments of inertia)
tarot.pSC.Tau = diag([tarot.pPar.Ix tarot.pPar.Iy tarot.pPar.Iz])*tarot.pPos.dXr(10:12);


% Tait-Brayn rotation matrix
Rx = [1 0 0; 0 cos(tarot.pPos.X(4)) -sin(tarot.pPos.X(4)); 0 sin(tarot.pPos.X(4)) cos(tarot.pPos.X(4))];
Ry = [cos(tarot.pPos.X(5)) 0 sin(tarot.pPos.X(5)); 0 1 0; -sin(tarot.pPos.X(5)) 0 cos(tarot.pPos.X(5))];
Rz = [cos(tarot.pPos.X(6)) -sin(tarot.pPos.X(6)) 0; sin(tarot.pPos.X(6)) cos(tarot.pPos.X(6)) 0; 0 0 1];

R = Rz*Ry*Rx;

tarot.pSC.F = [R*[0; 0; tarot.pSC.Thr]; tarot.pSC.Tau];

% -------------------------------------------------
% System dynamics
% Translational dynamics
MMt = [tarot.pPar.m*cos(tarot.pPos.X(6)), tarot.pPar.m*sin(tarot.pPos.X(6)), -tarot.pPar.m*tarot.pPos.X(5);...
    tarot.pPar.m*(-sin(tarot.pPos.X(6))+tarot.pPos.X(4)*tarot.pPos.X(5)*cos(tarot.pPos.X(6))), tarot.pPar.m*(cos(tarot.pPos.X(6))+tarot.pPos.X(4)*tarot.pPos.X(5)*sin(tarot.pPos.X(6))), tarot.pPar.m*tarot.pPos.X(4);...
    tarot.pPar.m*(tarot.pPos.X(5)*cos(tarot.pPos.X(6))+tarot.pPos.X(4)*sin(tarot.pPos.X(6))), tarot.pPar.m*(tarot.pPos.X(5)*sin(tarot.pPos.X(6))-tarot.pPos.X(4)*cos(tarot.pPos.X(6))), tarot.pPar.m];

CCt = zeros(3,3);

GGt = [-tarot.pPos.X(5)*tarot.pPar.m*tarot.pPar.g;...
    tarot.pPos.X(4)*tarot.pPar.m*tarot.pPar.g;...
    tarot.pPar.m*tarot.pPar.g];

% Rotational dynamics
MMr = [tarot.pPar.Ix, 0, -tarot.pPar.Ix*tarot.pPos.X(5);...
    0, tarot.pPar.Iy+tarot.pPar.Iz*tarot.pPos.X(4)^2, tarot.pPar.Iy*tarot.pPos.X(4)-tarot.pPar.Iz*tarot.pPos.X(4);...
    -tarot.pPar.Ix*tarot.pPos.X(5), tarot.pPar.Iy*tarot.pPos.X(4)-tarot.pPar.Iz*tarot.pPos.X(4), tarot.pPar.Ix*tarot.pPos.X(5)^2+tarot.pPar.Iy*tarot.pPos.X(4)^2+tarot.pPar.Iz];

CCr = [0, -tarot.pPar.Iz*tarot.pPos.X(4)*tarot.pPos.X(11)+1/2*(-tarot.pPar.Ix-tarot.pPar.Iy+tarot.pPar.Iz)*tarot.pPos.X(12), 1/2*(-tarot.pPar.Ix-tarot.pPar.Iy+tarot.pPar.Iz)*tarot.pPos.X(11)-tarot.pPar.Iy*tarot.pPos.X(4)*tarot.pPos.X(12);...
    tarot.pPar.Iz*tarot.pPos.X(4)*tarot.pPos.X(11)+1/2*(tarot.pPar.Ix+tarot.pPar.Iy-tarot.pPar.Iz)*tarot.pPos.X(12), tarot.pPar.Iz*tarot.pPos.X(4)*tarot.pPos.X(10), 1/2*(tarot.pPar.Ix+tarot.pPar.Iy-tarot.pPar.Iz)*tarot.pPos.X(10)-tarot.pPar.Ix*tarot.pPos.X(5)*tarot.pPos.X(12);...
    1/2*(-tarot.pPar.Ix+tarot.pPar.Iy-tarot.pPar.Iz)*tarot.pPos.X(11)+(tarot.pPar.Ix*tarot.pPos.X(5)+tarot.pPar.Iy*tarot.pPos.X(4))*tarot.pPos.X(12), 1/2*(-tarot.pPar.Ix+tarot.pPar.Iy-tarot.pPar.Iz)*tarot.pPos.X(10)+tarot.pPar.Ix*tarot.pPos.X(5)*tarot.pPos.X(12), tarot.pPar.Iy*tarot.pPos.X(4)*tarot.pPos.X(10)+tarot.pPar.Ix*tarot.pPos.X(5)*tarot.pPos.X(11)];

GGr = zeros(3,1);

MM = [MMt zeros(3,3); zeros(3,3) MMr];
CC = [CCt zeros(3,3); zeros(3,3) CCr];
GG = [GGt; GGr];

% -------------------------------------------------
% Numerical integration
tarot.pPos.dX(7:12) = MM\(tarot.pSC.F - CC*tarot.pPos.X(7:12) - GG);
tarot.pPos.dX(1:6)  = tarot.pPos.dX(1:6) + tarot.pPos.dX(7:12)*tarot.pPar.ts;
tarot.pPos.X(7:12) = tarot.pPos.dX(1:6);
tarot.pPos.X(1:6)   = tarot.pPos.X(1:6) + tarot.pPos.X(7:12)*tarot.pPar.ts;

% Heading angle correction: limits [-pi,+pi]
if tarot.pPos.X(6) > pi
    tarot.pPos.X(6) = -2*pi + tarot.pPos.X(6);
end
if tarot.pPos.X(6) < -pi
    tarot.pPos.X(6) = 2*pi + tarot.pPos.X(6);
end