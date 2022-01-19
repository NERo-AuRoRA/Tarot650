function sDynamicModelSimplify(drone)

% -------------------------------------------------
% System dynamics
% Translational dynamics
MMt = drone.pPar.m*[cos(drone.pPos.X(6)), sin(drone.pPos.X(6)), -drone.pPos.X(5);...
    (-sin(drone.pPos.X(6))+drone.pPos.X(4)*drone.pPos.X(5)*cos(drone.pPos.X(6))), (cos(drone.pPos.X(6))+drone.pPos.X(4)*drone.pPos.X(5)*sin(drone.pPos.X(6))), drone.pPos.X(4);...
    (drone.pPos.X(4)*sin(drone.pPos.X(6))+drone.pPos.X(5)*cos(drone.pPos.X(6))), (drone.pPos.X(5)*sin(drone.pPos.X(6))-drone.pPos.X(4)*cos(drone.pPos.X(6))), 1];

CCt = zeros(3,3);

GGt = drone.pPar.m*drone.pPar.g*...
    [-drone.pPos.X(5); 
    drone.pPos.X(4);...
    1];

% Rotational dynamics
MMr = [drone.pPar.Ixx, 0, -drone.pPar.Ixx*drone.pPos.X(5);...
    0, drone.pPar.Iyy+drone.pPar.Izz*drone.pPos.X(4)^2, drone.pPar.Iyy*drone.pPos.X(4)-drone.pPar.Izz*drone.pPos.X(4);...
    -drone.pPar.Ixx*drone.pPos.X(5), drone.pPar.Iyy*drone.pPos.X(4)-drone.pPar.Izz*drone.pPos.X(4), drone.pPar.Ixx*drone.pPos.X(5)^2+drone.pPar.Iyy*drone.pPos.X(4)^2+drone.pPar.Izz];

CCr = [0, -drone.pPar.Izz*drone.pPos.X(4)*drone.pPos.X(11)+1/2*(-drone.pPar.Ixx-drone.pPar.Iyy+drone.pPar.Izz)*drone.pPos.X(12), 1/2*(-drone.pPar.Ixx-drone.pPar.Iyy+drone.pPar.Izz)*drone.pPos.X(11)-drone.pPar.Iyy*drone.pPos.X(4)*drone.pPos.X(12);...
    drone.pPar.Izz*drone.pPos.X(4)*drone.pPos.X(11)+1/2*(drone.pPar.Ixx+drone.pPar.Iyy-drone.pPar.Izz)*drone.pPos.X(12), drone.pPar.Izz*drone.pPos.X(4)*drone.pPos.X(10), 1/2*(drone.pPar.Ixx+drone.pPar.Iyy-drone.pPar.Izz)*drone.pPos.X(10)-drone.pPar.Ixx*drone.pPos.X(5)*drone.pPos.X(12);...
    1/2*(-drone.pPar.Ixx+drone.pPar.Iyy-drone.pPar.Izz)*drone.pPos.X(11)+drone.pPar.Iyy*drone.pPos.X(4)*drone.pPos.X(12), 1/2*(-drone.pPar.Ixx+drone.pPar.Iyy-drone.pPar.Izz)*drone.pPos.X(10)+drone.pPar.Ixx*drone.pPos.X(5)*drone.pPos.X(12), drone.pPar.Iyy*drone.pPos.X(4)*drone.pPos.X(10)+drone.pPar.Ixx*drone.pPos.X(5)*drone.pPos.X(11)];

GGr = zeros(3,1);

MM = [MMt zeros(3,3); zeros(3,3) MMr];
CC = [CCt zeros(3,3); zeros(3,3) CCr];
GG = [GGt; GGr];


% -------------------------------------------------
% Numerical integration
drone.pPos.dX(7:12) = MM\([0; 0; drone.pSC.Ud] - CC*drone.pPos.dX(1:6) - GG);
drone.pPos.dX(1:6)  = drone.pPos.dX(1:6) + drone.pPar.Ts*drone.pPos.dX(7:12);
drone.pPos.X(7:12)  = drone.pPos.dX(1:6);
drone.pPos.X(1:6)   = drone.pPos.X(1:6) + drone.pPar.Ts*drone.pPos.X(7:12);

% Limiting the heading angle
if drone.pPos.X(6) >  pi
    drone.pPos.X(6) = -2*pi + drone.pPos.X(6);
end
if drone.pPos.X(6) < -pi
    drone.pPos.X(6) =  2*pi + drone.pPos.X(6);
end

end