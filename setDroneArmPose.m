%% Função para alterar a pose do manipulador aéreo

function setDroneArmPose(drone,manipulator,dronePose,manipPose)

    % -- Drone Posture
    drone.pPos.Xc(1:6) = dronePose;
    manipulator.pPos.q = manipPose;

    drone.mCADplot;
    manipulator.mCADplot(drone(1));
    
    drawnow limitrate nocallbacks
    
%     axis tight
    

end