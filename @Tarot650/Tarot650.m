classdef Tarot650 < handle
    % In a methods block, set the method attributes
    % and add the function signature
    properties
        
        % Properties or Parameters
        pCAD   % Tarot 650-Sport 3D image
        pPar   % Parameters
        pID    % Identification
        
        % Control variables
        pPos   % Posture
        pSC    % Signals
        pFlag  % Flags
        pHom   % Homogeneous Transform
        
        % Navigation Data and Communication
        pData % Flight Data
        pCom  % Communication  
        

    end
    
    methods
        function car = Tarot650(ID)
            if nargin < 1
                ID = 1;
            end
            car.pID = ID;
                
            
            drone.pFlag.Connected = 0;
            
            
            iControlVariables(car);
            iParameters(car);
            mCADload(car);
      
        end
        
        % ==================================================
        iControlVariables(car);
        iParameters(car);
        
        % ==================================================
        % Tarot 3D Image
        mCADload(car);
        mCADplot(car,ax,sk);
        mCADcolor(car,color);
        % mCADmakeSkeleton(car);

        % ==================================================
        % ArDrone Models for simulation
        %sDynamicModel(drone);      
        sDynamicModelSimplify(drone)
      
    end
end