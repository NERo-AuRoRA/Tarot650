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
        pFlag  % Flags
        pHom   % Homogeneous Transform
        

    end
    
    methods
        function car = Tarot650(ID)
            if nargin < 1
                ID = 1;
            end
            car.pID = ID;
                
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

      
    end
end