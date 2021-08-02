function mCADcolor(drone,color)
% Modify drone color

if nargin > 1
    drone.pCAD.mtl{1}(4).Kd = color';
    drone.pCAD.mtl{2}(1).Kd = color';
    drone.pCAD.mtl{3}(1).Kd = color';
end


    for idx = 3:3 % esq
        for ii = 1:length(drone.pCAD.obj{idx}.umat3)
            mtlnum = drone.pCAD.obj{idx}.umat3(ii);
            for jj=1:length(drone.pCAD.mtl{idx})
                if strcmp(drone.pCAD.mtl{idx}(jj).name,drone.pCAD.obj{idx}.usemtl(mtlnum-1))
                    break;
                end
            end
            fvcd3(ii,:) = drone.pCAD.mtl{idx}(jj).Kd';
        end

        drone.pCAD.i3D{idx}.FaceVertexCData  = fvcd3;
    end

    
    for idx = 2:2  % dir
        for ii = 1:length(drone.pCAD.obj{idx}.umat3)
            mtlnum = drone.pCAD.obj{idx}.umat3(ii);
            for jj=1:length(drone.pCAD.mtl{idx})
                if strcmp(drone.pCAD.mtl{idx}(jj).name,drone.pCAD.obj{idx}.usemtl(mtlnum-1))
                    break;
                end
            end
            fvcd3(ii,:) = drone.pCAD.mtl{idx}(jj).Kd';
        end

        drone.pCAD.i3D{idx}.FaceVertexCData  = fvcd3;
    end
        
    % - body
    for ii = 1:length(drone.pCAD.obj{1}.umat3)
        mtlnum = drone.pCAD.obj{1}.umat3(ii);
        for jj=1:length(drone.pCAD.mtl{1})
            if strcmp(drone.pCAD.mtl{1}(jj).name,drone.pCAD.obj{1}.usemtl(mtlnum-1))
                break;
            end
        end
        fvcd3(ii,:) = drone.pCAD.mtl{1}(jj).Kd';
    end

    drone.pCAD.i3D{1}.FaceVertexCData  = fvcd3;
end