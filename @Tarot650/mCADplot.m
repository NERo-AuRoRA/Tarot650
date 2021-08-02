function mCADplot(uav,axis,skeleton)
% Plot Tarot 650-Sport CAD model on its current position
% drone.pPos.Xc = [x y z psi theta phi dx dy dz dpsi dtheta dphi]^T


uav.pPos.Xc = uav.pPos.X;
uav.pPos.Xc(3) = uav.pPos.X(3) - 0.2575;


if nargin < 2
    uav.pCAD.flagFrame = 0; 
    uav.pCAD.flagSkeleton = 0;
end


if uav.pCAD.flagCreated == 0
    uav.pPos.psiHel = 0;
    uav.pCAD.flagFrame = 1;
    
    mCADmake(uav)
    
    if nargin < 3
        mCADplot(uav,axis)
    elseif nargin < 2
        mCADplot(uav)
    else
        mCADplot(uav,axis,skeleton)
    end
    
    uav.pPos.psiHel = -pi/3;
    
    drawnow limitrate nocallbacks
    
%     for jj = 1:length(uav.pCAD.i3D)-3
%         uav.pCAD.i3D{jj}.Visible = 'on';
%     end
    
else
    
    if nargin < 2
        uav.pCAD.flagFrame = 0;
        uav.pCAD.flagSkeleton = 0;
    else
        uav.pCAD.flagFrame = axis;
        uav.pCAD.flagSkeleton = 0;
        
        if nargin == 3
            uav.pCAD.flagSkeleton = skeleton;
        end
    end
        
    %------------------------------------------------------------
    % Update robot pose
    %%% Rotational matrix
    RotX = [1 0 0; 0 cos(uav.pPos.Xc(4)) -sin(uav.pPos.Xc(4)); 0 sin(uav.pPos.Xc(4)) cos(uav.pPos.Xc(4))];
    RotY = [cos(uav.pPos.Xc(5)) 0 sin(uav.pPos.Xc(5)); 0 1 0; -sin(uav.pPos.Xc(5)) 0 cos(uav.pPos.Xc(5))];
    RotZ = [cos(uav.pPos.Xc(6)) -sin(uav.pPos.Xc(6)) 0; sin(uav.pPos.Xc(6)) cos(uav.pPos.Xc(6)) 0; 0 0 1];
    
    Rot = RotZ*RotY*RotX;
    
    if uav.pCAD.flagFrame 
        
        Hframe = [Rot uav.pPos.Xc(1:3)+[0;0;0.2575]; 0 0 0 1];
        
        uav.pCAD.i3D{6}.Visible = 'on';
        uav.pCAD.i3D{7}.Visible = 'on';
        uav.pCAD.i3D{8}.Visible = 'on';
        
        xAxis = [uav.pCAD.OriginalAxis{1}(1,:); uav.pCAD.OriginalAxis{2}(1,:); uav.pCAD.OriginalAxis{3}(1,:)];
        vertBase = Hframe*[xAxis; ones(1,size(xAxis,2))];
        uav.pCAD.i3D{6}.XData = vertBase(1,:)';
        uav.pCAD.i3D{6}.YData = vertBase(2,:)';
        uav.pCAD.i3D{6}.ZData = vertBase(3,:)';
        
        yAxis = [uav.pCAD.OriginalAxis{1}(2,:); uav.pCAD.OriginalAxis{2}(2,:); uav.pCAD.OriginalAxis{3}(2,:)];
        vertBase = Hframe*[yAxis; ones(1,size(yAxis,2))];
        uav.pCAD.i3D{7}.XData = vertBase(1,:)';
        uav.pCAD.i3D{7}.YData = vertBase(2,:)';
        uav.pCAD.i3D{7}.ZData = vertBase(3,:)';
        
        zAxis = [uav.pCAD.OriginalAxis{1}(3,:); uav.pCAD.OriginalAxis{2}(3,:); uav.pCAD.OriginalAxis{3}(3,:)]; 
        vertBase = Hframe*[zAxis; ones(1,size(zAxis,2))];
        uav.pCAD.i3D{8}.XData = vertBase(1,:)';
        uav.pCAD.i3D{8}.YData = vertBase(2,:)';
        uav.pCAD.i3D{8}.ZData = vertBase(3,:)';
        
    else
        
        uav.pCAD.i3D{6}.Visible = 'off';
        uav.pCAD.i3D{7}.Visible = 'off';
        uav.pCAD.i3D{8}.Visible = 'off';
    end
    
    
    
    H = [Rot uav.pPos.Xc(1:3); 0 0 0 1];
    
    vis = uav.pCAD.flagSkeleton;
    
    if ~vis
        
        vertices = H*[uav.pCAD.obj{1}.v; ones(1,size(uav.pCAD.obj{1}.v,2))];
        uav.pCAD.i3D{1}.Vertices = vertices(1:3,:)';



        %%% Rotational matrix (H�lice Frente Dir.)
        RotX = [1 0 0; 0 cos(uav.pPos.Xc(4)) -sin(uav.pPos.Xc(4)); 0 sin(uav.pPos.Xc(4)) cos(uav.pPos.Xc(4))];
        RotY = [cos(uav.pPos.Xc(5)) 0 sin(uav.pPos.Xc(5)); 0 1 0; -sin(uav.pPos.Xc(5)) 0 cos(uav.pPos.Xc(5))];
        RotZ = [cos(uav.pPos.Xc(6)) -sin(uav.pPos.Xc(6)) 0; sin(uav.pPos.Xc(6)) cos(uav.pPos.Xc(6)) 0; 0 0 1];

        Rot = RotZ*RotY*RotX;
        H1 = [Rot [0.23450; 0.23625; 0.28800]; 0 0 0 1];


        Rot_J1 = [ cos(uav.pPos.psiHel) sin(uav.pPos.psiHel) 0;
                  -sin(uav.pPos.psiHel) cos(uav.pPos.psiHel) 0;
                            0                    0           1];

        vertices = H*H1*[Rot_J1*uav.pCAD.obj{2}.v; ones(1,size(uav.pCAD.obj{2}.v,2))];
        uav.pCAD.i3D{2}.Vertices = vertices(1:3,:)';



        %%% Rotational matrix (H�lice Tras Esq.)
        RotX = [1 0 0; 0 cos(uav.pPos.Xc(4)) -sin(uav.pPos.Xc(4)); 0 sin(uav.pPos.Xc(4)) cos(uav.pPos.Xc(4))];
        RotY = [cos(uav.pPos.Xc(5)) 0 sin(uav.pPos.Xc(5)); 0 1 0; -sin(uav.pPos.Xc(5)) 0 cos(uav.pPos.Xc(5))];
        RotZ = [cos(uav.pPos.Xc(6)) -sin(uav.pPos.Xc(6)) 0; sin(uav.pPos.Xc(6)) cos(uav.pPos.Xc(6)) 0; 0 0 1];

        Rot = RotZ*RotY*RotX;
        H1 = [Rot [-0.23450; -0.23625; 0.28800]; 0 0 0 1];

        Rot_J1 = [ cos(uav.pPos.psiHel + pi/2) sin(uav.pPos.psiHel + pi/2) 0;
                  -sin(uav.pPos.psiHel + pi/2) cos(uav.pPos.psiHel + pi/2) 0;
                                0                           0              1];

        vertices = H*H1*[Rot_J1*uav.pCAD.obj{3}.v; ones(1,size(uav.pCAD.obj{3}.v,2))];
        uav.pCAD.i3D{5}.Vertices = vertices(1:3,:)';


        %%% Rotational matrix (H�lice Frent Esq.)
        RotX = [1 0 0; 0 cos(uav.pPos.Xc(4)) -sin(uav.pPos.Xc(4)); 0 sin(uav.pPos.Xc(4)) cos(uav.pPos.Xc(4))];
        RotY = [cos(uav.pPos.Xc(5)) 0 sin(uav.pPos.Xc(5)); 0 1 0; -sin(uav.pPos.Xc(5)) 0 cos(uav.pPos.Xc(5))];
        RotZ = [cos(uav.pPos.Xc(6)) -sin(uav.pPos.Xc(6)) 0; sin(uav.pPos.Xc(6)) cos(uav.pPos.Xc(6)) 0; 0 0 1];

        Rot = RotZ*RotY*RotX;
        H1 = [Rot [0.23450; -0.23625; 0.28800]; 0 0 0 1];


        Rot_J1 = [ cos(-(uav.pPos.psiHel)) sin(-(uav.pPos.psiHel)) 0;
                  -sin(-(uav.pPos.psiHel)) cos(-(uav.pPos.psiHel)) 0;
                              0                       0            1];


        vertices = H*H1*[Rot_J1*uav.pCAD.obj{3}.v; ones(1,size(uav.pCAD.obj{3}.v,2))];
        uav.pCAD.i3D{3}.Vertices = vertices(1:3,:)';



        %%% Rotational matrix (H�lice Tras Dir.)
        RotX = [1 0 0; 0 cos(uav.pPos.X(4)) -sin(uav.pPos.X(4)); 0 sin(uav.pPos.X(4)) cos(uav.pPos.X(4))];
        RotY = [cos(uav.pPos.X(5)) 0 sin(uav.pPos.X(5)); 0 1 0; -sin(uav.pPos.X(5)) 0 cos(uav.pPos.X(5))];
        RotZ = [cos(uav.pPos.X(6)) -sin(uav.pPos.X(6)) 0; sin(uav.pPos.X(6)) cos(uav.pPos.X(6)) 0; 0 0 1];

        Rot = RotZ*RotY*RotX;
        H1 = [Rot [-0.23450; 0.23625; 0.28800]; 0 0 0 1];

        Rot_J1 = [ cos(-(uav.pPos.psiHel + pi/2)) sin(-(uav.pPos.psiHel + pi/2)) 0;
                  -sin(-(uav.pPos.psiHel + pi/2)) cos(-(uav.pPos.psiHel + pi/2)) 0;
                                0                           0                    1];

        vertices = H*H1*[Rot_J1*uav.pCAD.obj{2}.v; ones(1,size(uav.pCAD.obj{2}.v,2))];
        uav.pCAD.i3D{4}.Vertices = vertices(1:3,:)';


        mCADupdateSkeletonModel(uav,H,H,H,H,H,Rot_J1,Rot_J1,vis)

    
    else
        % - Modelo esqueleto
        
        %%% Rotational matrix
        RotX = [1 0 0; 0 cos(uav.pPos.X(4)) -sin(uav.pPos.X(4)); 0 sin(uav.pPos.X(4)) cos(uav.pPos.X(4))];
        RotY = [cos(uav.pPos.X(5)) 0 sin(uav.pPos.X(5)); 0 1 0; -sin(uav.pPos.X(5)) 0 cos(uav.pPos.X(5))];
        RotZ = [cos(uav.pPos.X(6)) -sin(uav.pPos.X(6)) 0; sin(uav.pPos.X(6)) cos(uav.pPos.X(6)) 0; 0 0 1];

        Rot = RotZ*RotY*RotX;
        H1 = [Rot [uav.pCAD.droneX(1); uav.pCAD.droneY(1); uav.pCAD.droneZ(1)]; 0 0 0 1];

        Rot_J1 = [ cos(-(uav.pPos.psiHel)) sin(-(uav.pPos.psiHel)) 0;
                  -sin(-(uav.pPos.psiHel)) cos(-(uav.pPos.psiHel)) 0;
                                0                           0      1];

        H2 = [Rot [uav.pCAD.droneX(2); uav.pCAD.droneY(2); uav.pCAD.droneZ(2)]; 0 0 0 1];
        H3 = [Rot [uav.pCAD.droneX(3); uav.pCAD.droneY(3); uav.pCAD.droneZ(3)]; 0 0 0 1];
        H4 = [Rot [uav.pCAD.droneX(4); uav.pCAD.droneY(4); uav.pCAD.droneZ(4)]; 0 0 0 1];

        Rot_J2 = [ cos(uav.pPos.psiHel+pi/2) sin(uav.pPos.psiHel+pi/2) 0;
                  -sin(uav.pPos.psiHel+pi/2) cos(uav.pPos.psiHel+pi/2) 0;
                                0                    0                 1];

                        
        mCADupdateSkeletonModel(uav,H,H1,H2,H3,H4,Rot_J1,Rot_J2,vis)

    end
    

%     uav.pPos.psiHel = uav.pPos.psiHel + 2*pi/6;
    uav.pPos.psiHel = uav.pPos.psiHel + pi/6;

end



end

% =========================================================================
function mCADmake(uav)

for i = 1:length(uav.pCAD.obj)
    
    hold on
    uav.pCAD.i3D{i} = patch('Vertices',uav.pCAD.obj{1,i}.v','Faces',uav.pCAD.obj{1,i}.f3');
    hold off
    
    fvcd3 = [];
    
    for ii = 1:length(uav.pCAD.obj{i}.umat3)
        mtlnum = uav.pCAD.obj{i}.umat3(ii);
        for jj=1:length(uav.pCAD.mtl{i})
            if strcmp(uav.pCAD.mtl{i}(jj).name,uav.pCAD.obj{i}.usemtl(mtlnum-1))
                break;
            end
        end
        fvcd3(ii,:) = uav.pCAD.mtl{i}(jj).Kd';
        
    end

    % size(fvcd3)
    
    uav.pCAD.i3D{i}.FaceVertexCData = fvcd3;
    uav.pCAD.i3D{i}.FaceColor = 'flat';
    uav.pCAD.i3D{i}.EdgeColor = 'none';
    uav.pCAD.i3D{i}.FaceAlpha = 1.0;
    uav.pCAD.i3D{i}.Visible = 'off';
    % light;

end

for iP = 4:5
    
    i = iP-2;
    
    hold on
    uav.pCAD.i3D{iP} = patch('Vertices',uav.pCAD.obj{1,i}.v','Faces',uav.pCAD.obj{1,i}.f3');
    hold off
    
    fvcd3 = [];
    
    for ii = 1:length(uav.pCAD.obj{i}.umat3)
        mtlnum = uav.pCAD.obj{i}.umat3(ii);
        for jj=1:length(uav.pCAD.mtl{i})
            if strcmp(uav.pCAD.mtl{i}(jj).name,uav.pCAD.obj{i}.usemtl(mtlnum-1))
                break;
            end
        end
        fvcd3(ii,:) = uav.pCAD.mtl{i}(jj).Kd';
        
    end

    % size(fvcd3)
    
    uav.pCAD.i3D{iP}.FaceVertexCData = fvcd3;
    uav.pCAD.i3D{iP}.FaceColor = 'flat';
    uav.pCAD.i3D{iP}.EdgeColor = 'none';
    uav.pCAD.i3D{iP}.FaceAlpha = 1.0;
    uav.pCAD.i3D{iP}.Visible = 'off';
end
    

if uav.pCAD.flagFrame
    hold on
    uav.pCAD.i3D{length(uav.pCAD.i3D)+1} = plot3([0 .40],[0 0],  [0 0], '-r','LineWidth',2,'Marker','o','MarkerSize',1,'MarkerEdgeColor','r','MarkerFaceColor','k','MarkerIndices',2);
    uav.pCAD.i3D{length(uav.pCAD.i3D)+1} = plot3([0 0],  [0 .30],[0 0], '-g','LineWidth',2,'Marker','o','MarkerSize',1,'MarkerEdgeColor','g','MarkerFaceColor','k','MarkerIndices',2);
    uav.pCAD.i3D{length(uav.pCAD.i3D)+1} = plot3([0 0],  [0 0],  [0 .30],'-b','LineWidth',2,'Marker','o','MarkerSize',1,'MarkerEdgeColor','b','MarkerFaceColor','k','MarkerIndices',2);
    hold off

    uav.pCAD.OriginalAxis{1} = [0 .40;
                                0   0;
                                0   0];
    uav.pCAD.OriginalAxis{2} = [0   0;
                                0 .30;
                                0   0];
    uav.pCAD.OriginalAxis{3} = [0   0;
                                0   0;
                                0 .30];

    uav.pCAD.i3D{6}.Visible = 'off';
    uav.pCAD.i3D{7}.Visible = 'off';
    uav.pCAD.i3D{8}.Visible = 'off';                            
   
end


% --- Modelo em Esqueleto (Transformar tudo em uma fun��o! e chamar aqui):

mCADmakeSkeleton(uav)

% --- --- --- --- --- --- --- --- --- ---

drawnow limitrate nocallbacks



uav.pCAD.flagCreated = 1;


end















% ------- 
function mCADmakeSkeleton(uav)

    droneX = [0.25 0.25 -0.25 -0.25];
    droneY = [-0.25 0.25 -0.25 0.25];
    droneZ = 1.0*[0.2575 0.2575 0.2575 0.2575];
    
    uav.pCAD.droneX = droneX;
    uav.pCAD.droneY = droneY;
    uav.pCAD.droneZ = droneZ;

    hold on
    uav.pCAD.ObjSkeleton{1} = plot3(uav.pPos.X(1),uav.pPos.X(2),uav.pPos.X(3),'.k',...
                                    'MarkerSize',5,...
                                    'MarkerEdgeColor','k');

    uav.pCAD.ObjSkeleton{2} = plot3([droneX(1) droneX(4)],[droneY(1) droneY(4)],[droneZ(1) droneZ(4)],...
                                    '-o','LineWidth',1.5,'MarkerSize',2.5,...
                                    'MarkerEdgeColor','k','Color',[0.7 0.7 0.7],...
                                    'MarkerFaceColor',[0.0,0.0,0.0]);

    uav.pCAD.ObjSkeleton{3} = plot3([droneX(2) droneX(3)],[droneY(2) droneY(3)],[droneZ(2) droneZ(3)],...
                                    '-ok','LineWidth',1.5,'MarkerSize',2.5,...
                                    'MarkerEdgeColor','k','Color',[0.7 0.7 0.7],...
                                    'MarkerFaceColor',[0.0,0.0,0.0]);         
    hold off


    % --- Define joint vertices
    n = 20; % n�mero de faces
    r = 0.015;
    r1 = 0.15;
    r2 = 0.15;

    % Cylinder
    circx = r1*cos(linspace(-pi,pi,n));
    circy = r2*sin(linspace(-pi,pi,n));

    uav.pCAD.vertices{1} = [circx' circy'  r*ones(length(circx),1);
                            circx' circy' -r*ones(length(circx),1)];


    % Define joint faces

    for ii = 1:(n-1)
        uav.pCAD.faces{1}(ii,:) = [ii ii+1 n+ii+1 n+ii];
    end

    for iii = 1:(n-1)
        uav.pCAD.faces{2}(iii,1) = [iii];
        uav.pCAD.faces{3}(iii,1) = [iii+n];
    end


    % --- Posicionamento H�lices:
    uav.pHom.Hfe = [1 0 0  droneX(1);
                    0 1 0  droneY(1);
                    0 0 1  droneZ(1);
                    0 0 0     1     ];


    vert_t = [uav.pCAD.vertices{1}'; ones(1,length(uav.pCAD.vertices{1}))];
    vert_tempFE = uav.pHom.Hfe*vert_t;
    uav.pCAD.vert_tempFE = vert_tempFE;
    % --- --- --- --- 

    uav.pHom.Hfd = [1 0 0  droneX(2);
                    0 1 0  droneY(2);
                    0 0 1  droneZ(2);
                    0 0 0     1     ];

    vert_t = [uav.pCAD.vertices{1}'; ones(1,length(uav.pCAD.vertices{1}))];
    vert_tempFD = uav.pHom.Hfd*vert_t;
    uav.pCAD.vert_tempFD = vert_tempFD;
    % --- --- --- ---         

    uav.pHom.Htd = [1 0 0  droneX(4);
                    0 1 0  droneY(4);
                    0 0 1  droneZ(4);
                    0 0 0     1     ];

    vert_t = [uav.pCAD.vertices{1}'; ones(1,length(uav.pCAD.vertices{1}))];
    vert_tempTD = uav.pHom.Htd*vert_t;        
    uav.pCAD.vert_tempTD = vert_tempTD;
    % --- --- --- --- 

    uav.pHom.Hte = [1 0 0  droneX(3);
                    0 1 0  droneY(3);
                    0 0 1  droneZ(3);
                    0 0 0     1     ];

    vert_t = [uav.pCAD.vertices{1}'; ones(1,length(uav.pCAD.vertices{1}))];
    vert_tempTE = uav.pHom.Hte*vert_t;
    uav.pCAD.vert_tempTE = vert_tempTE;
    
    % - Plot
    hold on
    uav.pCAD.ObjSkeleton{4} = patch('Vertices',vert_tempFE(1:3,:)','Faces',uav.pCAD.faces{1},'EdgeAlpha',0.0,'FaceAlpha',0.5,'FaceColor',[1.0 0.5 0.5]); % Trocar cor no mCADcolor!!!
    uav.pCAD.ObjSkeleton{5} = patch('Vertices',vert_tempFD(1:3,:)','Faces',uav.pCAD.faces{1},'EdgeAlpha',0.0,'FaceAlpha',0.5,'FaceColor',[1.0 0.5 0.5]); % Trocar cor no mCADcolor!!!
    uav.pCAD.ObjSkeleton{6} = patch('Vertices',vert_tempTD(1:3,:)','Faces',uav.pCAD.faces{1},'EdgeAlpha',0.0,'FaceAlpha',0.5,'FaceColor',[0.5 0.5 0.5]);
    uav.pCAD.ObjSkeleton{7} = patch('Vertices',vert_tempTE(1:3,:)','Faces',uav.pCAD.faces{1},'EdgeAlpha',0.0,'FaceAlpha',0.5,'FaceColor',[0.5 0.5 0.5]);


    % -- Covers:
    uav.pCAD.ObjSkeleton{8}  = patch('Vertices',vert_tempFE(1:3,:)','Faces',[uav.pCAD.faces{2}'; uav.pCAD.faces{3}'],'EdgeAlpha',0.75,'FaceAlpha',0.1,'FaceColor',[1 0 0]*0.2); % Trocar cor no mCADcolor!!!
    uav.pCAD.ObjSkeleton{9}  = patch('Vertices',vert_tempFD(1:3,:)','Faces',[uav.pCAD.faces{2}'; uav.pCAD.faces{3}'],'EdgeAlpha',0.75,'FaceAlpha',0.1,'FaceColor',[1 0 0]*0.2); % Trocar cor no mCADcolor!!!
    uav.pCAD.ObjSkeleton{10} = patch('Vertices',vert_tempTD(1:3,:)','Faces',[uav.pCAD.faces{2}'; uav.pCAD.faces{3}'],'EdgeAlpha',0.75,'FaceAlpha',0.1,'FaceColor',[0 0 0]*0.2);
    uav.pCAD.ObjSkeleton{11} = patch('Vertices',vert_tempTE(1:3,:)','Faces',[uav.pCAD.faces{2}'; uav.pCAD.faces{3}'],'EdgeAlpha',0.75,'FaceAlpha',0.1,'FaceColor',[0 0 0]*0.2);
    hold off

    % --- UAV Body 
                            %  x     y     z
    uav.pCAD.vertices{2} = [ 0.00 -0.05  0.00;
                             0.10 -0.10  0.00;
                             0.05  0.00  0.00;
                             0.10  0.10  0.00;
                             0.00  0.05  0.00;
                            -0.10  0.10  0.00;
                            -0.05  0.00  0.00;
                            -0.10  -0.10 0.00]; % + [zeros(8,2) .2575*ones(8,1)];


    rotUAV = [1 0 0;
              0 1 0;
              0 0 1];   

    uav.pHom.Hwd = [rotUAV [uav.pPos.X(1);
                            uav.pPos.X(2);
                            uav.pPos.X(3)];
                    0 0 0          1     ];

    vert_t = [uav.pCAD.vertices{2}'; ones(1,length(uav.pCAD.vertices{2}))];
    vert_tempBody = uav.pHom.Hwd*vert_t; 
    uav.pCAD.vert_tempBody = vert_tempBody;

    hold on
    uav.pCAD.ObjSkeleton{12} = patch('XData',vert_tempBody(1,:)',...
                                     'YData',vert_tempBody(2,:)',...
                                     'ZData',vert_tempBody(3,:),...
                                     'EdgeAlpha',0.75,'FaceAlpha',0.5,'FaceColor',[0.7 0.7 0.7]);

    % Trocar cor no mCADcolor!!!
    uav.pCAD.ObjSkeleton{13} = patch('XData',[vert_tempBody(1,2:4)'; 0.25],...
                                     'YData',[vert_tempBody(2,2:4)'; 0.00],...
                                     'ZData',[vert_tempBody(3,2:5)'],...
                                     'EdgeAlpha',0.75,'FaceAlpha',0.75,'FaceColor',[0.35 0.35 0.70]);


    uav.pCAD.ObjSkeleton{14} = plot3([uav.pPos.X(1) 0.25],[uav.pPos.X(2) 0.00],[uav.pPos.X(3) uav.pPos.X(3)],...
                                     '-k','LineWidth',1);

    hold off


    % --- P�s Rotativas
          %  x    y
    pas = [ 0.1  0.1;
           -0.1 -0.1]*0.9;

    pasTemp = [pas'; 0.0 0.0];   

    uav.pCAD.pasTemp = pasTemp;
    
    % -- Posicionamento H�lices:
%     uav.pHom.Hfe = [1 0 0  droneX(1);
%                     0 1 0  droneY(1);
%                     0 0 1  droneZ(1);
%                     0 0 0     1     ];

    uav.pHom.Hfe = [1 0 0  0;
                    0 1 0  0;
                    0 0 1  0;
                    0 0 0  1];

    vert_t = [pasTemp; ones(1,size(pasTemp,2))];
    vert_tempFEpas = uav.pHom.Hfe*vert_t;
    uav.pCAD.vert_tempFEpas = vert_tempFEpas;
    % --- --- --- --- 

%     uav.pHom.Hfd = [0 1 0  droneX(2);
%                    -1 0 0  droneY(2);
%                     0 0 1  droneZ(2);
%                     0 0 0     1     ];

    uav.pHom.Hfd = [1 0 0  0;
                    0 1 0  0;
                    0 0 1  0;
                    0 0 0  1];

    vert_tempFDpas = uav.pHom.Hfd*vert_t;
    uav.pCAD.vert_tempFDpas = vert_tempFDpas;
    % --- --- --- ---         

%     uav.pHom.Htd = [1 0 0  droneX(4);
%                     0 1 0  droneY(4);
%                     0 0 1  droneZ(4);
%                     0 0 0     1     ];

    uav.pHom.Htd = [1 0 0  0;
                    0 1 0  0;
                    0 0 1  0;
                    0 0 0  1];

    vert_tempTDpas = uav.pHom.Htd*vert_t;        
    uav.pCAD.vert_tempTDpas = vert_tempTDpas;
    % --- --- --- --- 

%     uav.pHom.Hte = [0 1 0  droneX(3);
%                    -1 0 0  droneY(3);
%                     0 0 1  droneZ(3);
%                     0 0 0     1     ];

    uav.pHom.Hte = [1 0 0  0;
                    0 1 0  0;
                    0 0 1  0;
                    0 0 0  1];

    vert_tempTEpas = uav.pHom.Hte*vert_t;
    uav.pCAD.vert_tempTEpas = vert_tempTEpas;
    % --- --- --- ---    


    hold on
    uav.pCAD.ObjSkeleton{15} = plot3([vert_tempFEpas(1,1) vert_tempFEpas(1,2)],...
                                     [vert_tempFEpas(2,1) vert_tempFEpas(2,2)],...
                                     [vert_tempFEpas(3,1) vert_tempFEpas(3,2)],...
                                     '-k','LineWidth',1.5);

    uav.pCAD.ObjSkeleton{16} = plot3([vert_tempFDpas(1,1) vert_tempFDpas(1,2)],...
                                     [vert_tempFDpas(2,1) vert_tempFDpas(2,2)],...
                                     [vert_tempFDpas(3,1) vert_tempFDpas(3,2)],...
                                     '-k','LineWidth',1.5);

    uav.pCAD.ObjSkeleton{17} = plot3([vert_tempTDpas(1,1) vert_tempTDpas(1,2)],...
                                     [vert_tempTDpas(2,1) vert_tempTDpas(2,2)],...
                                     [vert_tempTDpas(3,1) vert_tempTDpas(3,2)],...
                                     '-k','LineWidth',1.5);

    uav.pCAD.ObjSkeleton{18} = plot3([vert_tempTEpas(1,1) vert_tempTEpas(1,2)],...
                                     [vert_tempTEpas(2,1) vert_tempTEpas(2,2)],...
                                     [vert_tempTEpas(3,1) vert_tempTEpas(3,2)],...
                                     '-k','LineWidth',1.5);



    hold off

    
    for idx = 1:length(uav.pCAD.ObjSkeleton)
        uav.pCAD.ObjSkeleton{idx}.Visible = 'off';
    end

end



% Updating the Virtual Environment
function mCADupdateSkeletonModel(uav,H,H1,H2,H3,H4,Rot_J1,Rot_J2,vis)
    
    if vis
        if ~strcmp(uav.pCAD.ObjSkeleton, 'on')
            for idx = 1:length(uav.pCAD.ObjSkeleton)
                uav.pCAD.ObjSkeleton{idx}.Visible = 'on';
            end                
            for jj = 1:length(uav.pCAD.i3D)-3
                uav.pCAD.i3D{jj}.Visible = 'off';
            end
        end
    

    
        % Update robot pose
        uav.pCAD.ObjSkeleton{1}.XData = uav.pPos.X(1);
        uav.pCAD.ObjSkeleton{1}.YData = uav.pPos.X(2);
        uav.pCAD.ObjSkeleton{1}.ZData = uav.pPos.X(3);


        vertices = H*[[uav.pCAD.droneX; uav.pCAD.droneY; uav.pCAD.droneZ]; ones(1,size(uav.pCAD.droneX,2))];

        uav.pCAD.ObjSkeleton{2}.XData = [vertices(1,1) vertices(1,4)];
        uav.pCAD.ObjSkeleton{2}.YData = [vertices(2,1) vertices(2,4)];
        uav.pCAD.ObjSkeleton{2}.ZData = [vertices(3,1) vertices(3,4)];

        uav.pCAD.ObjSkeleton{3}.XData = [vertices(1,2) vertices(1,3)];
        uav.pCAD.ObjSkeleton{3}.YData = [vertices(2,2) vertices(2,3)];
        uav.pCAD.ObjSkeleton{3}.ZData = [vertices(3,2) vertices(3,3)];
            

        vertices = H*[[0.25; 0.00; 0.2575]; 1.00];

        uav.pCAD.ObjSkeleton{14}.XData = [uav.pPos.X(1) vertices(1)];
        uav.pCAD.ObjSkeleton{14}.YData = [uav.pPos.X(2) vertices(2)];
        uav.pCAD.ObjSkeleton{14}.ZData = [uav.pPos.X(3) vertices(3)];
            
            
        % --- Posicionamento H�lices:
        
        vert_t = [Rot_J1*uav.pCAD.pasTemp; ones(1,size(uav.pCAD.pasTemp,2))];
        uav.pHom.Hfe = H1;
        vert_tempFEpas = uav.pHom.Hfe*vert_t;

        uav.pCAD.vert_tempFEpas = vert_tempFEpas;

        vert = H*uav.pCAD.vert_tempFEpas;
    
    
        uav.pCAD.ObjSkeleton{15}.XData = [vert(1,1) vert(1,2)];
        uav.pCAD.ObjSkeleton{15}.YData = [vert(2,1) vert(2,2)];
        uav.pCAD.ObjSkeleton{15}.ZData = [vert(3,1) vert(3,2)];

        % --- --- --- --- 
        vert_t = [Rot_J2*uav.pCAD.pasTemp; ones(1,size(uav.pCAD.pasTemp,2))];
        uav.pHom.Hfd = H2;
        vert_tempFDpas = uav.pHom.Hfd*vert_t;

        uav.pCAD.vert_tempFDpas = vert_tempFDpas;
        
        vert = H*uav.pCAD.vert_tempFDpas;

        uav.pCAD.ObjSkeleton{16}.XData = [vert(1,1) vert(1,2)];
        uav.pCAD.ObjSkeleton{16}.YData = [vert(2,1) vert(2,2)];
        uav.pCAD.ObjSkeleton{16}.ZData = [vert(3,1) vert(3,2)];

        % --- --- --- ---      
        vert_t = [Rot_J1*uav.pCAD.pasTemp; ones(1,size(uav.pCAD.pasTemp,2))];
        uav.pHom.Htd = H4;
        vert_tempTDpas = uav.pHom.Htd*vert_t;

        uav.pCAD.vert_tempTDpas = vert_tempTDpas;
        
        vert = H*uav.pCAD.vert_tempTDpas;  

        uav.pCAD.ObjSkeleton{17}.XData = [vert(1,1) vert(1,2)];
        uav.pCAD.ObjSkeleton{17}.YData = [vert(2,1) vert(2,2)];
        uav.pCAD.ObjSkeleton{17}.ZData = [vert(3,1) vert(3,2)];

        % --- --- --- --- 
        vert_t = [Rot_J2*uav.pCAD.pasTemp; ones(1,size(uav.pCAD.pasTemp,2))];
        uav.pHom.Hte = H3;
        vert_tempTEpas = uav.pHom.Hte*vert_t;

        uav.pCAD.vert_tempTEpas = vert_tempTEpas;
                
        vert = H*uav.pCAD.vert_tempTEpas;

        uav.pCAD.ObjSkeleton{18}.XData = [vert(1,1) vert(1,2)];
        uav.pCAD.ObjSkeleton{18}.YData = [vert(2,1) vert(2,2)];
        uav.pCAD.ObjSkeleton{18}.ZData = [vert(3,1) vert(3,2)];


        % - Atualizando patchs
        vert = H*uav.pCAD.vert_tempFE;

        set(uav.pCAD.ObjSkeleton{4}, 'Vertices',vert(1:3,:)');
        set(uav.pCAD.ObjSkeleton{8}, 'Vertices',vert(1:3,:)');

        vert = H*uav.pCAD.vert_tempFD;

        set(uav.pCAD.ObjSkeleton{5}, 'Vertices',vert(1:3,:)');
        set(uav.pCAD.ObjSkeleton{9}, 'Vertices',vert(1:3,:)');

        vert = H*uav.pCAD.vert_tempTD;

        set(uav.pCAD.ObjSkeleton{6}, 'Vertices',vert(1:3,:)');
        set(uav.pCAD.ObjSkeleton{10}, 'Vertices',vert(1:3,:)');

        vert = H*uav.pCAD.vert_tempTE;

        set(uav.pCAD.ObjSkeleton{7}, 'Vertices',vert(1:3,:)');
        set(uav.pCAD.ObjSkeleton{11}, 'Vertices',vert(1:3,:)');

        vert = H*uav.pCAD.vert_tempBody;
        vertTemp = H*[[0.25; 0.00; 0.2575]; 1.00];


        set(uav.pCAD.ObjSkeleton{12}, 'XData', vert(1,:)');
        set(uav.pCAD.ObjSkeleton{12}, 'YData', vert(2,:)');
        set(uav.pCAD.ObjSkeleton{12}, 'ZData', vertTemp(3)*ones(8,1));

        set(uav.pCAD.ObjSkeleton{13}, 'XData', [vert(1,2:4)'; vertTemp(1)]);
        set(uav.pCAD.ObjSkeleton{13}, 'YData', [vert(2,2:4)'; vertTemp(2)]);
        set(uav.pCAD.ObjSkeleton{13}, 'ZData', vertTemp(3)*ones(4,1));

            
    drawnow limitrate
    
    else
        if ~strcmp(uav.pCAD.i3D{1}.Visible, 'on')
            for idx = 1:length(uav.pCAD.ObjSkeleton)
                uav.pCAD.ObjSkeleton{idx}.Visible = 'off';
            end

            for jj = 1:length(uav.pCAD.i3D)-3
                uav.pCAD.i3D{jj}.Visible = 'on';
            end
        end
    end
    
end