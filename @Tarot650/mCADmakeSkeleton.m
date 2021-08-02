function mCADmakeSkeleton(uav)

    droneX = [0.25 0.25 -0.25 -0.25];
    droneY = [-0.25 0.25 -0.25 0.25];
    droneZ = uav.pPos.X(3)*[1.0 1.0 1.0 1.0];

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
    n = 20; % número de faces
    r = 0.025;
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


    % --- Posicionamento Hélices:
    uav.pHom.Hfe = [1 0 0  droneX(1);
                    0 1 0  droneY(1);
                    0 0 1  droneZ(1);
                    0 0 0     1     ];


    vert_t = [uav.pCAD.vertices{1}'; ones(1,length(uav.pCAD.vertices{1}))];
    vert_tempFE = uav.pHom.Hfe*vert_t;

    % --- --- --- --- 

    uav.pHom.Hfd = [1 0 0  droneX(2);
                    0 1 0  droneY(2);
                    0 0 1  droneZ(2);
                    0 0 0     1     ];

    vert_t = [uav.pCAD.vertices{1}'; ones(1,length(uav.pCAD.vertices{1}))];
    vert_tempFD = uav.pHom.Hfd*vert_t;

    % --- --- --- ---         

    uav.pHom.Htd = [1 0 0  droneX(4);
                    0 1 0  droneY(4);
                    0 0 1  droneZ(4);
                    0 0 0     1     ];

    vert_t = [uav.pCAD.vertices{1}'; ones(1,length(uav.pCAD.vertices{1}))];
    vert_tempTD = uav.pHom.Htd*vert_t;        

    % --- --- --- --- 

    uav.pHom.Hte = [1 0 0  droneX(3);
                    0 1 0  droneY(3);
                    0 0 1  droneZ(3);
                    0 0 0     1     ];

    vert_t = [uav.pCAD.vertices{1}'; ones(1,length(uav.pCAD.vertices{1}))];
    vert_tempTE = uav.pHom.Hte*vert_t;

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
                            -0.10  -0.10 0.00];


    rotUAV = [1 0 0;
              0 1 0;
              0 0 1];   

    uav.pHom.Hwd = [rotUAV [uav.pPos.X(1);
                            uav.pPos.X(2);
                            uav.pPos.X(3)];
                    0 0 0          1     ];

    vert_t = [uav.pCAD.vertices{2}'; ones(1,length(uav.pCAD.vertices{2}))];
    vert_tempBody = uav.pHom.Hwd*vert_t; 


    hold on
    uav.pCAD.ObjSkeleton{12} = patch('XData',vert_tempBody(1,:)',...
                                     'YData',vert_tempBody(2,:)',...
                                     'ZData',vert_tempBody(3,:),...
                                     'EdgeAlpha',0.75,'FaceAlpha',0.5,'FaceColor',[0.7 0.7 0.7]);

    % Trocar cor no mCADcolor!!!
    uav.pCAD.ObjSkeleton{13} = patch('XData',[vert_tempBody(1,2:4)'; 0.25],...
                                     'YData',[vert_tempBody(2,2:4)'; 0.00],...
                                     'ZData',vert_tempBody(3,2:5),...
                                     'EdgeAlpha',0.75,'FaceAlpha',0.75,'FaceColor',[0.35 0.35 0.70]);


    uav.pCAD.ObjSkeleton{14} = plot3([uav.pPos.X(1) 0.25],[uav.pPos.X(2) 0.00],[uav.pPos.X(3) uav.pPos.X(3)],...
                                     '-k','LineWidth',1);

    hold off


    % --- Pás Rotativas
          %  x    y
    pas = [ 0.1  0.1;
           -0.1 -0.1]*0.9;

    pasTemp = [pas'; 0.0 0.0];   

    % -- Posicionamento Hélices:
    uav.pHom.Hfe = [1 0 0  droneX(1);
                    0 1 0  droneY(1);
                    0 0 1  droneZ(1);
                    0 0 0     1     ];

    vert_t = [pasTemp; ones(1,size(pasTemp,2))];
    vert_tempFEpas = uav.pHom.Hfe*vert_t;

    % --- --- --- --- 

    uav.pHom.Hfd = [0 1 0  droneX(2);
                   -1 0 0  droneY(2);
                    0 0 1  droneZ(2);
                    0 0 0     1     ];

    vert_tempFDpas = uav.pHom.Hfd*vert_t;

    % --- --- --- ---         

    uav.pHom.Htd = [1 0 0  droneX(4);
                    0 1 0  droneY(4);
                    0 0 1  droneZ(4);
                    0 0 0     1     ];

    vert_tempTDpas = uav.pHom.Htd*vert_t;        

    % --- --- --- --- 

    uav.pHom.Hte = [0 1 0  droneX(3);
                   -1 0 0  droneY(3);
                    0 0 1  droneZ(3);
                    0 0 0     1     ];

    vert_tempTEpas = uav.pHom.Hte*vert_t;

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


end