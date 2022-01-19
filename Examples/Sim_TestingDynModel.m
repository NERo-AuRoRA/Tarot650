% Validating the Dynamic Model and considering the mass adaptation

close all
clear
clc

try
    fclose(instrfindall);
end

% Looking for the root path
PastaAtual = pwd;
PastaRaiz = 'AuRoRA';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))


% Loading the robot
A = Tarot650;


[~,G,C] = cTarot650_UnderActuated_NearHovering(A);

% Setting a "wrong" mass for the controller
C.m = 3;

% t = tic;
% tc = tic;

tmax = 60;

XXX = [];
ii = 1;

errInt = 0;

% Running in real time
% while toc(t) < tmax
%     if toc(tc) > A.pPar.ts
%         tc = tic;

% Running in clock time
for t = 0:A.pPar.ts:tmax
       
        % ---------------------------------------------------------
        % Trajectory tracking 
        A.pPos.Xd(1) = 0.5*sin(2*pi*t/tmax*4);
        A.pPos.Xd(2) = 0.5*sin(2*pi*t/tmax*4);
        A.pPos.Xd(3) = 2 + 0.5*sin(2*pi*t/tmax*4);
        
        A.pPos.Xd(7) = (2*pi/tmax*4)*0.5*cos(2*pi*t/tmax*4);
        A.pPos.Xd(8) = (2*pi/tmax*4)*0.5*cos(2*pi*t/tmax*4);
        A.pPos.Xd(9) = (2*pi/tmax*4)*0.5*cos(2*pi*t/tmax*4);
        % ---------------------------------------------------------
        
        % ---------------------------------------------------------
        % Position/Regulation 
        %         if t  < 30 % toc(t)
        %             A.pPos.Xd(1) = 2;
        %             A.pPos.Xd(2) = 2;
        %             A.pPos.Xd(3) = 2;
        %             A.pPos.Xd(6) = pi/2;
        %         else
        %             A.pPos.Xd(1) = 0;
        %             A.pPos.Xd(2) = 0;
        %             A.pPos.Xd(3) = 1;
        %             A.pPos.Xd(6) = 0;
        %         end
        %         A.pPos.Xtil = A.pPos.Xd - A.pPos.X;
        % ---------------------------------------------------------
          
        [A,~,C] = cTarot650_UnderActuated_NearHovering(A,G,C);
    
        % ---------------------------------------------------------
        % Adaptive mass controller
        % Here only the mass comming from the controller is  changing
        % 
        % The adaptive process starts after 20s
        if t > 20
            dm = 1*A.pSC.Td(1)/C.g*(A.pPos.Xtil(9)+0.5*A.pPos.Xtil(3));
            C.m = C.m + dm*C.ts;
        end
        
        % After 40s, the drone changes its mass, it is simulating a load
        % carrying task, where the whole mass increases
        % In such a case, the C.m needs to adapt to the new value
        if t > 40
            A.pPar.m = 4;
        end
        
        A.sDynamicModel
        
        % Storaging the data
        XXX(ii+1,:) = [A.pPos.Xd' A.pPos.X' A.pSC.Ud' C.m t];
        ii = ii + 1;
        
        
%     end
end

% Ploting the position and heading
subplot(411)
plot(XXX(:,end),XXX(:,[1 13]))
subplot(412)
plot(XXX(:,end),XXX(:,[2 14]))
subplot(413)
plot(XXX(:,end),XXX(:,[3 15]))
subplot(414)
plot(XXX(:,end),XXX(:,[6 18]))

% Ploting the mass adaptation
figure
plot(XXX(:,end),XXX(:,29))
axis([0 tmax 2.5 4.5])
