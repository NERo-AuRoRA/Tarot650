% Testar modelo gr�fico em tempo real

close all
clear
clc

try
    fclose(instrfindall);
end

% - Rotina para buscar pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))


% - Representa��o do rob�
A = Tarot650;

% Alterando posi��o
A.pPos.X(3) = 0.75;


tmax = 60; % Tempo Simula��o em segundos
X = zeros(1,19); % Dados correntes da simula��o

limites = 1.5*[-0.5 1 -0.5 1 0 1];
figure(1)
% Ground = patch(limites([1 1 2 2]),limites([3 4 4 3]),limites([5 5 5 5]),[0.6 1 0.6]);
% Ground.FaceAlpha = 0.4;
% view(40,40)
view(45,30)

axis equal
axis(limites)
grid on

% Estilizando superficie
lighting phong;
material shiny;
colormap winter;
lightangle(-45,30)


Info = title(['Time: ' num2str(0,'%05.2f') ' | ' num2str(tmax,'%05.2f') 's']);  

A.mCADplot(1);

% Alterando cor do rob�
A.mCADcolor([230 230 230]/255);

drawnow
pause(5)
disp('Start............')

% =========================================================================
t = tic;
tc = tic;
tp = tic;

XX = [];
TT = [];

% - como ainda n�o temos modelo din�mico ou controlador...
stepTime = 1/30;
n = numel(0:stepTime:tmax);

x = linspace(A.pPos.X(1),0.5,n/2);
y = linspace(A.pPos.X(2),0.75,n/2); 
z = linspace(A.pPos.X(3),1.25,n/2); 
phi = linspace(A.pPos.X(4),0,n/2);
theta = linspace(A.pPos.X(5),0,n/2); 
psi = linspace(A.pPos.X(6),-pi/2,n/2);

idx = 1;


while toc(t) < tmax
    
    if toc(tc) > 1/30/4
        
        TT = [TT toc(tc)];
        
        
        tc = tic;
        
        if idx < length(x)        
            A.pPos.X(1:6) = [x(idx) y(idx) z(idx) phi(idx) theta(idx) psi(idx)];
        end
        
        idx = idx + 1;
    
    
    if toc(tp) > 0.1
                
        if toc(t) > 3*tmax/4
            A.mCADplot(1,1);
        elseif toc(t) > 2*tmax/4
            A.mCADplot(0,1);
        elseif toc(t) > 1*tmax/4
            A.mCADplot(1,0);
        else
            A.mCADplot();
        end
        
        Info.String = ['Time: ' num2str(toc(t),'%05.2f') ' | ' num2str(tmax,'%05.2f') 's'];
        
        pause(5e-5)
        
        
    end
    end
    
end

% figure
% subplot(211),plot(XX(end,:),XX([4 16],:)'*180/pi)
% legend('\phi_{Des}','\phi_{Atu}')
% grid
% subplot(212),plot(XX(end,:),XX([5 17],:)'*180/pi)
% legend('\theta_{Des}','\theta_{Atu}')
% grid
% 
% figure
% plot(XX([1,13],:)',XX([2,14],:)')
% axis equal
% 
% figure
% subplot(211),plot(XX(end,:),XX([1 13],:)')
% legend('x_{Des}','x_{Atu}')
% grid
% subplot(212),plot(XX(end,:),XX([2 14],:)')
% legend('y_{Des}','y_{Atu}')
% grid

% figure
% subplot(211),plot(XX(end,:),XX(19,:)')
% legend('x_{Des}','x_{Atu}')
% grid
% subplot(212),plot(XX(end,:),[XX(13,:); [0 diff(XX(13,:))]*30])
% legend('y_{Des}','y_{Atu}')
% grid

