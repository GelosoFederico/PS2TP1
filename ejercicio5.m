%% Ejercicio 5
% a) Con los valores del ej 2 con x0 = 40 -1000 0 0 0 0, ver la matriz del
% estado estacionario
% Esa la saco con la ecuación de Ricatti creo, y la comparo con la última
% de lo otro

clear all;close all;clc;

load('datos.mat');

var_ruido_proc_pos=3e-4;
var_ruido_proc_vel=2e-3;
var_ruido_proc_acel=1e-2;

A_d = [ 1 0 1 0 0.5 0;
        0 1 0 1 0 0.5;
        0 0 1 0 1 0;
        0 0 0 1 0 1;
        0 0 0 0 1 0;
        0 0 0 0 0 1  ];
Q_d = diag([var_ruido_proc_pos ...
            var_ruido_proc_pos, ...
            var_ruido_proc_vel ...
            var_ruido_proc_vel, ...
            var_ruido_proc_acel ...
            var_ruido_proc_acel]);
        
x0 = [40 -1000 0 0 0 0]';
P0_0 = diag([10^6 10^6, 100 100, 10 10]);


% a) medicion posicion
C = [1 0 0 0 0 0;
     0 1 0 0 0 0];
B = eye(6);

sigma_pos= 100; %Ruido de medicion para coordenadas x e y
R= diag([sigma_pos*sigma_pos sigma_pos*sigma_pos]);

% Armo las mediciones con el ruido
yk(:,1)=Pos(:,1)+sigma_pos*randn(length(Pos(:,1)),1);
yk(:,2)=Pos(:,2)+sigma_pos*randn(length(Pos(:,2)),1);
N=length(Pos);

p00=P0_0;
D=0;
[x,ek,P,K] = KalmanFilter(A_d,B,C,D,Q_d,R,x0,p00,yk);

[P_est,vals, K_est] = dare(A_d',C',B*Q_d*B',R);
x0
[x2,ek2,P2,K2] = KalmanFilterSteadyState(A_d,B,C,D,Q_d,R,x0,p00,yk);
x = x';
x2 = x2';

%Grafico la trayectoria
h1=figure;
hold on
plot(yk(:,1),yk(:,2),'LineWidth',1.2,'Color',[0 161 0]/255);
plot(Pos(:,1),Pos(:,2),'b','LineWidth',1.6);
plot(x(1,:),x(2,:),'r','LineWidth',1.6);
plot(x2(1,:),x2(2,:),'c','LineWidth',1.6);
axis([-7000 1000 -3000 1000])
grid on
ylabel('Y [m]')
xlabel('X [m]')
title(['Trayectoria estimada'])
legend('Pos sin FK','Pos Real',  'Pos con FK', 'Pos con FK est', 'location', 'northwest')
print(h1,'trayectoria','-dpng');
hold off

% Posicion-x vs tiempo
h2=figure;
subplot(2,1,1)
hold on
plot(Pos(:,3),Pos(:,1),'k','LineWidth',1.6);
plot(Pos(:,3),x(1,:),'r.','LineWidth',1.6);
plot(Pos(:,3),x2(1,:),'c.','LineWidth',1.6);
grid on
ylabel('Pos-X [m]')
xlabel('Tiempo [muestras]')
legend('Real', 'FK', 'FK est')
hold off

% Posicion-y vs tiempo
subplot(2,1,2)
hold on
plot(Pos(:,3),Pos(:,2),'k','LineWidth',1.6);
plot(Pos(:,3),x(2,:),'r.','LineWidth',1.6);
plot(Pos(:,3),x2(2,:),'c.','LineWidth',1.6);
grid on
ylabel('Pos-Y [m]')
xlabel('Tiempo [muestras]')
legend('Real', 'FK', 'FK est')
print(h2,'p_vs_t','-dpng','-r0');
hold off

% Velocidad-x vs tiempo
h3=figure;
subplot(2,1,1)
hold on
plot(Pos(:,3),Vel(:,1),'k','LineWidth',1.6);
plot(Pos(:,3),x(3,:),'r.','LineWidth',1.6);
plot(Pos(:,3),x2(3,:),'c.','LineWidth',1.6);
grid on
ylabel('Vel-X [m]')
xlabel('Tiempo [muestras]')
legend('Real', 'FK', 'FK est')
hold off

% Velocidad-y vs tiempo
subplot(2,1,2)
hold on
plot(Pos(:,3),Vel(:,2),'k','LineWidth',1.6);
plot(Pos(:,3),x(4,:),'r.','LineWidth',1.6);
plot(Pos(:,3),x2(4,:),'c.','LineWidth',1.6);
grid on
ylabel('Vel-Y [m]')
xlabel('Tiempo [muestras]')
legend('Real', 'FK', 'FK est')
print(h3,'v_vs_t','-dpng','-r0');
hold off

% Aceleracion-x vs tiempo
h4=figure;
subplot(2,1,1)
hold on
plot(Pos(:,3),Acel(:,1),'k','LineWidth',1.6);
plot(Pos(:,3),x(5,:),'r.','LineWidth',1.6);
plot(Pos(:,3),x2(5,:),'c.','LineWidth',1.6);
grid on
ylabel('Acel-X [m]')
xlabel('Tiempo [muestras]')
legend('Real', 'FK', 'FK est','location','SE')
hold off

% Aceleracion-y vs tiempo
subplot(2,1,2)
hold on
plot(Pos(:,3),Acel(:,2),'k','LineWidth',1.6);
plot(Pos(:,3),x(6,:),'r.','LineWidth',1.6);
plot(Pos(:,3),x2(6,:),'c.','LineWidth',1.6);
grid on
ylabel('Acel-Y [m]')
xlabel('Tiempo [muestras]')
legend('Real', 'FK', 'FK est')
print(h4,'a_vs_t','-dpng','-r0');
hold off

% Innovaciones
h5=figure;
hold on
plot(xcorr(ek(1,:)),'k','LineWidth',1.2);
plot(xcorr(ek(2,:)),'r','LineWidth',1.2);
plot(xcorr(ek2(1,:)),'m','LineWidth',1.2);
plot(xcorr(ek2(2,:)),'c','LineWidth',1.2);
grid on
ylabel('Autocorrelacion')
xlabel('Tau')
legend('e1', 'e2', 'e1 est', 'e2 est')
print(h5,'innovaciones','-dpng','-r0');
hold off
