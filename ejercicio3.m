% Ejercicio KALMAN
% Richter, Patricio
clear all; close all; clc;

load('datos.mat');

%DATOS
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

P0_0 = diag([10^6 10^6, 100 100, 10 10]);

%Condiciones iniciales:
x0 = [40 -200 0 0 0 0]';
P0_0 = 100*P0_0;

% x0 = [200 -3000 0 0 0 0]';
% P0_0 = 100*P0_0;

% x0 = [40 -200 0 0 0 0]';
% P0_0 = 0.01*P0_0;

% x0 = [200 -3000 0 0 0 0]';
% P0_0 = 0.01*P0_0;


% medicion posicion
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
[x,ek] = KalmanFilter(A_d,B,C,D,Q_d,R,x0,p00,yk);
x=x';


%Grafico la trayectoria
h1=figure;
hold on
plot(yk(:,1),yk(:,2),'LineWidth',1.2,'Color',[0 161 0]/255);
plot(Pos(:,1),Pos(:,2),'b','LineWidth',1.6);
plot(x(1,:),x(2,:),'r','LineWidth',1.6);
axis([-7000 1000 -3000 1000])
grid on
ylabel('Y [m]')
xlabel('X [m]')
title(['Trayectoria estimada'])
legend('Pos sin FK','Pos Real',  'Pos con FK')
hold off

% Error posicion-x vs tiempo
h2=figure;
subplot(2,1,1)
hold on
plot(Pos(:,3),Pos(:,1)-x(1,:)','k','LineWidth',1.6);
grid on
ylabel('error pos-X [m]')
xlabel('Tiempo [muestras]')
%legend('Real', 'FK')
hold off

% Error posicion-y vs tiempo
subplot(2,1,2)
hold on
plot(Pos(:,3),Pos(:,2)-x(2,:)','k','LineWidth',1.6);
grid on
ylabel('error pos-Y [m]')
xlabel('Tiempo [muestras]')
%legend('Error posicion y')
%print(h2,'p_vs_t','-dpng','-r0');
hold off

% Velocidad-x vs tiempo
h3=figure;
subplot(2,1,1)
hold on
plot(Pos(:,3),Vel(:,1)-x(3,:)','k','LineWidth',1.6);
grid on
ylabel('error vel-X [m]')
xlabel('Tiempo [muestras]')
%legend('Real', 'FK')
hold off

% Velocidad-y vs tiempo
subplot(2,1,2)
hold on
plot(Pos(:,3),Vel(:,2)-x(4,:)','k','LineWidth',1.6);
grid on
ylabel('error vel-Y [m]')
xlabel('Tiempo [muestras]')
%legend('Real', 'FK')
%print(h3,'v_vs_t','-dpng','-r0');
hold off

% Aceleracion-x vs tiempo
h4=figure;
subplot(2,1,1)
hold on
plot(Pos(:,3),Acel(:,1)-x(5,:)','k','LineWidth',1.6);
grid on
ylabel('error acel-X [m]')
xlabel('Tiempo [muestras]')
%legend('Real', 'FK')
hold off

% Aceleracion-y vs tiempo
subplot(2,1,2)
hold on
plot(Pos(:,3),Acel(:,2)-x(6,:)','k','LineWidth',1.6);
grid on
ylabel('error acel-Y [m]')
xlabel('Tiempo [muestras]')
%legend('Real', 'FK')
%print(h4,'a_vs_t','-dpng','-r0');
hold off

% Innovaciones
h5=figure;
hold on
plot(xcorr(ek(1,:)),'k','LineWidth',1.2);
plot(xcorr(ek(2,:)),'r','LineWidth',1.2);
grid on
ylabel('Autocorrelacion')
xlabel('Tau')
legend('e1', 'e2')
%print(h5,'innovaciones','-dpng','-r0');
hold off
