clear all; close all; clc;

load('datos.mat');

Pos1=Pos;


Pos=Pos.*(binornd(1,0.9,[1,length(Pos(:,1))]))';
Pos(:,1)=Pos1(:,1);

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

%Condiciones iniciales:
x0 = [40 -200 0 0 0 0]';
P0_0 = diag([10^6 10^6, 100 100, 10 10]);

% a) medicion posicion
C = [1 0 0 0 0 0;
     0 1 0 0 0 0];
B = eye(6);
sigma_pos= 3e-4; %Ruido de medicion para coordenadas x e y
R= diag([sigma_pos*sigma_pos sigma_pos*sigma_pos]);
% Armo las mediciones con el ruido
yk(:,1)=Pos(:,1)+sigma_pos*randn(length(Pos(:,1)),1);
yk(:,2)=Pos(:,2)+sigma_pos*randn(length(Pos(:,2)),1);
yk;
N=length(Pos);

p00=P0_0;
D=0;
[x,ek] = KalmanFilterej7(A_d,B,C,D,Q_d,R,x0,p00,yk,sigma_pos);
x=x';

O=obsv(A_d,C);
rank(O) %Tiene que ser 6 para que sea observable

%Grafico la trayectoria
h1=figure;
hold on
%plot(yk(:,1),yk(:,2),'LineWidth',1.2,'Color',[0 161 0]/255);
plot(Pos1(:,1),Pos1(:,2),'b','LineWidth',1.6);
plot(x(1,:),x(2,:),'r','LineWidth',1.6);
axis([-7000 1000 -3000 1000])
grid on
ylabel('Y [m]')
xlabel('X [m]')
title(['Trayectoria estimada'])
legend('Pos sin FK','Pos Real',  'Pos con FK')
print(h1,'trayectoria_R_100','-dpng','-r0');
hold off

% Posicion-x vs tiempo
h2=figure;
subplot(2,1,1)
hold on
plot(Pos1(:,3),Pos1(:,1),'k','LineWidth',1.6);
plot(Pos(:,3),x(1,:),'r.','LineWidth',1.6);
grid on
ylabel('Pos-X [m]')
xlabel('Tiempo [muestras]')
legend('Real', 'FK')
hold off

% Posicion-y vs tiempo
subplot(2,1,2)
hold on
plot(Pos1(:,3),Pos1(:,2),'k','LineWidth',1.6);
plot(Pos(:,3),x(2,:),'r.','LineWidth',1.6);
grid on
ylabel('Pos-Y [m]')
xlabel('Tiempo [muestras]')
legend('Real', 'FK')
print(h2,'p_vs_t_R_100','-dpng','-r0');
hold off
