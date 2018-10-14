%% Ejercicio 4
% a) Estimar variables de estado midiendo posición con un sesgo
% Voy a agregar al vector de estados común (de 6 cosas) 2 más, que son
% sesgo en x y sesgo en y

clear all;close all;clc;

load('datos.mat'); % Da Acel, Pos, tiempo, Vel

%DATOS
% IMPORTANTE: Parámetros que hay que analizar: 
% var_ruido_proc_ses, que es la varianza que le damos al ruido de proceso
% del sesgo.
% Valores iniciales para el sesgo: estos supongo que son cosas que
% diseñamos.
var_ruido_proc_pos=3e-4;
var_ruido_proc_vel=2e-3;
var_ruido_proc_acel=1e-2;
% Esto es un parámetro para que el filtro de Kalman pueda ajustar bien
var_ruido_proc_ses = 1e-6; 

A_sin_sesgo = [ 1 0 1 0 0.5 0;
        0 1 0 1 0 0.5;
        0 0 1 0 1 0;
        0 0 0 1 0 1;
        0 0 0 0 1 0;
        0 0 0 0 0 1  ];
A_d = [A_sin_sesgo zeros(6,2);
    zeros(2,6) eye(2)];
Q_d = diag([var_ruido_proc_pos,
            var_ruido_proc_pos,
            var_ruido_proc_vel,
            var_ruido_proc_vel,
            var_ruido_proc_acel,
            var_ruido_proc_acel,
            var_ruido_proc_ses,
            var_ruido_proc_ses
            ]);
%Condiciones iniciales:
x0 = [40 -200 0 0 0 0 0 0]';
P0_0 = diag([10^6 10^6, 100 100, 10 10, 1e7 1e7]);

% Medimos posición, y le sumamos el sesgo:

C_viejo = [1 0 0 0 0 0;
     0 1 0 0 0 0];
C = [C_viejo eye(2)];

B = eye(8);

% Armo las mediciones, con R para hacer el ruido y con su sesgo

sigma_pos= 100; %Ruido de medicion para coordenadas x e y
R= diag([sigma_pos^2 sigma_pos^2]);
sesgo_x = 300;
sesgo_y = 200;

yk(:,1)=Pos(:,1)+sigma_pos*randn(length(Pos(:,1)),1) + sesgo_x * ones(length(Pos(:,1)),1);
yk(:,2)=Pos(:,2)+sigma_pos*randn(length(Pos(:,2)),1) + sesgo_y * ones(length(Pos(:,2)),1);
N=length(Pos);

p00=P0_0;
D=0;
[x,ek] = KalmanFilter(A_d,B,C,D,Q_d,R,x0,p00,yk);

x=x';

%Determino observabilidad
O=obsv(A_d,C);
rank(O) %Tiene que ser 6 para que sea observable

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
print(h1,'trayectoria','-dpng','-r0');
hold off

% Posicion-x vs tiempo
h2=figure;
subplot(2,1,1)
hold on
plot(Pos(:,3),Pos(:,1),'k','LineWidth',1.6);
plot(Pos(:,3),x(1,:),'r.','LineWidth',1.6);
grid on
ylabel('Pos-X [m]')
xlabel('Tiempo [muestras]')
legend('Real', 'FK')
hold off

% Posicion-y vs tiempo
subplot(2,1,2)
hold on
plot(Pos(:,3),Pos(:,2),'k','LineWidth',1.6);
plot(Pos(:,3),x(2,:),'r.','LineWidth',1.6);
grid on
ylabel('Pos-Y [m]')
xlabel('Tiempo [muestras]')
legend('Real', 'FK')
print(h2,'p_vs_t','-dpng','-r0');
hold off

% Velocidad-x vs tiempo
h3=figure;
subplot(2,1,1)
hold on
plot(Pos(:,3),Vel(:,1),'k','LineWidth',1.6);
plot(Pos(:,3),x(3,:),'r.','LineWidth',1.6);
grid on
ylabel('Vel-X [m]')
xlabel('Tiempo [muestras]')
legend('Real', 'FK')
hold off

% Velocidad-y vs tiempo
subplot(2,1,2)
hold on
plot(Pos(:,3),Vel(:,2),'k','LineWidth',1.6);
plot(Pos(:,3),x(4,:),'r.','LineWidth',1.6);
grid on
ylabel('Vel-Y [m]')
xlabel('Tiempo [muestras]')
legend('Real', 'FK')
print(h3,'v_vs_t','-dpng','-r0');
hold off

% Aceleracion-x vs tiempo
h4=figure;
subplot(2,1,1)
hold on
plot(Pos(:,3),Acel(:,1),'k','LineWidth',1.6);
plot(Pos(:,3),x(5,:),'r.','LineWidth',1.6);
grid on
ylabel('Acel-X [m]')
xlabel('Tiempo [muestras]')
legend('Real', 'FK')
hold off

% Aceleracion-y vs tiempo
subplot(2,1,2)
hold on
plot(Pos(:,3),Acel(:,2),'k','LineWidth',1.6);
plot(Pos(:,3),x(6,:),'r.','LineWidth',1.6);
grid on
ylabel('Acel-Y [m]')
xlabel('Tiempo [muestras]')
legend('Real', 'FK')
print(h4,'a_vs_t','-dpng','-r0');
hold off

% Sesgo x vs tiempo
h6=figure;
subplot(2,1,1)
hold on
plot(Pos(:,3),x(7,:),'r-','LineWidth',1.6);
grid on
ylabel('Estimación del sesgo (x) [m]')
xlabel('Tiempo [muestras]')
%legend('Real', 'FK')
hold off

% Sesgo y vs tiempo
subplot(2,1,2)
hold on
plot(Pos(:,3),x(8,:),'r-','LineWidth',1.6);
grid on
ylabel('Estimación del sesgo (y) [m]')
xlabel('Tiempo [muestras]')
%legend('Real', 'FK')
print(h6,'sesgo','-dpng','-r0');
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
print(h5,'innovaciones','-dpng','-r0');
hold off
