clear all;close all;clc;


datos = load('datos.mat');
T = 1;
A =[1 0 T 0 T^2/2 0;
    0 1 0 T 0 T^2/2;
    0 0 1 0 T 0;
    0 0 0 1 0 T;
    0 0 0 0 1 0;
    0 0 0 0 0 1];
B = eye(6);
Q = [eye(2)*3e-4 zeros(2) zeros(2);
     zeros(2) eye(2)*2e-3 zeros(2);
     zeros(2) zeros(2) eye(2)*1e-2]*T;
R = eye(2) * 60^2;
C = [eye(2); 0 0; 0 0; 0 0; 0 0];
C = C';
D = 0;
x0 = [40 -200 0 0 0 0]';
p00 = eye(6);
p00(2,2) = 100^2;
p00(1,1) = p00(2,2);
p00(4,4) = 1;
p00(3,3) = p00(4,4);
p00(6,6) = 0.1;
p00(5,5) = p00(6,6);
data = datos.Pos(:,1:2);
data_tocado = data +  60*randn(size(data));
salida = KalmanFilter(A,B,C,D,Q,R,x0,p00,data_tocado);
plot (salida(:,1),salida(:,2));
hold on
plot (data_tocado(:,1),data_tocado(:,2),'r');
plot (data(:,1),data(:,2),'c');

