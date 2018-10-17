function [out, e] =  KalmanFilterej7(A,B,C,D,Q,R,x0,p00,data,sigma)
% A,B,C,D Dinamica del sistema
% q, r covarianzas del ruido de entrada (proceso) y salida (sensor)
% x0 y p00 son condiciones iniciales y covarianza
% La data son los y-es

% Inicio
x_k1_k1 = x0;
y_k = data(1,:)';
P_k1_k1 = p00;
out = zeros(length(data),length(x0));
out(1,:) = x0;

for i=2:length(data)
% Prediccion
    x_k_k1 = A*x_k1_k1;
    P_k_k1 = A*P_k1_k1*A' + B*Q*B';
    K_k = P_k_k1 * C'*inv((R+C*P_k_k1*C'));
    if  (abs((data(i-1,1))==sigma) || (abs(data(i-1,2))==sigma))
        x_k_k = A*x_k1_k1;
    else
        x_k_k = x_k_k1 + K_k * (y_k - C * x_k_k1);
    end
    
    % Actualizacion
    
    P_k_k = (eye(size(A))-K_k*C) * P_k_k1;
    P_k1_k1 = P_k_k;
    out(i,:) = x_k_k';
    e(:,i)= y_k - C * x_k_k1;
    x_k1_k1 = x_k_k;
    y_k = data(i,:)';

end