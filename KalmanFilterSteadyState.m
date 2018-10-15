function [out, e, P, K] =  KalmanFilterSteadyState(A,B,C,D,Q,R,x0,p00,data)
% A,B,C,D Dinamica del sistema
% q, r covarianzas del ruido de entrada (proceso) y salida (sensor)
% x0 y p00 son condiciones iniciales y covarianza
% La data son los y-es
out = zeros(length(data),length(x0));
[P,vals, K] = dare(A',C',B*Q*B',R);
K = P*C'*inv(R+C*P*C');
out(1,:) = x0;
y_k = data(1,:)';
for i=2:length(data)
    x_k1_k1 = out(i-1,:)';
    cosa = K * (y_k - C * A*x_k1_k1);
    x_k_k = A*x_k1_k1 + K * (y_k - C * A*x_k1_k1);
    out(i,:) = x_k_k;
    e(:,i) = y_k - C * A*x_k1_k1;
    y_k = data(i,:)';
end