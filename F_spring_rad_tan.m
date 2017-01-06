function dx = F_spring_rad_tan(t,x,m,k,L,d,thrust_flag,L_low, E_des, E_low)
% Derivative function for a 2D SLIP model.
%
% States:
%   x(1):   spring length (meter)
%   x(2):   spring velocity (meter/sec)
%   x(3):   angle between spring and verticle line. (To the right is positive)
%   x(4):   angular velocity of the angle mentioned above 
dx = zeros(4,1);

% system parameters:
g = 9.81;                       % gravitational constant (m/s^2)

% current energy
E = m*g*x(1)*cos(x(3)) + 0.5*m*(x(2)^2+(x(1)*x(4))^2) + 0.5*k*(L-x(1))^2;   

% Controller for stance phase   % This needs to be modified.
if thrust_flag && (E_des > E) 
    F_ctrl = (E_des-E_low)/(L-L_low);
%     F_ctrl = (E_des-E)*10;
else
    F_ctrl = 0;
end

% Calculate derivative
dx(1) = x(2);
dx(2) = -g*cos(x(3)) + k/m*(L-x(1)) + (F_ctrl - d*x(2))/m;
dx(3) = x(4);
dx(4) =  g*sin(x(3))/x(1);


