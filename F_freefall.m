function dx = F_freefall(t,x, L, t_prev_stance, target_pos, k_f, max_dx_des)
% Derivative function for a 2D SLIP model.
%
% States:
%   x(1):   x - position
%   x(2):   x - velocity
%   x(3):   y - position
%   x(4):   y - velocity
%   x(5):   angle between spring and verticle line. (To the right is positive)
%   x(6):   angular velocity of the angle mentioned above 
dx = zeros(6,1);

% system parameters:
g = 9.81;   % gravitational constant (m/s^2)

% Controller
F_ctrl = Controller_flight(x, L, t_prev_stance, target_pos, k_f, max_dx_des);

dx(1) = x(2);
dx(2) = 0;
dx(3) = x(4);
dx(4) = -g;
dx(5) = x(6);
dx(6) = F_ctrl(2)/0.1;  % This is a wrong mechanics. Just for simple testing. TODO


