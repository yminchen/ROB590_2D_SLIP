function F = Controller_flight(x, L, t_prev_stance, target_pos, k_f, max_dx_des)
% Function for state feedback controller.
%
% F:
%   F(1):   Trust force (N)
%   F(2):   Joint Torque (N*m)
F = zeros(2,1);

% Position controller parameters
kp_pos = k_f(1);
kd_pos = k_f(2);
% Position controller (PD control)
dx_des = -kp_pos*(x(1)-target_pos) - kd_pos*x(2); % target_pos is fixed.
if dx_des>max_dx_des
    dx_des = max_dx_des;
elseif dx_des<-max_dx_des
    dx_des = -max_dx_des;
end

% dx_des = 2; % for tuning kp_rai

% Raibert style controller parameters
kp_rai = k_f(3);
max_phi_tar = 50*pi/180;
% Raibert style controller
x_des = x(2)*t_prev_stance/2 + kp_rai*(x(2)-dx_des);
phi_tar = asin(x_des/L);
if phi_tar > max_phi_tar
    phi_tar = max_phi_tar;
elseif phi_tar < -max_phi_tar
    phi_tar = -max_phi_tar;
end

% phi_tar = 1; % for tuning the next PD controller

% PD controller parameters
kp = 10;   % 1.5
kd = 1.7;   % 1.2
% PD controller for desired phi.
err = x(5) - phi_tar;
derr =  x(6);     
        %%% TODO: phi_target is dynamic, so I should change derr.
f_final = -kp*err - kd*derr;
        %%% TODO: Remember to put saturation in the end! 

% Assignment
F(1) = 0;
F(2) = f_final;



