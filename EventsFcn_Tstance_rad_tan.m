function [position,isterminal,direction] = EventsFcn_Tstance_rad_tan(t,x,L)

position = [x(1)-L, pi/2-abs(x(3))]; % The value that we want to be zero
isterminal = [1, 1];  % Halt integration 
direction = [1, -1];   % The direction that the zero is approached 