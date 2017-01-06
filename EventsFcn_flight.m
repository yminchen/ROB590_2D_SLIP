function [position,isterminal,direction] = EventsFcn_flight(t,x,L)

position = x(3) - L*cos(x(5)); % The value that we want to be zero
isterminal = 1;  % Halt integration 
direction = -1;   % The zero can be approached from either direction