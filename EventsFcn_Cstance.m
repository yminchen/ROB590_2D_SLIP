function [position,isterminal,direction] = EventsFcn_Cstance(t,x,L)

position = x(4); % The value that we want to be zero
isterminal = 1;  % Halt integration 
direction = 1;   % The zero can be approached from either direction