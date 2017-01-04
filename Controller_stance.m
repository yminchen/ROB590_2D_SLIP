function F = Controller_stance(x)
% Function for state feedback controller.
%
% F:
%   F(1):   Trust force (N)
%   F(2):   Joint Torque (N*m)
F = zeros(2,1);



% Remember to put saturation in the end!!!!!!!!!!!! TODO

F(1) = 0;
F(2) = 0;


