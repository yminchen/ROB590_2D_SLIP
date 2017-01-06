function dx = F_spring(t,x,m,k,L,d,E,H,contact_pos,thrust_flag,l_spr_low, dx_des)
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
g = 9.81;                       % gravitational constant (m/s^2)
l_spring = sum(([x(1);x(3)]-contact_pos).^2)^0.5;    
                                % current length of spring (m)
% change rate of spring length (m/s)
cx = x(1) - contact_pos(1);
cy = x(3) - contact_pos(2);
dL = (cx*x(2)+cy*x(4))/(cx^2+cy^2)^0.5;

% Controller for stance phase   % This needs to be modified.
E_des = m*g*H + d*dL*(L-l_spr_low) + 0.5*m*dx_des^2; % desired energy
if thrust_flag && (E_des > E) 
    F_ctrl = (E_des-E)/(L-l_spr_low);
%     F_ctrl = (E_des-E)*10;
else
    F_ctrl = 0;
end

% Calculate derivative
idea = 1;   
%%% idea I %%%
if idea == 1
    dx(1) = x(2);
    dx(2) = (- k*(l_spring-L) + F_ctrl - d*dL)/m*(-sin(x(5)));
    dx(3) = x(4);
    dx(4) = (- k*(l_spring-L) + F_ctrl - d*dL)/m*  cos(x(5)) -g ;
    dx(5) = x(6);
    %%% idea I(a)
    dx(6) = g*sin(x(5))/l_spring; % Why this doesn't work?
    %%% idea I(b)-1
%     c = x(1) - contact_pos(1);
%     dx(6) = -2*cos(x(5))*sin(x(5))*x(6)*(-x(2)*x(3)+x(4)*c)/x(3)^2 +...
%         (cos(x(5))^2)*((x(4)*x(2)-dx(2)*x(3))/x(3)^2 + ...
%         ((dx(4)*c+x(4)*x(2))*x(3)^2 - (x(4)*c)*2*x(3)*x(4))/x(3)^4);
    %%% idea I(b)-2
%     c = contact_pos(1) - x(1);
%     dx(6) = (-x(2)*x(3)-x(4)*c)*...
%         2*cos(x(5))*(-sin(x(5))*x(6)*x(3)-cos(x(5))*x(4))/x(3)^3 +...
%         ((cos(x(5))/x(3))^2)*(-dx(2)*x(3)-dx(4)*c);
    
%%% idea II %%%    (decouple tangent and radius direction)
elseif idea == 2
    dx(1) = x(2);
    dx(2) = (- k*(l_spring-L) + F_ctrl - d*dL)/m*(-sin(x(5)));
    dx(3) = x(4);
    dx(4) = (- k*(l_spring-L) + F_ctrl - d*dL)/m*  cos(x(5)) -g ;
    dx(5) = -(x(2)*cos(x(5))-x(4)*sin(x(5)) )/l_spring;
    dx(6) = g*l_spring*sin(x(5))/(l_spring^2); 

%%% idea III %%%    (overwrite phi)
elseif idea == 3
    x(5) = atan((contact_pos(1)-x(1))/(x(3)-contact_pos(2)));
    dx(1) = x(2);
    dx(2) = (- k*(l_spring-L) + F_ctrl - d*dL)/m*(-sin(x(5)));
    dx(3) = x(4);
    dx(4) = (- k*(l_spring-L) + F_ctrl - d*dL)/m*  cos(x(5)) -g ;
    dx(5) = 0;
    dx(6) = 0;
end



