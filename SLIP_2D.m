%% SLIP model jumping in two dimension.

% Author: Yu-Ming Chen, University of Michigan, Ann Arbor
% Email: yminchen@umich.edu
% Date: 01/01/2017

% This code is modified from an example 
% "Simulation and Animation of a Linear and Nonlinear Pendulum Model"
% written by 
% James T. Allison, Assistant Professor, University of Illinois at
% Urbana-Champaign.


%% INITIALIZATION:
clear,clc
% settings
fflag = 1;          % flag to enable figure plot (don't enable this with animation at the same time)
aflag = not(fflag); % flag to enable animation creation
vflag = not(fflag); % flag to enable animation recording (create a avi video)
fignum = 1;         % figure number
moviename = 'SLIP_2D.avi';  % avi file name for VideoWriter
subplotFlag = 0;    % flag to enable animation subplot
xy_cor_flag = 0;    % flag to enable using x-y coordinate for ode45 during stance phase


% system parameters
m = 1;              % point mass (kg)
L = 0.7;            % spring length (m)
k = 2000;           % spring constant    
        % higher stiffness improves stability.
        % if you want higher speed, you need higher k.
d = 10;             % spring damping 
g = 9.81;           % gravitational constant (m/s^2)

% controller parameters
thrust_flag = 0;
target_pos = 10;
t_prev_stance = 0.2/(k/100);
H = 1;              % desired height (effecting kp_rai and kp_pos!)
max_dx_des = 2;     % maximum of desired speed (not real speed)
        % max_dx_des can go to 8 or higher, but then it also jumps higher.
dx_des = 0;         % desired speed (initialized to be 0)
E_low = 0;          % SLIP energy at lowest point (initialized to be 0)
E_des = 0;          % SLIP desired energy (initialized to be 0)
L_low = 0;          % spring length when mass reaches lowest height 
x_td = 0;           % state vector at previous touchdown 
prev_t = 0;
% position controller parameters
kp_pos = 2;       
        % kp_pos depends on max_dx_des.
kd_pos = 1.5;
% Raibert controller parameter
kp_rai = 0.04;    % Raibert sytle controller
        % kp_rai depends on H (the height) and k (the stiffness).
        % For larger desired speed, you need higher kp_rai.
        % But the larger kp_rai is, the more unstable when changing speed.
        % When it's stable enough, you can try to increase max_dx_des.
k_f = [kp_pos kd_pos kp_rai];  % f stands for flight

% simulation parameters
T0 = 0;
Tf = 9;
% ode45 events parameters
t_evmax = 1;
% initial simulation parameters
tstep = 0.01;               % time increment (s)
tspan = T0:tstep:t_evmax;   % initial time vector
            % Including intermidiate time allows us to fix the 
            % length of ode output. 

% initial state  
x0 = [  0
        0 
        1.2
        0
        0
        0]';        
    %   x(1):   x - position (meter)
    %   x(2):   x - velocity (meter/sec)
    %   x(3):   y - position
    %   x(4):   y - velocity
    %   x(5):   angle (rad) between spring and verticle line. (To the right is positive)
    %   x(6):   angular velocity (rad/s) of the angle mentioned above 
    % PS: x0 is a row vector.

% contact position of spring and ground
contact_pos = zeros(2,1);

% phase (finite state machine)
phase = 0;          % 0: flight phase 
                    % 1: stance phase
                    
% output value        
T = zeros(0,0);     % t_output (will have one column)
T(1,1) = T0;
X = zeros(0,0);     % x_output (will have six columns)
X(1,:) = x0;
lenX = 1;           % the number of rows of x_output 
                    
% plotting parameters
line_height = 7;

% create figure window for plotting
if fflag
    figure;
end 


%% SIMULATION:

while T(size(T,1)) < Tf
    %%% flight phase %%%
    if phase == 0 
        
        options = odeset('Events', @(t,x) EventsFcn_flight(t,x,L));
        [t,x,te,xe,ie] = ode45(@(t,x) F_freefall(t,x,L,...
            t_prev_stance,target_pos,k_f,max_dx_des), tspan, x0, options);
        
        n = size(x,1);
        % assign output
        X(lenX:lenX+n-1, :) = x;
        T(lenX:lenX+n-1, :) = t;
        
        % update the time info and initial state for ode45
        lenX = lenX + n-1;
        tspan = t(n) : tstep : t(n)+t_evmax;
        x0 = x(n, :);
        
        % If it's hitting the ground, switch to stance phase.
        if size(te,1)>0 
            phase = 1;
            % print
            display('switch to stance compression phase');
            
            % contact point
            contact_pos = [x(n,1) + L*sin(x(n,5)); 0];
            % transition info
            prev_t = t(n);
            % flight controller info
            dx_des = -k_f(1)*((x(n,1)+x(n,2)*t_prev_stance/2)-target_pos)...
                     -k_f(2)*x(n,2);
            if dx_des>max_dx_des
                dx_des = max_dx_des;
            elseif dx_des<-max_dx_des
                dx_des = -max_dx_des;
            end
            % state vector at touch down
            x_td = x(n,:);
            
            % if use x-y coordinate for stance phase
            if xy_cor_flag
                % update the initial condition for angular velocity when hiting
                % the ground
                cx = X(lenX,1) - contact_pos(1); 
                cy = X(lenX,3) - contact_pos(2);
                x0(6) = (-X(lenX,2)*cos(X(lenX,5))-X(lenX,4)*sin(X(lenX,5)))/((cx^2+cy^2)^0.5) ;
            end
        end
        
    %%% stance phase %%%
    elseif phase == 1
        
        % if use x-y coordinate for stance phase
        if xy_cor_flag
            if thrust_flag
                options = odeset('Events', @(t,x) EventsFcn_Tstance(t,x,L));
            else
                options = odeset('Events', @(t,x) EventsFcn_Cstance(t,x,L));
            end
            [t,x,te,xe,ie] = ode45(@(t,x) F_spring(t,x,m,k,L,d,...
                contact_pos,thrust_flag,L_low,E_des,E_low), tspan, x0, options);
            n = size(x,1);
        
        % if use rad-tan coordinate for stance phase
        else
            if thrust_flag
                options = odeset('Events', @(t,x) EventsFcn_Tstance_rad_tan(t,x,L));
            else
                options = odeset('Events', @(t,x) EventsFcn_Cstance_rad_tan(t,x));
            end

            % transform to radius-tangent coordinate
            cx = X(lenX,1) - contact_pos(1); 
            cy = X(lenX,3) - contact_pos(2);
            x0 = [  (cx^2+cy^2)^0.5 
                    -X(lenX,2)*sin(X(lenX,5))+X(lenX,4)*cos(X(lenX,5))      % equivalent to: (cx*X(lenX,2)+cy*X(lenX,4))/(cx^2+cy^2)^0.5
                    X(lenX,5)
                    (-X(lenX,2)*cos(X(lenX,5))-X(lenX,4)*sin(X(lenX,5)))/((cx^2+cy^2)^0.5 )]';
                %   x0(1):   spring length (meter)
                %   x0(2):   spring velocity (meter/sec)
                %   x0(3):   angle (rad) between spring and verticle line. (To the right is positive)
                %   x0(4):   angular velocity (rad/s) of the angle mentioned above 

            % ode45
            [t,x,te,xe,ie] = ode45(@(t,x) F_spring_rad_tan(t,x,m,k,L,d,...
                thrust_flag,L_low,E_des,E_low), tspan, x0, options);
            n = size(x,1);

            % transform to x-y coordinate
            x = [   contact_pos(1) - x(:,1).*sin(x(:,3)),...
                    -x(:,2).*sin(x(:,3))-x(:,1).*x(:,4).*cos(x(:,3)),...
                    x(:,1).*cos(x(:,3)),...
                    +x(:,2).*cos(x(:,3))-x(:,1).*x(:,4).*sin(x(:,3)),...
                    x(:,3),...
                    x(:,4)];
        end
        
        % assign output
        X(lenX:lenX+n-1, :) = x;
        T(lenX:lenX+n-1, :) = t;
        
        % update the time info and initial state for ode45
        lenX = lenX + n-1;
        tspan = t(n) : tstep : t(n)+t_evmax;
        x0 = x(n, :);        
        
        % If the point mass falls on the ground, stop the simulation.
        if (size(te,1)>0) && (ie==2)
            break;
            
        % If it's leaving the ground, switch to flight phase.
        elseif (size(te,1)>0) && (thrust_flag==1)
            phase = 0;
            thrust_flag = 0;
            % print
            display('switch to flight phase');
            
            % transition info and plot
            t_prev_stance = t(n) - prev_t;
            if fflag
                hold on;
                fp(1) = fill([prev_t prev_t t(n) t(n)],[-line_height,line_height,line_height,-line_height], [0.9 0.9 0.9], 'EdgeColor',[0.9 0.9 0.9]);
                hold off;
            end
            
        % If it reaches the lowest hieght, switch to thrust phase.
        elseif (size(te,1)>0) && (thrust_flag == 0)
            thrust_flag = 1;
            % print
            display('switch to stance thrust phase');
            
            % calculate spring length and energy at lowest height
            L_low = sum(([x(n,1);x(n,3)]-contact_pos).^2)^0.5;   
            E_low = m*g*x(n,3) + 0.5*m*(x(n,2)^2+x(n,4)^2) + 0.5*k*(L_low - L)^2;
            % calculate desired energy
            dL = abs(-x_td(2)*sin(x_td(5))+x_td(4)*cos(x_td(5)));
            E_des = m*g*H + pi/4*d*dL*(L-L_low) + 0.5*m*dx_des^2;
        end
    end

end

%%%% TODO: In order to make the time in the video super accurate, 
%%%%       we should record the output every 0.01 second. 
%%%%       So don't assign the event time to the output (both X and T)
%%%%       (although we can assign it to another variable).
%%%% FYI: frame rate is decided by "vidObj.FrameRate"

%% Trajectary Plot
if fflag
    if phase == 1
        hold on;
        fp(1) = fill([prev_t prev_t tspan(1) tspan(1)],[-line_height,line_height,line_height,-line_height], [0.9 0.9 0.9], 'EdgeColor',[0.9 0.9 0.9]);
        hold off;
    end
        
    hold on;
    fp(2) = plot(T,X(:,1),'b','LineWidth',2);
    fp(3) = plot(T,X(:,2),'r','LineWidth',2);
    fp(4) = plot(T,X(:,3),'g','LineWidth',2);
    fp(5) = plot(T,X(:,4),'k','LineWidth',2);
    fp(6) = plot(T,X(:,5),'m','LineWidth',2);
    fp(7) = plot(T,X(:,6),'c','LineWidth',2);
    fp(8) = plot([T(1) T(size(T,1))], [target_pos target_pos],'r--','LineWidth',1);
    hold off
    
    if d == 0
        title('\fontsize{12}\fontname{Arial Black}SLIP without Damping (2D)')
    else
        title('\fontsize{12}\fontname{Arial Black}SLIP with Damping (2D)')        
    end
    legend(fp([1 2 3 4 5 6 7 8]), 'stance phase',...
        'xPos (m)',...
        'xVel (m/s)', 'yPos (m)', 'yVel (m/s)',...
        'phi (rad)', 'dphi (rad/s)', 'xPos_{des} (m)');
    xlabel('\fontsize{10}\fontname{Arial Black} Time(s)');
    text(1,9,['\fontsize{10}\fontname{Arial Black}kp_{pos}: ' num2str(k_f(1),'%1.2f')], 'Color','red');
    text(1,8.5,['\fontsize{10}\fontname{Arial Black}kd_{pos} : ' num2str(k_f(2),'%1.2f')], 'Color','red');
    text(1,8,['\fontsize{10}\fontname{Arial Black}kp_{rai} : ' num2str(k_f(3),'%1.2f')], 'Color','red');
%     axis([0 T(size(T,1)) 0 2]);
    
    figure;
    plot(T,X(:,5),'m','LineWidth',2);hold on;
    plot(T,X(:,6),'c','LineWidth',2);
    legend('phi (rad)', 'dphi (rad/s)');
    title('\fontsize{12}\fontname{Arial Black}Angle plot');
    axis([0 T(size(T,1)) -2 2]);
    
    figure;
    plot(X(:,1),X(:,3),'b','LineWidth',2); 
    title('\fontsize{12}\fontname{Arial Black}Trajectory of 2D SLIP');
    ylabel('\fontsize{10}\fontname{Arial Black} (m)');
    xlabel('\fontsize{10}\fontname{Arial Black} (m)');
end

%% POSTPROCESSING:

% Calculate max min velocity
Vmax = max(X(:,2));
Vmin = min(X(:,2));

% position plot window size
if aflag
    dt = (Tf-0)/8;
    ndt = floor(dt/tstep);   % number of time increments displayed in window
    h=figure(fignum); clf
    set(h,'Position',[0 0 1000 400]);
end

%% Animation
if aflag
    % step through each time increment and plot results
    if vflag
        vidObj = VideoWriter(moviename);
        vidObj.FrameRate = length(T)/(Tf-0);
        open(vidObj);
        F(length(T)).cdata = []; F(length(T)).colormap = []; % preallocate
    end
    
    boarderR = max(X(:,1))+1;
    boarderL = min(X(:,1))-1;
    % This for loop would show animation as well as store the animation in F().
    for ti=1:length(T)
        % Position and (orientation) of SLIP mass at T(ti)
        x1 = X(ti);             % x coord
        %y1 = Y(ti);             % y coord
        %s1 = S(ti,1);           % rotation

        %%%%% Prepare figure %%%%
        figure(fignum); clf; 

        %%%%% Plot physical world (1st subfigure) %%%%
        if subplotFlag
           subplot(1,3,1);     
           subplot('Position',[0.05 0.05 0.30 0.95]); 
        end
        hold on
        axis equal; axis([boarderL boarderR -0.1 H+1])

        % Plot ground
        fill([boarderL boarderR boarderR boarderL],[0,0,-0.1,-0.1], [0 0 0], 'EdgeColor',[0 0 0]);
        % Plot target position
        scatter(target_pos,0,50,'MarkerEdgeColor','b',...
                  'MarkerFaceColor','g',...
                  'LineWidth',1);
        % Plot spring
        if X(ti,3) - L*cos(X(ti,5)) > 0
            plot([X(ti,1) X(ti,1)+L*sin(X(ti,5))],[X(ti,3) X(ti,3)-L*cos(X(ti,5))],'r','LineWidth',2);
        else    
            plot([X(ti,1) X(ti,1)+X(ti,3)*tan(X(ti,5))],[X(ti,3) 0],'r','LineWidth',2);
        end
        % Plot mass shape
        scatter(X(ti,1),X(ti,3),40,'MarkerEdgeColor',[0 0 0],...
                  'MarkerFaceColor',[0 0 0],...
                  'LineWidth',1.5);
        % Display time on plot
        tc = T(ti);     % current time
        text(-0.95*L,0.8*L+0.1,'\fontsize{10}\fontname{Arial Black}elapsed time:')
        text(-0.85*L,0.5*L+0.1,['\fontsize{10}\fontname{Arial Black}' ...
            num2str(tc,'%1.1f') ' sec'])
        
        title('\fontsize{12}\fontname{Arial Black}SLIP Animation (2D)')
        xlabel('\fontsize{10}\fontname{Arial Black} (m)')
        ylabel('\fontsize{10}\fontname{Arial Black} (m)')
        
        %%%%% The other two plots %%%%
        if subplotFlag
            %%%%% Plot position trajectory (2nd subfigure)%%%%
            subplot(1,3,2);
            subplot('Position',[0.40 0.34 0.125 0.42]); hold on
            axis([-dt dt -0.1 X(1,1)+0.1])

            % obtain time history for position to current time
            if (ti-ndt) >= 1 
                % full time history indices
                thi = ti-ndt:ti;
                x2h = X(thi);
            else
                % partial time history indices
                thi = 1:ti;
                x2h = X(thi);
            end
            th = T(thi);
            % plot position time history 
            plot(th,x2h,'b-','LineWidth',3); hold on
            axis([(tc-dt) (tc+dt) -0.1 X(1,1)+0.1])
            % plot marker
            plot(tc,x1,'k+','MarkerSize',18,'LineWidth',2)
            plot(tc,x1,'ko','MarkerSize',10,'LineWidth',2)
            % plot vertical position line
            plot([(tc-dt) (tc+dt)],[x1 x1],'r-')
            % plot zero position line
            plot([(tc-dt) (tc+dt)],[0 0],'k-')

            title('\fontsize{12}\fontname{Arial Black}SLIP Mass Height')
            xlabel('\fontsize{10}\fontname{Arial Black}Time (sec)')
            ylabel('\fontsize{10}\fontname{Arial Black}Height (m)')
            set(gca,'XTick',0:ceil(Tf)+1)
            clear th thi x2h 

            %%%%% plot velocity time history (3rd subfigure)%%%%
            ax(3) = subplot(1,3,3);
            subplot('Position',[0.60 .34 0.125 0.42]); hold on
            axis([-dt dt Vmin-1 Vmax+1])

            % obtain velocity time history to current time
                % nonlinear 
            if (ti-ndt) >= 1 
                % full time history
                thi = ti-ndt:ti;
                Vth = X(thi,2);
            else
                % partial time history
                thi = 1:ti;
                Vth = X(thi,2);
            end
            th = T(thi);
            % plot time history
            plot(th,Vth,'g-','LineWidth',3); hold on
            axis([(tc-dt) (tc+dt) Vmin-1 Vmax+1])
            % plot marker
            plot(tc,X(ti,2),'k+','MarkerSize',18,'LineWidth',2)
            plot(tc,X(ti,2),'ko','MarkerSize',10,'LineWidth',2)
            % plot vertical position line
            plot([(tc-dt) (tc+dt)],[X(ti,2) X(ti,2)],'r-')
            % plot zero velocity line
            plot([(tc-dt) (tc+dt)],[0 0],'k-')

            title('\fontsize{12}\fontname{Arial Black}SLIP Mass Velocity')
            xlabel('\fontsize{10}\fontname{Arial Black}Time (sec)')
            ylabel('\fontsize{10}\fontname{Arial Black}Velocity (m/s)')
            set(gca,'XTick',0:ceil(Tf)+1)
            set(gcf,'Color','w')
            clear th thi Vth 
        end

        h=figure(fignum);
        F(ti) = getframe(h,[0 0 1000 400]); 
        if vflag
            writeVideo(vidObj,F(ti));
        end

    end

    if vflag
        close(vidObj);
        %moviename2 = 'SLIP_2D(2).avi';      % avi file name for movie2avi
        %movie2avi(F,moviename2,'fps',length(T)/(Tf-t0),'compression','none')
    end

end


