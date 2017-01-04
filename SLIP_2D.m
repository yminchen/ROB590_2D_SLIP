%% SLIP model jumping in two dimension.

% Author: Yu-Ming Chen, University of Michigan, Ann Arbor
% Email: yminchen@umich.edu
% Date: 01/01/2017

% This code is modified from an example 
% "Simulation and Animation of a Linear and Nonlinear Pendulum Model"
% written by 
% James T. Allison, Assistant Professor, University of Illinois at
% Urbana-Champaign,


%% INITIALIZATION:
clear,clc
% settings
fflag = 1;          % flag to enable figure plot (don't enable this with animation at the same time)
aflag = not(fflag); % flag to enable animation creation
vflag = not(fflag); % flag to enable animation recording (create a avi video)
fignum = 1;         % figure number
subplotFlag = 0;    % flag to enable animation subplot
moviename = 'SLIP_2D.avi';  % avi file name for VideoWriter

% system parameters
m = 1;              % point mass (kg)
L = 1;              % spring length (m)
k = 200;            % spring constant    
                        % higher stiffness improves stability!
                        % (I can't tune raibert controller when k=100.)
d = 10;             % spring damping 
g = 9.81;           % gravitational constant (m/s^2)

% controller parameters
thrust_flag = 0;
target_pos = 10;
t_prev_stance = 0.4/(k/100);
H = 3;              % desired height
max_dx_des = 2;     % maximum of desired speed
dx_des = 0;         % desired speed (initialization)
E = 0;              % SLIP energy (initialization)
l_spr_low = 0;      % spring length when mass reaches lowest height 
prev_t =0;
% Raibert controller parameter
kp_pos = 0.3;      % position control
kp_rai = 0.1;       % Raibert sytle controller
        % 0.07 good, 0.1 much better (this might depend on spring stiffness)
        % For larger desired speed, you need higher kp_rai.
k_f = [kp_pos kp_rai];  % f stands for flight

% simulation parameters
T0 = 0;
Tf = 20;
% initial simulation parameters
t0 = T0;            % initial time (s)
tf = 0.01;          % final time (s) 
tstep = 0.01;       % time increment (s)
tspan = t0:tstep:tf;% time vector

% initial state  
x0 = [  0
        0 
        2
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
X0 = x0;

% contact position of spring and ground
contact_pos = zeros(2,1);

% phase (finite state machine)
phase = 0;          % 0: flight phase 
                    % 1: stance phase
                    
% output parameters
total_time = Tf;                % (sec) 
total_loop = total_time/tf;           
T = zeros(total_loop+1, 1);     % t_output
X = [T T T T T T];              % x_output
X(1,:) = x0;
                    
% plotting parameters
line_height = 7;

% create figure window for plotting
if fflag
    figure;
end 

% for testing
% [t, x] = ode45('F_nonlin_spring',tspan,x0);
% % [t, x] = ode45('F_nonlin_freefall',tspan,x0);
% plot(tspan,x(:,1));
% hold on;
% plot(tspan,x(:,2));

%% SIMULATION:

i_loop = 1;
while i_loop < total_loop + 1
    %%% flight phase %%%
    if phase == 0 
        
        [t, x] = ode45(@(t,x) F_freefall(t,x,L,...
            t_prev_stance,target_pos,k_f,max_dx_des), tspan, x0);
        
        n = size(x,1);
        % assign output
        X(i_loop+1, :) = x(n, :);
        T(i_loop+1, :) = [tspan(2)];

        % update the time info and initial state for ode45
        t0 = tf;
        tf = tf + tstep;
        tspan = t0:tstep:tf;
        x0 = x(n, :);
        
        % If it's hitting the ground, switch to stance phase.
        if x(n,3) - L*cos(x(n,5)) < 0
            phase = 1;
            % print
            display('switch to stance compression phase');
            
            % contact point
            contact_pos = [x(n,1) + x(n,3)*sin(x(n,5)); 0];
            % transition info
            prev_t = tspan(1);
            % flight controller info
            dx_des = -k_f(1)*(x(n,1)-target_pos);
            if dx_des>max_dx_des
                dx_des = max_dx_des;
            elseif dx_des<-max_dx_des
                dx_des = -max_dx_des;
            end
        end

    %%% stance compression phase %%%
    elseif phase == 1
        
        [t, x] = ode45(@(t,x) F_spring_wDamp(t,x,m,k,L,d,E,H,...
            contact_pos,thrust_flag,l_spr_low,dx_des), tspan, x0);
        
        n = size(x,1);
        % overwrite angle (not using ODE to get angle directly...)
        x(1,5) = atan((contact_pos(1)-x(1,1))/(x(1,3)-contact_pos(2)));
        x(n,5) = atan((contact_pos(1)-x(n,1))/(x(n,3)-contact_pos(2)));
        x(n,6) = (x(n,5)-x(1,5))/tstep;
        % assign output
        X(i_loop+1, :) = x(n, :);
        T(i_loop+1, :) = [tspan(2)];

        % update the time info and initial state for ode45
        t0 = tf;
        tf = tf + tstep;
        tspan = t0:tstep:tf;
        x0 = x(n, :);
        
        % If it's leaving the ground, switch to flight phase.
        if x(n,3) - L*cos(x(n,5)) > 0  % Use "spring length > L" instead?
            phase = 0;
            thrust_flag = 0;
            % print
            display('switch to flight phase');
            
            % transition info and plot
            t_prev_stance = tspan(1) - prev_t;
            if fflag
                hold on;
                fp(1) = fill([prev_t prev_t tspan(1) tspan(1)],[-line_height,line_height,line_height,-line_height], [0.9 0.9 0.9], 'EdgeColor',[0.9 0.9 0.9]);
                hold off;
            end
        elseif (thrust_flag == 0) && (x(n,3) > x(1,3))
            thrust_flag = 1;
            % print
            display('switch to stance thrust phase');
            
            % calculate the energy and spring at lowest height
            l_spr_low = rms([x(n,1);x(n,3)]-contact_pos);
            E = m*g*x(n,3) + 0.5*m*(x(n,2)^2+x(n,4)^2) + 0.5*k*(l_spr_low - L)^2;
        end
        
        % If it hits the ground, stop the simulation.
        if x(n,3) < 0
            i_loop = total_loop;
        end
    end

    %display(i_loop);
    i_loop = i_loop + 1;
end

%% Trajectary Plot
if fflag
    if phase == 1
        hold on;
        fp(1) = fill([prev_t prev_t tspan(1) tspan(1)],[-line_height,line_height,line_height,-line_height], [0.9 0.9 0.9], 'EdgeColor',[0.9 0.9 0.9]);
        hold off;
    end
        
    tspan = 0:tstep:total_time;
    hold on;
    fp(2) = plot(tspan,X(:,1),'b','LineWidth',2);
    fp(3) = plot(tspan,X(:,2),'r','LineWidth',2);
    fp(4) = plot(tspan,X(:,3),'g','LineWidth',2);
    fp(5) = plot(tspan,X(:,4),'k','LineWidth',2);
    fp(6) = plot(tspan,X(:,5),'m','LineWidth',2);
    fp(7) = plot(tspan,X(:,6),'c','LineWidth',2);
    fp(8) = plot([tspan(1) tspan(total_time/tstep+1)], [target_pos target_pos],'r--','LineWidth',1);
    hold off
    
    if d == 0
        title('\fontsize{12}\fontname{Arial Black}SLIP without Damping (2D)')
    else
        title('\fontsize{12}\fontname{Arial Black}SLIP with Damping (2D)')        
    end
    legend(fp([1 2 3 4 5 6 7 8]), 'stance phase',...
        'xPos (m)', 'xVel (m/s)', 'yPos (m)', 'yVel (m/s)',...
        'phi (rad)', 'dphi (rad/s)', 'xPos_{des} (m/s)');
    xlabel('\fontsize{10}\fontname{Arial Black} Time(s)');
    text(1,9,['\fontsize{10}\fontname{Arial Black}kp_{pos}: ' num2str(k_f(1),'%1.2f')], 'Color','red');
    text(1,8.5,['\fontsize{10}\fontname{Arial Black}kp_{rai} : ' num2str(k_f(2),'%1.2f')], 'Color','red');
    
    figure;
    plot(tspan,X(:,5),'m','LineWidth',2);hold on;
    plot(tspan,X(:,6),'c','LineWidth',2);
    legend('phi (rad)', 'dphi (rad/s)');
    title('\fontsize{12}\fontname{Arial Black}Angle plot');
    
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
    dt = (total_time-0)/8;
    ndt = floor(dt/tstep);   % number of time increments displayed in window
    h=figure(fignum); clf
    set(h,'Position',[0 0 1000 400]);
end

%% Animation
if aflag
    % step through each time increment and plot results
    if vflag
        vidObj = VideoWriter(moviename);
        vidObj.FrameRate = length(T)/(total_time-0);
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
            axis([-dt dt -0.1 X0(1)+0.1])

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
            axis([(tc-dt) (tc+dt) -0.1 X0(1)+0.1])
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


