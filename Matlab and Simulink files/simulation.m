 %% Parameters
kcw = 1;      % feedback gain
L = 0.5;      % half-length of track
v0 = 0.5;     % initial velocity
t_span = [0 10]; % simulation time span

%% ODE definition (cart dynamics)
% xv = [x; v]
odefun = @(t, xv) [xv(2);  kcw * sign(xv(1)) .* abs(xv(1)).^0.5 .* log(1 - abs(xv(1))/L)];

%% Solve ODE using ode45
opts = odeset('RelTol',1e-6,'AbsTol',1e-9);
xv0 = [0; v0];  % initial [x; v]
[t_sol, sol] = ode45(odefun, t_span, xv0, opts);

x_traj = sol(:,1);
v_traj = sol(:,2);
a_traj = kcw * sign(x_traj) .* abs(x_traj).^0.5 .* log(1 - abs(x_traj)/L);

%% Detect zero-crossings for three oscillations
zero_crossings = 0;
prev_sign = sign(x_traj(1));
osc_end_idx = length(x_traj);

for i = 2:length(x_traj)
    curr_sign = sign(x_traj(i));
    if curr_sign ~= prev_sign && curr_sign ~= 0
        zero_crossings = zero_crossings + 1;
    end
    prev_sign = curr_sign;
    if zero_crossings >= 6  % 3 full oscillations
        osc_end_idx = i;
        break;
    end
end

% Trim trajectories to 3 oscillations
x_traj = x_traj(1:osc_end_idx);
v_traj = v_traj(1:osc_end_idx);
a_traj = a_traj(1:osc_end_idx);
t_sol = t_sol(1:osc_end_idx);

%% Dynamic 2D Visualization
figure;
cart_width = 0.05;
cart_height = 0.03;

for t = 1:length(x_traj)
    clf; hold on; axis equal; grid on;
    xlim([-L-0.1 L+0.1]); ylim([-0.1 0.1]);
    
    % Track and barriers
    plot([-L L],[0 0],'k','LineWidth',2);
    plot([-L -L],[-0.05 0.05],'r','LineWidth',3);
    plot([L L],[-0.05 0.05],'r','LineWidth',3);
    
    % Cart
    rectangle('Position',[x_traj(t)-cart_width/2,-cart_height/2,cart_width,cart_height],...
              'FaceColor','b');
    
    % Acceleration arrow
    quiver(x_traj(t),0.06,0,a_traj(t)*0.05,'r','LineWidth',2,'MaxHeadSize',2);
    
    % Live values
    text(x_traj(t),0.1,sprintf('x = %.3f m', x_traj(t)),'HorizontalAlignment','center','FontSize',10);
    text(x_traj(t),0.08,sprintf('a = %.3f m/s^2', a_traj(t)),'HorizontalAlignment','center','FontSize',10,'Color','r');
    
    title(sprintf('Cart Simulation: t = %.2f s', t_sol(t)));
    xlabel('Position x (m)');
    ylabel('Track / y-axis');
    pause(0.01);
end

%% Acceleration vs Position after 3 oscillations
figure;
plot(x_traj, a_traj,'LineWidth',2);
xlabel('x (m)');
ylabel('a(x)');
title('Acceleration vs Position (after three oscillations)');
grid on;
