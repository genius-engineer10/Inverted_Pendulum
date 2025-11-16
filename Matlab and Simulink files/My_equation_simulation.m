%% Parameters
kcw = 1;      
L = 0.5;      
B = 5;        % tanh smoothing 
A = 1;        % velocity damping
dt = 0.01;    
max_steps = 5000;

%% Preallocate arrays
x_traj = zeros(1,max_steps);
v_traj = zeros(1,max_steps);
a_traj = zeros(1,max_steps);

x_curr = 0.0;  
v_curr = 0.5;   
idx = 1;

%% Simulation loop
while idx <= max_steps
    % Compute acceleration 
    arg = B * (1 / (1 - abs(x_curr)/L));
    a_curr = -kcw .* x_curr * (abs(x_curr)/L)^0.5 * tanh(arg) - A*v_curr;
    
    % Update velocity and position
    v_curr = v_curr + a_curr * dt;
    x_curr = x_curr + v_curr * dt;
    
    % Store current values
    x_traj(idx) = x_curr;
    v_traj(idx) = v_curr;
    a_traj(idx) = a_curr;
    
    % Natural stop condition to avoid runaway
    if abs(x_curr) > L*1.2 || isnan(x_curr)
        disp('Simulation stopped: cart exceeded limit or NaN encountered');
        break;
    end
    
    idx = idx + 1;
end

%  Ensure idx does not exceed array length
if idx > max_steps
    idx = max_steps;
end

% Trim arrays safely
x_traj = x_traj(1:idx);
v_traj = v_traj(1:idx);
a_traj = a_traj(1:idx);

%% 2D Visualization: dynamic animation
figure;
cart_width = 0.05;
cart_height = 0.03;

for t = 1:length(x_traj)
    clf; hold on; axis equal; grid on;
    xlim([-L-0.1, L+0.1]); ylim([-0.15, 0.15]);
    
    % Track
    plot([-L L],[0 0],'k','LineWidth',2);
    
    % End barriers
    plot([L L],[-0.05 0.05],'r','LineWidth',3);
    plot([-L -L],[-0.05 0.05],'r','LineWidth',3);
    
    % Cart
    rectangle('Position',[x_traj(t)-cart_width/2,-cart_height/2,cart_width,cart_height],...
              'FaceColor','b');
    
    % Acceleration vector (scaled)
    quiver(x_traj(t),0.06,0,a_traj(t)*0.05,'r','LineWidth',2,'MaxHeadSize',2);
    
    % Live values
    text(x_traj(t),0.1,sprintf('x = %.3f m',x_traj(t)),'HorizontalAlignment','center','FontSize',10);
    text(x_traj(t),0.08,sprintf('a = %.2f m/s^2',a_traj(t)),'HorizontalAlignment','center','FontSize',10,'Color','r');
    
    title(sprintf('Cart Simulation: t = %.2f s', t*dt));
    xlabel('Position x (m)');
    ylabel('Track');
    pause(0.01);
end

%% Plot acceleration vs position
figure;
plot(x_traj,a_traj,'LineWidth',2);
xlabel('Position x (m)');
ylabel('Acceleration a (m/s^2)');
title('Acceleration vs Position (Full Simulation)');
grid on;
