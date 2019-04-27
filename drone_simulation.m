start_time = 0;
end_time = 10;
dt = 0.005;

times = start_time:dt:end_time;
% Number of points in the simulation.
N = numel(times);

pos = [0;0;0];
velocity = [0;0;0];
theta = [0;0;0];

length = 16;
drag_coefficient = 0;
thrust_coefficient = 200;

% I just made these numbers up. I think we need to look at the PD
% Controller portion to figure out real values
input1 = 1;
input2 = 2;
input3 = 3;
input4 = 4;

for t = times
    % Take input from our controller.
    % These are where the inputs abouve are supposed to come from 
    %i = input(t);

    % I'm trying to find omega in the model using the integrator, but
    % idk if that is working
    %omega = thetadot2omega(thetadot, theta);
    
    % Compute linear and angular accelerations.
    % Haven't looked at this yet
    %a = acceleration(i, theta, xdot, m, g, k, kd);
    
    % Trying to do these computations in the simulink model
    %omegadot = angular_acceleration(i, omega, I, L, b, k);

%     omega = omega + dt * omegadot;
%     thetadot = omega2thetadot(omega, theta);
%     theta = theta + dt * thetadot;
%     xdot = xdot + dt * a;
%     x = x + dt * xdot;
end
