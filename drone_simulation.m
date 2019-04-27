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
input3 = 5;
input4 = 4;

thetadot_val = 3*pi/8;
thetadot = [thetadot_val;thetadot_val;thetadot_val]

m = 16;
for t = times
    % Take input from our controller.
    % These are where the inputs abouve are supposed to come from 
    %i = input(t);

    % I'm trying to find omega in the model using the integrator, but
    % idk if that is working
    I = [m*(pos(2)^2+pos(3)^2) 0 0;
         0 m*(pos(1)^2+pos(3)^2) 0;
         0 0 m*(pos(1)^2+pos(2)^2)];

    omega = thetadot2omega(thetadot, theta);
    a = sim('drone_model',200);
    a.angular_accel
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

function omega = thetadot2omega(thetadot, theta)
    rotation = [1 0 -sin(theta(2));
             0 cos(theta(1)) cos(theta(2))*sin(theta(1));
             0 -sin(theta(1)) cos(theta(2))*cos(theta(1))];
         
    omega = rotation * thetadot;
end

