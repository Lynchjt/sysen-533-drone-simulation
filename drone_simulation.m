clear;

start_time = 0;
end_time = 10;
dt = 0.005;

times = start_time:dt:end_time;
% Number of points in the simulation.
N = numel(times);

pos = [5;5;5];
velocity = [0;0;0];
theta = [0;0;0];

length = 16;
drag_coefficient = 0;
thrust_coefficient = 200;
global_drag_coef = .05;

% I just made these numbers up. I think we need to look at the PD
% Controller portion to figure out real values
input1 = 10;
input2 = -10;
input3 = 10;
input4 = -10;

thetadot_val = 3*pi/8;
thetadot = [thetadot_val;thetadot_val;thetadot_val];

m = 16;
for t = times
    disp(t);
    % Take input from our controller.
    % These are where the inputs abouve are supposed to come from 
    %i = input(t);

    % I'm trying to find omega in the model using the integrator, but
    % idk if that is working
    I = [m*(pos(2)^2+pos(3)^2) 0 0;
         0 m*(pos(1)^2+pos(3)^2) 0;
         0 0 m*(pos(1)^2+pos(2)^2)];

    omega = thetadot2omega(thetadot, theta);

    % Compute linear and angular accelerations.
    % Haven't looked at this yet
    a = acceleration(input1,input2,input3,input4, theta, velocity, m, 9.8, thrust_coefficient, global_drag_coef);
    
    % Run simulation to calculate torque and angular acceleration
    result = sim('drone_model','CaptureErrors','on');
    
    % Why does the simulation return a long vector of angular
    % acceleration?????? Workaround to take the first 3x1 vector
%     omegadot = angular_acceleration(input1,input2,input3,input4,omega,I,length,drag_coefficient,thrust_coefficient);

    omegadot = result.angular_accel;
    
    omega = omega + dt * omegadot;

    thetadot = omega2thetadog(omega, theta);
    theta = theta + dt * thetadot;
    velocity = velocity + dt * a;
    pos = pos + dt * velocity;
    
    pos
end

function tau = torques(input1,input2,input3,input4, length, drag_coefficient, thrust_coefficient)
    % Inputs are values for ?i

    tau = [
    length * thrust_coefficient * (input1 - input3)

    length * thrust_coefficient * (input2 - input4)
    drag_coefficient * (input1 - input2 + input3 - input4)
    ];
end

function omegadot = angular_acceleration(input1,input2,input3,input4, omega, I, length, drag_coefficient, thrust_coefficient)
    tau = torques(input1,input2,input3,input4, length, drag_coefficient, thrust_coefficient);
    omegadot = inv(I) * (tau - cross(omega, I * omega));
end
function omega = thetadot2omega(thetadot, theta)  
rotation = [1 0 -sin(theta(2));
         0 cos(theta(1)) cos(theta(2))*sin(theta(1));
         0 -sin(theta(1)) cos(theta(2))*cos(theta(1))];

    omega = rotation * thetadot;
end

function thetadot = omega2thetadog(omega, theta)
rotation = [1 0 -sin(theta(2));
         0 cos(theta(1)) cos(theta(2))*sin(theta(1));
         0 -sin(theta(1)) cos(theta(2))*cos(theta(1))];
 
    thetadot = rotation^-1*omega;
end

function T = thrust(input1,input2,input3,input4, k)
    % Inputs are values for ?i

    T = [0; 0; k * (input1+input2+input3+input4)];
end

function R = RotationMatrix(angles)
cos_phi = cos(angles(1));
cos_omega = cos(angles(2));
cos_psi = cos(angles(3));

sin_phi = sin(angles(1));
sin_omega = sin(angles(2));
sin_psi = sin(angles(3));

R = [cos_phi*cos_psi-cos_omega*sin_phi*sin_psi -cos_psi*sin_phi-cos_phi*cos_omega*sin_psi sin_omega*sin_psi;
     cos_omega*cos_psi*sin_psi+cos_phi*sin_psi cos_phi*cos_omega*cos_psi-sin_phi*sin_psi -cos_psi*sin_omega;
     sin_phi*sin_omega cos_phi*sin_omega cos_omega];
end

function a = acceleration(input1,input2,input3,input4, angles, xdot, m, g, k, kd)
    gravity = [0; 0; -g];
    R = RotationMatrix(angles);
    T = R * thrust(input1,input2,input3,input4, k);
    Fd = -kd * xdot;
    a = gravity + 1 / m * T + Fd;
end
