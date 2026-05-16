clear; clc; 

% =========================================================================
% 1. ROBOT CAR AND TRACK PARAMETERS
% =========================================================================
rR = 0.0325;         % Radius of Right wheel (meters)
rL = 0.0325;         % Radius of Left wheel (meters)
b = 0.1;           % Half-width of the robot (meters)
L = 1;            % Length of straight segment (meters)
R_turn = 0.5;       % Radius of the U-Turn (meters)
v = 0.3;            % Target forward velocity (m/s) , 
% v >> It's based on the operational RPM (around 50 RPM) of the motor u will choose by Arduino Uno controller  

% =========================================================================
% 2. TIMING AND COMMAND SIGNALS
% =========================================================================
% Calculate how long each segment takes based on speed
t_straight = L / v;
t_turn = (pi * R_turn) / v; % Half of Circumference 
t_lap = (2 * t_straight) + (2 * t_turn); % For one lap

% Use a very small time step (0.005s) to eliminate integration drift
dt = 0.005;
t = 0:dt:(2 * t_lap); % Run for exactly 2 full laps

% Calculate required wheel angular velocities (rad/s)
phidot_s  = v / rR;                          % Both wheels straight driving angular velocities
omega     = v / R_turn;                       % Robot yaw rate during U-turns
phidot_tR = (v + (omega * b)) / rR;          % Right (outer) wheel during left & right turn
phidot_tL = (v - (omega * b)) / rL;          % Left  (inner) wheel during left & right turn

% Pre-allocatation 
phidotR = zeros(1, length(t));
phidotL = zeros(1, length(t));

% Fill the arrays with commands based on the exact lap time sequence
for i = 1:length(t)
    time_in_lap = mod(t(i), t_lap); % Loops the timer every lap

    if time_in_lap < t_straight
        % Segment 1: Bottom Straight
        phidotR(i) = phidot_s;  phidotL(i) = phidot_s;
    elseif time_in_lap < (t_straight + t_turn)
        % Segment 2: First U-Turn
        phidotR(i) = phidot_tR; phidotL(i) = phidot_tL;
    elseif time_in_lap < ((2 * t_straight) + t_turn)
        % Segment 3: Top Straight
        phidotR(i) = phidot_s;  phidotL(i) = phidot_s;
    else
        % Segment 4: Second U-Turn
        phidotR(i) = phidot_tR; phidotL(i) = phidot_tL;
    end
end

% =========================================================================
% 3. KINEMATICS INTEGRATION
% =========================================================================
% Pre-allocate pose variables
x     = zeros(1, length(t));
y     = zeros(1, length(t));
theta = zeros(1, length(t));

% Jacobain Matrix: accumulate small pose increments at each time step
for ii = 2:length(t)
    J = [rR*cos(theta(ii-1))/2,  rL*cos(theta(ii-1))/2; ...
         rR*sin(theta(ii-1))/2,  rL*sin(theta(ii-1))/2; ...
         0.5*rR/b,              -0.5*rL/b];

    deltapose = dt * J * [phidotR(ii-1); phidotL(ii-1)];

    x(ii)     = x(ii-1)     + deltapose(1);
    y(ii)     = y(ii-1)     + deltapose(2);
    theta(ii) = theta(ii-1) + deltapose(3);
end
%%
% =========================================================================
% 4. VISUALIZATION & ANIMATION _ Extra part
% =========================================================================
figure('Name', 'Simple Robot Car Animation', 'Position', [100, 100, 800, 500]);
hold on; axis equal; grid on;
title('Differential Drive Robot');
xlabel('X (meters)'); ylabel('Y (meters)');

% Draw the track
ang = linspace(-pi/2, pi/2, 50);
plot([0, L], [0, 0],               'k--', 'LineWidth', 2); % Bottom straight
plot([0, L], [2*R_turn, 2*R_turn], 'k--', 'LineWidth', 2); % Top straight
plot(L + R_turn*cos(ang),  R_turn + R_turn*sin(ang), 'k--', 'LineWidth', 2); % Right curve
plot(0 - R_turn*cos(ang),  R_turn + R_turn*sin(ang), 'k--', 'LineWidth', 2); % Left curve

% Define the shape of the robot in its local (body) frame
chassis_x = [-0.1,  0.1,  0.1, -0.1];
chassis_y = [-0.1, -0.1,  0.1,  0.1];
wheel_x   = [-0.08, 0.08, 0.08, -0.08];
wheelR_y  = [-0.17, -0.17, -0.11, -0.11]; % Right wheel
wheelL_y  = [ 0.11,  0.11,  0.17,  0.17]; % Left wheel

% Create graphic objects (empty at first, updated each animation frame)
h_chassis = patch('XData', [], 'YData', [], 'FaceColor', [0.7 0.7 0.7], 'EdgeColor', 'k');
h_wheelR  = patch('XData', [], 'YData', [], 'FaceColor', 'k');
h_wheelL  = patch('XData', [], 'YData', [], 'FaceColor', 'k');
h_nose    = plot(0, 0, 'r-', 'LineWidth', 3); % Red arrow shows heading direction

% Animation loop (every 10th frame to run at approximately real time)
for ii = 1:10:length(t)

    % 2D rotation matrix for current heading angle
    th    = theta(ii);
    R_mat = [cos(th), -sin(th); sin(th), cos(th)];

    % Rotate all car parts from local frame to world frame
    rot_chassis = R_mat * [chassis_x; chassis_y];
    rot_wheelR  = R_mat * [wheel_x;   wheelR_y];
    rot_wheelL  = R_mat * [wheel_x;   wheelL_y];

    % Translate rotated shapes to the robot's current world position
    set(h_chassis, 'XData', rot_chassis(1,:) + x(ii), 'YData', rot_chassis(2,:) + y(ii));
    set(h_wheelR,  'XData', rot_wheelR(1,:)  + x(ii), 'YData', rot_wheelR(2,:)  + y(ii));
    set(h_wheelL,  'XData', rot_wheelL(1,:)  + x(ii), 'YData', rot_wheelL(2,:)  + y(ii));

    % Update heading arrow (15 cm forward from robot centre)
    set(h_nose, 'XData', [x(ii), x(ii) + 0.15*cos(th)], ...
                'YData', [y(ii), y(ii) + 0.15*sin(th)]);

    drawnow;
end

fprintf('Finished simulating exactly %d points.\n', length(t));