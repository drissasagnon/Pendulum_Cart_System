%% Example trajectory generator for the Pendulum-Cart system -- WHEEL MODEL
% Realistic actuation: the motor torque u acts between the WHEEL and the
% chassis (which rigidly carries the pendulum arm), not directly on q1.
% Rolling-without-slipping constraint: q1 = R*phi  =>  qdot1 = R*phidot
%
% Virtual work of the motor torque (acting on wheel/chassis RELATIVE angle):
%   delta_W = u*(delta_phi - delta_q2) = u*(delta_q1/R - delta_q2)
%   => Q1 = u/R   (NEW: nonzero! breaks the momentum-conservation problem)
%      Q2 = -u
%
% M(q) and h(q,qdot) are UNCHANGED from the original derivation -- only
% gq changes from [0;1] to [1/R;-1].

clear; clc; close all;

%% System parameters
params.mc = 1.5;   % cart/wheel mass         [kg]
params.mp = 0.5;   % pendulum point mass     [kg]
params.L  = 1.0;   % pendulum arm length     [m]
params.g  = 9.82;  % gravity                 [m/s^2]
params.d1 = 0.01;  % cart damping (friction)
params.d2 = 0.01;  % joint damping (friction)
params.R  = 0.1;   % wheel radius            [m]  <-- NEW parameter
params.CartRadius = params.R;   % reuse for the animation's wheel drawing

%% Linearized model about q2* = 0 (needed to design K)
mc=params.mc; mp=params.mp; L=params.L; g=params.g;
d1=params.d1; d2=params.d2; R=params.R;

A = [0,        0,                1,       0;
     0,        0,                0,       1;
     0,        mp*g/mc,         -d1/mc,  -d2/(mc*L);
     0,        (mc+mp)*g/(mc*L),-d1/(mc*L),-(mc+mp)*d2/(mc*mp*L^2)];

Mq0 = [mc+mp, -L*mp; -L*mp, L^2*mp];
gq  = [1/R; -1];                 % <-- the key change vs the original model
B23 = Mq0 \ gq;
B   = [0; 0; B23];

fprintf('rang(ctrb(A,B)) = %d / 4 (devrait etre 4 : systeme entierement commandable)\n', ...
        rank(ctrb(A,B)));

%% LQR design (full 4-state system, no restriction needed anymore!)
Q = diag([10 50 1 1]);   % [q1, q2, q1dot, q2dot] weights -- adjust as desired
Rw = 1;
K = lqr(A, B, Q, Rw);
fprintf('K = '); disp(K);
fprintf('eig(A-B*K) = '); disp(eig(A-B*K).');

%% Initial conditions: cart at rest, pendulum tilted 10 deg from upright
x0 = [0; deg2rad(10); 0; 0];   % [q1; q2; q1dot; q2dot]

%% Control input: full state feedback (q1 now genuinely stabilizable!)
u_func = @(t, x) -K*x;

% For the open-loop (uncontrolled) case instead, use:
%   u_func = @(t, x) 0;

%% Simulate the nonlinear system
tspan = [0 10];
opts  = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);
[t, x] = ode45(@(t, x) pendulum_cart_wheel_ode(t, x, params, u_func), tspan, x0, opts);

q1 = x(:, 1);
q2 = x(:, 2);

%% Animate
animate_pendulum_cart(t, q1, q2, params, 'VideoName', 'pendulum_cart_wheel.mp4');

%% ---- local function: nonlinear equations of motion (wheel-driven) ----
function dx = pendulum_cart_wheel_ode(t, x, params, u_func)
    mc = params.mc; mp = params.mp; L = params.L;
    g  = params.g;  d1 = params.d1; d2 = params.d2; R = params.R;

    q1  = x(1); q2  = x(2);
    q1d = x(3); q2d = x(4);

    u = u_func(t, x);

    Mq = [ mc + mp,          -L*mp*cos(q2) ;
           -L*mp*cos(q2),     L^2*mp       ];

    h  = [  L*mp*q2d^2*sin(q2) + d1*q1d ;
           -L*g*mp*sin(q2)     + d2*q2d ];

    gq = [1/R; -1];    % NEW input mapping (wheel-driven, vs [0;1] before)

    qdd = Mq \ (gq*u - h);

    dx = [q1d; q2d; qdd(1); qdd(2)];
end