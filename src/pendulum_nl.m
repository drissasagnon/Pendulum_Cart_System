function dx = pendulum_nl(x, u, params)

% State vector : x = [q1, q2, dq1, dq2]
q1 = x(1); q2 = x(2); dq1 = x(3); dq2 = x(4);
s = sin(q2); c = cos(q2);

% parameters
M = params.M; m = params.m; l = params.l;
k1 = params.k1; k2 = params.k2; g = params.g;

% Mass *[ddq1; ddq2] = rhs

Mass = [M+m,   -m*l*c;
        -m*l*c, m*l^2];
rhs = [u - m*l*s - k1*dq1;
       m*g*l*s - k2*dq2];

ddq = Mass\rhs;

dx = [dq1, dq2, ddq];
end