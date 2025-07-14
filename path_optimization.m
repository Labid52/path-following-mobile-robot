clc;
clear;
close all;

%% Setup
xg = 10; yg = 12;
x = 0:0.5:xg;
y0 = 0;
L = 2;
N = length(x);

%% Optimization Variables
P1 = optimvar('P1','LowerBound',-inf);
P2 = optimvar('P2','LowerBound',-inf);
P3 = optimvar('P3','LowerBound',-inf);
P4 = optimvar('P4','LowerBound',-inf);

x0guess.P1 = rand();
x0guess.P2 = rand();
x0guess.P3 = rand();
x0guess.P4 = rand();

% y(x) as a cubic polynomial
y_expr = fcn2optimexpr(@(P1,P2,P3,P4) P1 + P2*x + P3*x.^2 + P4*x.^3, P1, P2, P3, P4);

% Finite differences
dx = diff(x);
dy_expr = y_expr(2:end) - y_expr(1:end-1);

% Slope
slope_expr = dy_expr ./ dx;

% Path length (objective)
path_len = sqrt(dx.^2 + dy_expr.^2);
obj = sum(path_len);
prob = optimproblem('Objective', obj);

%% Constraints
prob.Constraints.start_y = y_expr(1) == y0;
prob.Constraints.end_y = y_expr(end) == yg;
prob.Constraints.max_step = dy_expr <= 0.5;
prob.Constraints.min_step = dy_expr >= -0.5;
prob.Constraints.smoothness = slope_expr(2:end) - slope_expr(1:end-1) <= 2;  % limit 2nd derivative
prob.Constraints.reverse_smooth = slope_expr(2:end) - slope_expr(1:end-1) >= -2;

%% Solve
[Psol, fval] = solve(prob, x0guess);
P = struct2cell(Psol);

%% Postprocess & Plot
y_out = P{1} + P{2}*x + P{3}*x.^2 + P{4}*x.^3;
plot(x, y_out, 'b', 'LineWidth', 2); grid on;
title('Optimized Path without atan()'); xlabel('x'); ylabel('y');
