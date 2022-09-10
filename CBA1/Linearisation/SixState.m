
% Init Variables
METR7203_PendulumPlant;
syms t theta theta_dot theta_ddot k m M g l p_ddot p_dot p gamma F c

% Define the Lagragian - 
L= 0.5 * M * p_dot^2 + 0.5 * m * (p_dot - l * theta_dot * cos(theta))^2 + 0.5 * m * (l * sin(theta) * theta_dot)^2 - m * g * l * cos(theta);

V = [p, p_dot, p_ddot, theta, theta_dot, theta_ddot];

% Generalized forces 
genfor = [F - c * p_dot, -gamma * theta_dot];
%genfor = subs(genfor, F, (M + m)*p_ddot - m*l*cos(theta)*theta_ddot+m*l*sin(theta)*theta_dot^2 + c*p_dot);
% Calculate the equations of motion 
eom=lagrange(L,V) - genfor; 

eom = simplify(eom);

eom = collect(eom, V);

S = solve(eom(1), eom(2),p_ddot, theta_ddot);

f = [p_dot, theta_dot, S.p_ddot, S.theta_ddot];
x = [p, theta, p_dot, theta_dot];

% Determine Jacobian
Jacx = jacobian(f,x);
Jacu = jacobian(f,F);

PBA2numericVars
Jacx_num = eval(Jacx);
Jacu_num = eval(Jacu);



