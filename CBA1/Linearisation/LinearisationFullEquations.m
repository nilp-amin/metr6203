
% Init Variables
METR7203_PendulumPlant;
m_num = m; % save the mass of the pendulum
syms t % Time 
% All states and their derivatives
syms theta_1 theta_dot_1 theta_ddot_1 
syms theta_2 theta_dot_2 theta_ddot_2 
syms theta_3 theta_dot_3 theta_ddot_3 
syms theta_4 theta_dot_4 theta_ddot_4 

%% Full equations of motion from Abbas et. al
% Torques (Tf terms are friction terms)
syms T_1 T_f1 T_2 T_f2 T_3 T_f3 T_4 T_f4
T_syms = [T_1 T_f1 T_2 T_f2 T_3 T_f3 T_4 T_f4];
% Inertias
syms I_A J_A K_A I_B J_B K_B I_C J_C K_C I_D J_D K_D
J_syms = [I_A J_A K_A I_B J_B K_B I_C J_C K_C I_D J_D K_D];
J_num = [IA JA KA IB JB KB IC JC KC ID JD KD];
syms m l g % Symbolic for pendulum mass and length (effective)
pend_syms = [m l g];
pend_num = [m_num, lcg, 9.81]; 
syms c_1 c_2 c_3 c_4
damp_syms = [c_1 c_2 c_3 c_4];
damp_num = [0.000187, 0.0118, 0.0027, 0.00001];

Jz_bar_num = Jz_bar;
syms Jz_bar J_vert
% J_eq*
T_3 = J_D*theta_dot_1*theta_dot_2;
F = T_3/Rh;
theta_ddot_3_largeOmega = theta_dot_1*theta_dot_2*J_D/J_vert;

eq1 = J_D*theta_ddot_1 == T_1 - theta_dot_1*c_1;
eq2 = T_2 - theta_dot_2*c_2 + J_D*theta_dot_1*theta_dot_3 - (I_C + I_D)*theta_ddot_2 == 0;
eq3 = J_D*theta_dot_1*theta_dot_2 + (I_D + K_A + K_B + K_C)*theta_ddot_3 + theta_dot_3*c_3 == 0;
eq4 = Jz_bar*theta_ddot_4 + m*l*cos(theta_4)*theta_ddot_3_largeOmega/Rh + m*g*l*sin(theta_4) - theta_dot_4*c_4 == 0;

S = solve([eq2, eq3, eq4], [theta_ddot_2, theta_ddot_3, theta_ddot_4]);
S.theta_ddot_1 = solve(eq1, theta_ddot_1)
S.theta_ddot_2 = solve(eq2, theta_ddot_2)
S.theta_ddot_3 = solve(eq3, theta_ddot_3)
S.theta_ddot_4 = solve(eq4, theta_ddot_4)

% six state
f = [theta_dot_2,  theta_dot_3, theta_dot_4, S.theta_ddot_2, S.theta_ddot_3, S.theta_ddot_4];
x = [theta_2,  theta_3, theta_4, theta_dot_2,  theta_dot_3, theta_dot_4];
% eight state
f = [theta_dot_1, theta_dot_2,  theta_dot_3, theta_dot_4, S.theta_ddot_1, S.theta_ddot_2, S.theta_ddot_3, S.theta_ddot_4];
x = [theta_1, theta_2,  theta_3, theta_4, theta_dot_1, theta_dot_2,  theta_dot_3, theta_dot_4];
% Four state
% f = [theta_dot_3, theta_dot_4,  S.theta_ddot_3, S.theta_ddot_4];
% x = [theta_3, theta_4, theta_dot_3, theta_dot_4];
f = [theta_dot_2, theta_dot_4,  S.theta_ddot_2, S.theta_ddot_4];
x = [theta_2, theta_4, theta_dot_2, theta_dot_4];

u = [T_1, T_2];

% Determine Jacobian
Jacx = jacobian(f,x);
Jacu = jacobian(f,u);

% Substitute numeric values
Omega = 400*2*pi/60;
Jacx_num = subs(Jacx, J_syms, J_num); % Inertia properties
Jacx_num = subs(Jacx_num, Jz_bar, Jz_bar_num); % Pendulum inertia
Jacx_num = subs(Jacx_num, J_vert, J1); % Effetive inertia about the vertical
Jacx_num = subs(Jacx_num, [m, l ,g], [m_num, lcg, 9.81]); % Pendulum stuff
Jacx_num = subs(Jacx_num, damp_syms, damp_num); % Damping
Jacx_num = subs(Jacx_num, theta_dot_1, Omega)

Jacu_num = subs(Jacu, J_syms, J_num);
Jacu_num = subs(Jacu_num, Jz_bar, Jz_bar_num);
Jacu_num = subs(Jacu_num, J_vert, J1);
Jacu_num = subs(Jacu_num, [m, l ,g], [m_num, lcg, 9.81]);
Jacu_num = subs(Jacu_num, damp_syms, damp_num);
Jacu_num = subs(Jacu_num, theta_dot_1, Omega)

% Now we can substitute our initial conditions for theta_4 (pendulum)
A_up = subs(Jacx_num, [theta_4, theta_dot_2, theta_dot_3], [0, 0, 0])
B_up = Jacu_num
C_up = [0 0 1 0 0 0]
C_up = [eye(length(x)/2), zeros(length(x)/2)]

if rank(ctrb(A_up, B_up)) == length(x)
    disp("System is controllable.")
else
    disp("System is not controllable")
    eigv = eig(A_up);
    for i = 1:length(eigv)
        lambda = eigv(i);
        r = rank([lambda*eye(length(x)) - A_up, B_up]);
        fprintf("Eigenvalue %0.5g + %0.5g j has rank %i.\n", real(lambda), imag(lambda), r)
    end
        
end

if rank(obsv(A_up, C_up)) == length(x)
    disp("System is observable.")
else
    disp("System is not observable")
    for i = 1:length(eigv)
        lambda = eigv(i);
        r = rank([lambda*eye(length(x)) - A_up; C_up]);
        fprintf("Eigenvalue %0.5g + %0.5g j has rank %i.\n", real(lambda), imag(lambda), r)
    end
end

die
% Full Kinetic energy
g = 9.81;
L = 0.5*ID*theta_dot_2*(theta_dot_2 - sin(theta_3)*theta_dot_3) + ...
    0.5*JD * theta_dot_1*(theta_dot_1 + cos(theta_2)*0 + sin(theta_2)*1*theta_dot_3) + ...
    0.5*ID * sin(theta_2) * 0 * (sin(theta_2) * 0 - cos(theta_2)*1*theta_dot_3) + ...
    0.5*JD*cos(theta_2)*0*(theta_dot_1 + cos(theta_2)*0 + sin(theta_2)*1*theta_dot_3) + ...
    0.5*JD*sin(theta_2)*1*theta_dot_3*(theta_dot_1 + cos(theta_2)*0 + sin(theta_2)*1*theta_dot_3) - ...
    0.5*ID*sin(theta_3)*theta_dot_3*(theta_dot_2 - sin(theta_3)*theta_dot_3) - ...
    0.5*ID*cos(theta_2)*1*theta_dot_3*(sin(theta_2)*0 - cos(theta_2)*cos(theta_3)*theta_dot_3) + ...
    0.5*m_num*(theta_dot_2*Rh - lcg*theta_dot_4*cos(theta_4))^2 + 0.5*m_num*(lcg*sin(theta_4)*theta_dot_4)^2 - m_num*g*lcg*cos(theta_4); %(subtract last as potential energy term)

V = [theta_1, theta_2, theta_3, theta_4; ...
     theta_dot_1, theta_dot_2, theta_dot_3, theta_dot_4;...
     theta_ddot_1, theta_ddot_2, theta_ddot_3, theta_ddot_4];
 V = reshape(V, 1, 12);

% Generalized forces 
genfor = [T_1 - c_1*theta_dot_1, T_2 - c_2*theta_dot_2, - c_3*theta_dot_3, - c_4*theta_dot_4];
% Calculate the equations of motion 
eom=lagrange(L,V) - subs(genfor, damp_syms, damp_num); 
eom = simplify(eom);
eom = collect(eom, V); 

SL.theta_ddot_1 = solve(eom(1), theta_ddot_1);
SL.theta_ddot_2 = solve(eom(2), theta_ddot_2);
SL.theta_ddot_3 = solve(eom(3), theta_ddot_3);
SL.theta_ddot_4 = solve(eom(4), theta_ddot_4);

SL = solve(eom(1), eom(2), eom(3), eom(4), theta_ddot_1, theta_ddot_2, theta_ddot_3, theta_ddot_4);

f = [theta_dot_1, theta_dot_2,  theta_dot_3, theta_dot_4, SL.theta_ddot_1, SL.theta_ddot_2, SL.theta_ddot_3, SL.theta_ddot_4];
x = [theta_1, theta_2,  theta_3, theta_4, theta_dot_1, theta_dot_2,  theta_dot_3, theta_dot_4];
% x = [theta_2,  theta_3, theta_4, theta_dot_1, theta_dot_2, theta_dot_4];
u = [T_1, T_2];

% Determine Jacobian
JacxL = jacobian(f,x);
JacuL = jacobian(f,u);

op_point = zeros(1, length(x));
A_up = double(subs(JacxL, [x T_1 T_2], [op_point 0 0]));
B_up = double(subs(JacxL, [x T_1 T_2], [op_point 0 0]));
C_up = [eye(length(x)/2), zeros(length(x)/2)];

if rank(ctrb(A_up, B_up)) == length(x)
    disp("System is controllable.")
else
    disp("System is not controllable")
    eigv = eig(A_up);
    for i = 1:length(eigv)
        lambda = real(eigv(i));
        r = rank([lambda*eye(length(x)) - A_up, B_up]);
        fprintf("Eigenvalue %0.5g + %0.5g j has rank %i.\n", real(lambda), imag(lambda), r)
    end
        
end

if rank(obsv(A_up, C_up)) == length(x)
    disp("System is observable.")
else
    disp("System is not observable")
    for i = 1:length(eigv)
        lambda = eigv(i);
        r = rank([lambda*eye(length(x)) - A_up; C_up]);
        fprintf("Eigenvalue %0.5g + %0.5g j has rank %i.\n", real(lambda), imag(lambda), r)
    end
end