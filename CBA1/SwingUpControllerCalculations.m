syms p_ddot t theta_2 theta_3_dot theta_2_ddot theta_2_dot theta_4 theta_4_dot theta_4_ddot

METR7203_PendulumPlant
Omega = 400 * 2*pi/60; % RPM to 
theta_3_ddot = p_ddot/Rh;

% Add some damping in around the base, can be set to zero
c = sym('c', 'positive');
J = sym('J', 'positive');
T = J*theta_3_ddot + c*theta_3_dot;

J_fw = sym("J_fw", 'positive'); % Equvalent to ID/KD in PendulumPlant
T_fw = sym("T_fw", 'real');
J_fw = ID;
J_eq = J1;
H = Omega * J_fw;
theta_3_ddot_largeOmega = Omega*theta_2_dot*J_fw^2/J_eq;
theta_3_ddot_fullEq = 1/J_eq*(Omega*theta_2_dot*J_fw^2 + ...(
    (T_fw +(J_fw^2 + 2*(JC-JD-KC))*theta_2_dot*theta_3_dot)*theta_2);

% Control Law
k = sym('k', 'positive');
g = 9.81;
l = lcg;
E = m*g*l*(cos(theta_4) - 1) + 1/2*m*l^2*theta_4_dot.^2;
E_0 = 0;
u = -k*(E-E_0).*theta_4_dot.*cos(theta_4);
sol = solve(theta_3_ddot == theta_3_ddot_largeOmega, theta_2_dot);
theta_2_dot = subs(sol, p_ddot, u);

% theta_2_dot now indicates the required speed fo rotation of the gimbal
% need (fast) gimbal controller to turn Voltage input into speed output. 
