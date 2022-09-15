METR7203_PendulumPlant
m = 0.1580;
lcg = 0.2698;
rh = 0.37;
J1 = 0.1314;
Jz_bar = 0.0131;
g = 9.81;
damp_num = [0.000187, 0.0118, 0.0027, 0.0002];
c2 = damp_num(2);
c3 = damp_num(3);
c4 = damp_num(4);
k = 0.2;
Eo = 0;
Id = 0.0148;
dtheta1 = 400 * (2*pi/60);

%Motor model
Jm = (ID + IC);
bm = 1;
Rm = 0.3;
Lm = 0.0824E-3;
kt = 2.5;

A_motor = double([-bm/Jm kt/Jm 0; -kt/Lm -Rm/Lm 0; 0 0 1]);
B_motor = double([0; 1/Lm; 0]);
C_motor = eye(3);

