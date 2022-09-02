clc;
clear all
METR7203_PendulumPlant
% %% State Space Feedback
m = 0.1580;
lcg = 0.2698;
rh = 0.37;
J1 = 0.1314;
Jz_bar = 0.0131;
g = 9.81;
c = 0.1;
k = 0.2;
Eo = 0;
Id = 0.0148;
dtheta1 = 400 * (2*pi/60);
%Use the linearised model to develop state space Feedback
C = eye(4);
D = zeros(4,1);
orig_plant = ss(A,B,C, D);
%Develop a statespace 
P = [-1, -2, -3, -4] *1.5;
K1 = place(A,B, P);

plant1 = feedback(orig_plant, K1); 

P =  1.25 * [-1, -2, -3, -4];
K2 = place(A,B, P);
plant2 = feedback(orig_plant, K2);

Q = [
    0.1, 0, 0, 0; %Penalize theta_3 error
    0, 0.1, 0, 0; %Penalize dTheta_3 error
    0, 0, 1, 0; %Penalize theta_4 eror
    0, 0, 0, 1]; %Penalize dtheta_4 error


R = 1; %Penalise the amount of torque applied to the controller

[K3, S, e] = lqr(A,B, Q, R);
sys3 = feedback(orig_plant, K3);
initial(sys3,[0,0,0.1,0])

Q2 = [
    0.1, 0, 0, 0; %Penalize theta_3 error
    0, 0.1, 0, 0; %Penalize dTheta_3 error
    0, 0, 0.1, 0; %Penalize theta_4 eror
    0, 0, 0, 0.1]; %Penalize dtheta_4 error
R2 = 1;

[K4, S,e] = lqr(A,B,Q2,R2);