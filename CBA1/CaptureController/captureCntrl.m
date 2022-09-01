clc;
clear all
METR7203_PendulumPlant
%% State Space Feedback
%Use the linearised model to develop state space Feedback
C = eye(4);
D = zeros(4,1);
orig_plant = ss(A,B,C, D);
%Develop a statespace 
P = [-1, -2, -3, -4];
K1 = place(A,B, P);

plant1 = feedback(orig_plant, K1);

P =  2 * [-1, -2, -3, -4];
K2 = place(A,B, P);
plant2 = feedback(orig_plant, K2);

Q = [
    1, 0, 0, 0; %Penalize theta_3 error
    0, 1, 0, 0; %Penalize dTheta_3 error
    0, 0, 5, 0; %Penalize theta_4 eror
    0, 0, 0, 1]; %Penalize dtheta_4 error


R = 0.01; %Penalise the amount of torque applied to the controller

[K3, S, e] = lqr(A,B, Q, R);
sys3 = feedback(orig_plant, K3);
initial(sys3,[0,0,0.1,0])