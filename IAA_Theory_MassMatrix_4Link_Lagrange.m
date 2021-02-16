%% IAA_Theory_MassMatrix_3Link_Lagrange.m
% Written by Sarah A Schloemer 09/18/16
% Last Updated by Sarah A Roelker 01/16/21

clc; clear all; close all;

%% Define Symbolic Terms
syms m0 m1 m2 m3 theta0 theta1 theta2 theta3;
syms Ixx0 Iyy0 Izz0 Ixx1 Iyy1 Izz1 Ixx2 Iyy2 Izz2 Ixx3 Iyy3 Izz3 r0 r1 r2 r3 L0 L1 L2 L3;

% segments:
    % 0 = foot
    % 1 = shank
    % 2 = thigh
    % 3 = foot
%% Generalized Coordinates

% Link 3
r_G3_A = [L1*cos(theta1) + L2*cos(theta1 + theta2) + r3*cos(theta1 + theta2 + theta3); L1*sin(theta1) + L2*sin(theta1 + theta2) + r3*sin(theta1 + theta2 + theta3)];
J_v3 = jacobian(r_G3_A, [theta0; theta1; theta2; theta3]);
J_v3_T = transpose(J_v3);
J_omega3 = [0 0 0 0; 0 0 0 0; 0 0 0 0];
J_omega3_T = transpose(J_omega3);
I_3 = [Ixx3 0 0; 0 Iyy3 0; 0 0 Izz3];

% Link 2
r_G2_A = [L1*cos(theta1) + r2*cos(theta1 + theta2); L1*sin(theta1) + r2*sin(theta1 + theta2)];
J_v2 = jacobian(r_G2_A, [theta0; theta1; theta2; theta3]);
J_v2_T = transpose(J_v2);
J_omega2 = [0 0 0 0; 0 0 0 0; 0 0 0 0];
J_omega2_T = transpose(J_omega2);
I_2 = [Ixx2 0 0; 0 Iyy2 0; 0 0 Izz2];

% Link 1
r_G1_A = [r1*cos(theta1); r1*sin(theta1)];
J_v1 = jacobian(r_G1_A, [theta0; theta1; theta2; theta3]);
J_v1_T = transpose(J_v1);
J_omega1 = [0 0 0 0; 0 0 0 0; 0 0 0 0];
J_omega1_T = transpose(J_omega1);
I_1 = [Ixx1 0 0; 0 Iyy1 0; 0 0 Izz1];

% Link 0
r_G0_A = [r0*sin(theta0); r0*cos(theta0)];
J_v0 = jacobian(r_G0_A, [theta0; theta1; theta2; theta3]);
J_v0_T = transpose(J_v0);
J_omega0 = [0 0 0 0; 0 0 0 0; 0 0 0 0];
J_omega0_T = transpose(J_omega0);
I_0 = [Ixx0 0 0; 0 Iyy0 0; 0 0 Izz0];
%% Equations of Motion

M = simplify((m0*J_v0_T*J_v0) + (J_omega0_T*I_0*J_omega0) + (m1*J_v1_T*J_v1) + (J_omega1_T*I_1*J_omega1) + (m2*J_v2_T*J_v2) + (J_omega2_T*I_2*J_omega2) + (m3*J_v3_T*J_v3) + (J_omega3_T*I_3*J_omega3))


