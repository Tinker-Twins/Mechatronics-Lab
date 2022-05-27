%% Experiment 1 - Pendulum Over a Cart
% Tanmay Alias Manjeet Vilas Samak (RA1711018010101)

% Initialization
clc;
clear all;
close all;

% System Parameters
M = 0.5; % Mass of Cart (kg)
m = 0.2; % Mass of Pendulum (kg)
b = 0.1; % Friction/Damping Coefficient (N/m/s)
g = 9.8; % Gravity (m/s^2)
l = 0.3; % Length of Pendulum (m)
I = 0.006; % Inertia (kg.m^2)

%% Task 1 - System Representation

% Transfer Function Representation

s = tf('s');
q = (M+m)*(I+m*l^2)-(m*l)^2;
P_cart = (((I+m*l^2)/q)*s^2-(m*g*l/q))/(s^4+(b*(I+m*l^2))*s^3/q-((M+m)*m*g*l)*s^2/q-b*m*g*l*s/q);
P_pend = (m*l*s/q)/(s^3+(b*(I+m*l^2))*s^2/q-((M+m)*m*g*l)*s/q-b*m*g*l/q);
sys_tf = [P_cart;P_pend];
inputs = {'u'};
outputs = {'x';'phi'};
set(sys_tf,'InputName',inputs)
set(sys_tf,'OutputName',outputs)
sys_tf

% State Space Representation

p = I*(M+m)+M*m*l^2;

A = [0 1              0             0;
     0 -(I+m*l^2)*b/p (m^2*g*l^2)/p 0;
     0 0              0             1;
     0 -(m*l*b)/p     m*g*l*(M+m)/p 0];

B = [0;
     (I+m*l^2)/p;
     0;
     m*l/p];

C = [1 0 0 0;
     0 0 1 0];

D = [0;
     0];

states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'u'};
outputs = {'x';'phi'};
sys_ss = ss(A,B,C,D,'statename',states,'InputName',inputs,'OutputName',outputs)

%% Task 2 - Open Loop System Response

t = 0:0.01:10;
figure
impulse(sys_tf,t) % Impulse Response
figure
step(sys_tf,t) % Step Response

%% Task 3 - PID Controller

Kp = input('Enter Kp Value: '); % 100
Ki = input('Enter Ki Value: '); % 1
Kd = input('Enter Kd Value: '); % 20
ctrl = pid(Kp,Ki,Kd);
sys = feedback(P_pend,ctrl)
figure
impulse(sys) % Impulse Response