% Variable initialization
close all;
clear all;
clc;
global a k_r1 k_r2 pi_m pi_l

% load manipulator dynamic parameters without load mass
  param;
  pi_l = pi_m;

% gravity acceleration
  g = 9.81;

% friction matrix
  K_r = [k_r1,0;0,k_r2];
  F_v = 0;%K_r*[0.001,0;0,0.001]*K_r;

%   
  Tc = 0.1;

% controller gains
  K_p = 0.25*eye(2);  
  K_d = 0.125*eye(2);
  
% desired position
  q_d = [-1.3587;-0.8967];
 
% initial position
  q_i(1) = 0;
  q_i(2)=-pi/2;

% duration of simulation
  t_d = 2.5;

% sample time for plots
  Ts = Tc;
  sim model
  plotsimulation;
  figure(3)
  subplot 211
  plot(t,u(:,1),'Linewidth',2);
  title('Control Torque1');
  xlabel('time/sec');ylabel('Control Torque/N.m');
  legend('motor1');
  subplot 212
  plot(t,u(:,2),'r','Linewidth',2);
  title('Control Torque2');
  xlabel('time/sec');ylabel('Control Torque/N.m');
  legend('motor2');
  figure(6)
  plot(0:0.1:5,gq,'Linewidth',2)
  title('g(q)');xlabel('time/s');ylabel('g(q)');
  legend('g(q) for motor1','g(q) for motor2');