%% Example: step_pid 
% Example of step_pid function using function handle, transfer function and
% state space models.
%
%%

clear ; close all ; clc

%% Parameters
t   = 0:0.01:1;             % Time [s]

% Controller
Kp  = 80;                   % Proportional gain
Ki  = 100;                  % Integral gain
Kd  = 20;                   % Derivative gain
N   = 100;                  % Derivative filter coefficient
% Reference
amp = 2;                    % Step amplitude

%% step_pid using function handle

% Function handle
sys_fh = @model_dynamics;      
% Closed loop simulation
[y_fh,x_fh] = step_pid(sys_fh,t,Kp,Ki,Kd,N,amp,2);

%% step_pid using transfer function

% Transfer function model
s       = tf('s');
sys_tf  = 1/(s^2 + s + 1);
% Closed loop simulation
[y_tf,x_tf] = step_pid(sys_tf,t,Kp,Ki,Kd,N,amp);

%% step_pid using state space

% State space matrices
A   = [0 1 ; -1 -1];
B   = [0 ; 1];
C   = [1 0];
D   = 0;
% State space model
sys_ss = ss(A,B,C,D);
% Closed loop simulation
[y_ss,x_ss] = step_pid(sys_ss,t,Kp,Ki,Kd,N,amp);

%% Results

figure
hold on ; grid on ; box on
plot(t,y_fh,'b')
plot(t,y_tf,'r:','LineWidth',1.5)
plot(t,y_ss,'g--')
xlabel('Time [s]')
ylabel('Output Position [m]')
legend('Function Handle','Transfer Function','State Space','Location','SouthEast')

%% Auxiliary function

function dz = model_dynamics(~,z,u)
    % Example: second order dynamics
    dz(1,1) = z(2);
    dz(2,1) = -z(1) -z(2) + u;
end