[![View step_pid on File Exchange](https://www.mathworks.com/matlabcentral/images/matlab-file-exchange.svg)](https://www.mathworks.com/matlabcentral/fileexchange/92363-step_pid)
# step_pid
Simulation of a PID controlled system for a reference step input. Input arguments are process dynamics and PID parameters. Output parameters are step response data and state trajectories.

This function is an extension of the conventional step function designed to simulate closed loop systems. Process dynamics defined using function handle, state space model and transfer function are accepted. The controlled variable must be the first state.

**Syntax**

[y,x] = step_pid(sys,t,Kp,Ki,Kd,N,R)

[y,x] = step_pid(sys,t,Kp,Ki,Kd,N,R,S)

**Input Arguments:**

sys - Function handle, state space of transfer function.

t - time vector

Kp, Ki, Kd - Controller gains

N - Derivative filter coefficient

R - Step amplitude

S - Number of state variables (Only for sys=function handle)

**Output Arguments:**

y - Step response data

x - State trajectories

![Logo](https://www.mathworks.com/matlabcentral/mlc-downloads/downloads/ae7552db-7c8d-46a0-b666-cf12c76ad8c5/011070e9-a5a1-4a1d-a9c7-de938e2a7194/images/1621202794.png)

![Model](https://www.dropbox.com/s/lkdaruolprr1hvs/step_pid_model.png?raw=1)
