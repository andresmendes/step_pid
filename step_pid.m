function [y,x] = step_pid(varargin)
    % step_pid - Simulation of a PID controlled system for a reference step
    % input. Input arguments are process dynamics and PID parameters.
    % Output parameters are step reponse data and state trajectories.
    %
    % This function is an extension of the conventional step function
    % designed to simulate closed loop systems. Process dynamics defined
    % using funtion handle, state space and transfer function are accepted.
    % The controlled variable must be the first state.
    %
    % Syntax
    %
    % [y,x] = step_pid(sys,t,Kp,Ki,Kd,N,R)
    %
    % [y,x] = step_pid(sys,t,Kp,Ki,Kd,N,R,S)
    %
    % Input Arguments:
    %
    % sys - Function handle, state space of transfer function.
    %
    % t - time vector
    %
    % Kp, Ki, Kd - Controller gains
    %
    % N - Derivative filter coefficient
    %
    % R - Step amplitude
    %
    % S - Number of state variables (Only for sys=function handle)
    %
    % Output Arguments:
    %
    % y - Step response data
    %
    % x - State trajectories
    

    sys = varargin{1};
    t   = varargin{2};
    Kp  = varargin{3};
    Ki  = varargin{4};
    Kd  = varargin{5};
    N   = varargin{6};
    R   = varargin{7};

    % Case: function handle 
    if isa(sys,'function_handle')

        S   = varargin{8}; % Number of states
        % Simulation initial conditions
        z_c_0   = [0 0];        % Controller dynamics initial conditions
        z0      = [z_c_0 zeros(1,S)];
        [t,x]   = ode45(@(t,z) simulation(t,z,sys,Kp,Ki,Kd,N,R),t,z0);

        % Controlled variable (first state after PID dynamics)
        y = x(:,3);
    end
    
    % Case: transfer function
    if isa(sys,'tf')
        C = pid(Kp,Ki,Kd,1/N);
        T = feedback(sys*C,1);
        [y,~,x] = step(R*T,t);
    end
    
    % Case: state space
    if isa(sys,'ss')
        C = pid(Kp,Ki,Kd,1/N);
        [num,den] = ss2tf(sys.a,sys.b,sys.c,sys.d);
        P = tf(num,den);
        T = feedback(P*C,1);
        [y,~,x] = step(R*T,t);
    end
end

function dz = simulation(t,z,sys,Kp,Ki,Kd,N,R)
    % simulation - Simulation of controller and process dynamics.
    %
    % t - time vector
    % 
    % z - States: z1 and z2 are the controller states, z3 is the controlled
    % variable, and so on.
    %
    % sys - Function handle, state space of transfer function.
    %
    % Kp, Ki, Kd - Controller gains
    %
    % N - Derivative filter coefficient
    %
    % R - Step amplitude
    
    e = R - z(3);           % Error

    % Controller derivatives and output
    [dz_c,u]    = pid_controller(z(1:2),e,Kp,Ki,Kd,N);
    % Process derivatives
    dz_sys      = sys(t,z(3:end),u);
    % Controller and process derivatives
    dz          = [dz_c ; dz_sys];
end

function [dz,u] = pid_controller(z,e,Kp,Ki,Kd,N)
    % pid_controller - Controller dynamics and output.
    
    % Constants
    C1 = (Kp/N + Kd);
    C2 = Kp + Ki/N;
    C3 = Ki;

    % State space matrices
    A = [0 1 ; 0 -N];
    B = [0 ; N];
    C = [C3 (C2-C1*N)];
    D = C1*N;

    % State equation
    dz  = A*z + B*e;
    % Output equation
    u   = C*z + D*e;
end
