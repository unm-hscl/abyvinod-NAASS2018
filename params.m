%% Problem parameters
time_horizon=5;
mean_vector=zeros(4,1);
sigma_matrix = diag([1e-4, 1e-4, 5e-8, 5e-8]);    % sigma^2 values
%%
ymax=2;
slice_at_vx_vy = [0.0, 0.0];
no_of_gammas = 15;              % Total would be this + 2 (one for 0 and other for \infty)
initial_state = [0.75;
                 -0.75;
                 slice_at_vx_vy'];
% Sample in the triangle and then towards the top direction
label_cells={'Safe Set','Target Set','Algorithm 1'};

%% PS options
display_string = 'off';
myeps=1e-3;
mesh_tolerance_for_patternsearch = myeps*5e-2;
MaxFunEvals = 10000;
%constraint_tolerance_for_patternsearch = myeps*5e-2;

%% System dynamics: CWH dynamics --- Euler discretization of the continuous-time system 
%see  W. Wiesel, Spaceflight Dynamics. New York: McGraw-Hill, 1989.
% State of the system is [x,y,v_x,v_y]

T_dyn = 20;                         % sampling period in sec.
R = 850 + 6378.1;               % Orbital radius in m
G= 6.673e-11;                   % Universal gravitational constant                                           
M=5.9472e24;                    % Mass of the earth
mu = G*M/(1000^3);              % Gravitation constant for the pull of the earth
omega = sqrt(mu/(R^3));         % Angular velocity in the orbit

mc = 300;                       % Mass of the chief kg

tau = omega*T_dyn;                  
Btemp = [0 0; 0 0;1/mc 0; 0 1/mc];
system_matrix = [4 - 3*cos(tau), 0, sin(tau)/omega, (2/omega)*(1-cos(tau));
        6*(sin(tau) - tau), 1, -(2/omega)*(1-cos(tau)), (4*sin(tau)-3*tau)/omega;
      3*omega*sin(tau), 0, cos(tau), 2*sin(tau);
      -6*omega*(1-cos(tau)), 0, -2*sin(tau), 4*cos(tau)-3];
state_dimension = size(system_matrix,2);
B_int = @(t,~) [          4*t - 3*sin(omega*t)/omega, 0,               -cos(omega*t)/omega^2,            (2/omega)*(t - sin(omega*t)/omega);
                6*(sin(omega*t)/omega - omega*t^2/2), t, -(2/omega)*(t - sin(omega*t)/omega), (-4*cos(omega*t)/omega - 3*omega*t^2/2)/omega;
                                     -3*cos(omega*t), 0,                  sin(omega*t)/omega,                         -2*cos(omega*t)/omega;
                   -6*omega*(t - sin(omega*t)/omega), 0,                2*cos(omega*t)/omega,                    4*sin(omega*t)/omega - 3*t];

input_matrix = (B_int(T_dyn,omega) - B_int(0,omega))*Btemp;
disturbance_matrix = eye(state_dimension);

% System parameters
input_set = Polyhedron('lb', -[umax;umax], 'ub', [umax;umax]);
A_input_set = input_set.A;
b_input_set = input_set.b;
input_dimension = size(input_set.A,2);

% Safe Set --- LoS cone K
% |x|<=y and y\in[0,ymax] and |vx|<=vxmax and |vy|<=vymax
vxmax=0.5;
vymax=0.5;
A_safe_set = [1, 1, 0, 0;           
             -1, 1, 0, 0; 
              0, -1, 0, 0;
              0, 0, 1,0;
              0, 0,-1,0;
              0, 0, 0,1;
              0, 0, 0,-1];
b_safe_set = [0;
              0;
              ymax;
              vxmax;
              vxmax;
              vymax;
              vymax];
polytope_safe_set = Polyhedron(A_safe_set, b_safe_set);
minHRep(polytope_safe_set);

% Target set --- T
% Box centered [-0.1,0.1]x[-0.1,0]x[-0.01,0.01]x[-0.01,0.01]
% Creating the polyhedron using the upper and lower bounds
polytope_target_set = Polyhedron('lb', [-0.1; -0.1; -0.01; -0.01], 'ub', [0.1; 0; 0.01; 0.01]);
A_target_set = polytope_target_set.A;
b_target_set = polytope_target_set.b;

