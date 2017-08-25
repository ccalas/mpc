%%%%%%%%%%%%% - USV autonomous control algorithm using MPC - %%%%%%%%%%%%%%
%%% This code use and MPC algorithm to track an angle reference while   %%% 
%%% keeping a constant cruise speed.                                    %%% 
%%% Written by Colin Calas at Cardiff University on August 2017         %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% TODO :
%%%     - Test with a testbench path

%%%%%%%%%%% - Clean the workspace and close the open figures - %%%%%%%%%%%%
clear
%close all

%%%%%%%%%%%%%%%%%%% - Boat and simulation parameters - %%%%%%%%%%%%%%%%%%%%
m = 37;         %Mass of the boat
D = 0.7;        %Distance between the motors and the center of mass
I = 0.1;        %Moment of inertia     (arbitrary value, should be identified)
k = 0.1;        %Viscosity coefficient (arbitrary value, should be identified)
Tfinal = 150;   %Total simulation time
Te = 0.1;       %Sampling period
V_cruise = 2;   %Cruise velocity   
i = 1;          %Loop index
l = 1;          %Boat width
h = 1.5;        %Boat height

%%%%%%%%%%%%%%% - Vectors used to hold simulation data - %%%%%%%%%%%%%%%%%%
x = zeros(9, ceil(Tfinal/Te));          %State vector for angle reference
u = zeros(2, ceil(Tfinal/Te));          %Input vector
delta_u = zeros(2, ceil(Tfinal/Te));    %Input increment vector
a = zeros(5, ceil(Tfinal/Te));          %State vector for MPC
a_ref = zeros(1, ceil(Tfinal/Te));      %Angle reference


%%%%%%%%%%%%%%%%%%%%%%%%% - Mission planning - %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%x_list = [2 4 32 40 25 10 2]';      %X coordinates of the waypoints
%y_list = [2 15 17 7 0 -5 2]';       %Y coordinates of the waypoints
x_list = [2 2 7 7 12 12 17 17 22 22]';
y_list = [2 22 22 2 2 22 22 2 2 22]';
nb_obj = size(x_list,1);            %Number of objectives
r_list = [x_list y_list];           %Objective list containing X and Y data
current_obj = 2;                    %As the boat starts in the first 
                                    %waypoint, the current objective is 2


%%%%%%%%%%%%%%% - State-space system used for the MPC - %%%%%%%%%%%%%%%%%%%
%Initial conditions
a(:,i) = [0 0 1.4181 0 0]';     %Initial state vector
u(:,i) = [0 0]';                %Initial input vector
delta_u(:,i) = [0 0]';          %Initial input increment vector

%System matrices
A = [1  0 0 0 0;...             %State matrix
     Te 1 0 0 0;...
     0 Te 1 0 0;...
     0 0  0 1 Te;...
     0 0  0 -k/m 1];
B = [D/(2*I) -D/(2*I);...       %Input matrix
     0 0; 0 0; 0 0; 1/m 1/m];       

n_input = size(B,2);            %Number of inputs
n_state = size(B,1);            %Number of state variables

%Augmented system matrices
A_tilde = [A B; zeros(n_input,n_state), eye(n_input)];  %State matrix
B_tilde = [zeros(n_state,n_input); eye(n_input)];       %Input matrix
C_tilde = [0 0 1 0 0 0 0;...                            %Output matrix
           0 0 0 1 0 0 0];                         
n_output = size(C_tilde,1);                             %Number of outputs

%%%%% - State-space system used to provide an angle reference vector - %%%%
x(:,i) = [2 2 1.4181 0 0 0 0 0 0]';     %Initial state vector

%Linearized state matrix
Fx = [1 0 0 0 0 Te*cos(x(3,i)) 0 -Te*sin(x(3,i)) 0;...
      0 1 0 0 0 Te*sin(x(3,i)) 0 Te*cos(x(3,i)) 0;...
      0 0 1 Te 0 0 0 0 0;...
      0 0 0 1 Te 0 0 0 0;...
      0 0 0 0 1  0 0 0 0;...
      0 0 0 0 0  1 Te 0 0;...
      0 0 0 0 0 -k/m 1 0 0;...
      0 0 0 0 0 0 0 1 Te;...
      0 0 0 0 0 0 0 -k/m 1];
%Linearized input matrix  
Fu = [0 0; 0 0; 0 0; 0 0;...                         
      D/(2*I) -D/(2*I); 0 0; 1/m 1/m; 0 0; 0 0];

%%%%%%%%%%%%%%%%%%%%%% - Disturbances modeling - %%%%%%%%%%%%%%%%%%%%%%%%%%
Dx = -0.02;                         %Disturbance surge on the X axis
Dy = 0.01;                          %Disturbance surge on the Y axis
alpha = angle(complex(Dx,Dy));      %Angle of the disturbance vector
Dk = [0 0 0 0 0 0 ...               %Disturbance vector
     (1/m)*Dx*cos(alpha-x(3,i)) 0 (1/m)*Dy*sin(alpha-x(3,i))]';

%%%%%%%%%%%%%%%%%%%%%%%%%% - MPC parameters - %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nu = 2;                                         %Control horizon
ny = 30;                                        %Prediction horizon
r = zeros(n_output*ny,1);                       %Angle reference vector

%Matrices used for optimization
gainS = computeGainS(n_output, 1);              %Steady-state gain
gainQ = computeGainQ(n_output, [2 0;0 1]);              %Outputs running cost gain
gainR = computeGainR(n_input, 1);               %Inputs running cost gain
Qbar = computeQbar(C_tilde, gainQ, gainS, ny);
Tbar = computeTbar(gainQ, C_tilde, gainS, ny);
Rbar = computeRbar(gainR, nu);
Cbar = computeCbar(A_tilde, B_tilde, ny, nu);
Abar = computeAbar(A_tilde, ny);
Hbar = computeHbar(Cbar, Qbar, Rbar);
Fbar = computeFbar(Abar, Qbar, Cbar, Tbar);

%State constraints vectors
%[theta_dot_dot theta_dot theta x_dot x_dot_dot U1 U2]
Cmin = [-100 -100 -2*pi 0 -10000 -1000*k/2 -1000*k/2]';
Cmax = [100 100 2*pi V_cruise 10000 1000*k/2 1000*k/2]';

%Quadprog solver options
%Keep the solver quiet
options = optimoptions('quadprog');
options = optimoptions(options, 'Display', 'off');

%Simulation loop
for t=0:Te:(Tfinal-1)
    a_tilde = [a(:,i); u(:,i)];             %Augmented state vector for MPC
    
    %Compute the ny next angle objectives
    tmp = x(:,i);           %Get the current state
    obj = current_obj;      %Get the current objective
    j = 1;
    while(j <= n_output*ny)
        %Compute the angle of the line between the boat and the waypoint
        goal_angle = angle(complex(r_list(obj,1)-tmp(1,1), r_list(obj,2)-tmp(2,1)));
        
        %If the distance between the goal angle and the current angle is
        %greater than pi, it means that there is a shorter path to this 
        %angle than getting all the way around the unit circle
        dist = abs(goal_angle - tmp(3,1));
        if dist > pi
            if tmp(3,1) < 0
                goal_angle = tmp(3,1) - (2*pi - dist);
            else
                goal_angle = tmp(3,1) + (2*pi - dist);
            end
        end
        
        %Considering an optimal orientation of the boat and equal inputs
        %compute the Ny next boat position.
        %If the boat is not yet arrived to the waypoint put the goal_angle 
        %into the reference vector, else recompute the goal angle.
        tmp(3,1) = goal_angle;
        
        %Make sure we're using the right state and disturbance matrices
        Fx = [1 0 0 0 0 Te*cos(tmp(3,1)) 0 -Te*sin(tmp(3,1)) 0;...
              0 1 0 0 0 Te*sin(tmp(3,1)) 0 Te*cos(tmp(3,1)) 0;...
              0 0 1 Te 0 0 0 0 0;...
              0 0 0 1 Te 0 0 0 0;...
              0 0 0 0 1  0 0 0 0;...
              0 0 0 0 0  1 Te 0 0;...
              0 0 0 0 0 -k/m 1 0 0;...
              0 0 0 0 0 0 0 1 Te;...
              0 0 0 0 0 0 0 -k/m 1];
        Dk = [0 0 0 0 0 0 (1/m)*Dx*cos(alpha-tmp(3,1)) ...
              0 (1/m)*Dy*sin(alpha-tmp(3,1))]';
        
        %Make a step forward
        tmp = Fx*tmp + Fu*(V_cruise*[k/2;k/2]) + Dk;
        
        %Have we reached the next waypoint ?
        if(tmp(1) > r_list(obj,1) - 0.5 && tmp(1) < r_list(obj,1) + 0.5 && ... 
           tmp(2) > r_list(obj,2) - 0.5 && tmp(2) < r_list(obj,2) + 0.5)
            %If yes, update the objective
            obj = mod(obj,7) + 1;
        end 
        
        r(j,1) = goal_angle;
        r(j+1,1) = V_cruise;

        j = j + n_output;
    end
    
    %Save the angle reference
    a_ref(1,i) = r(1,1);                
    
    %Constraints related matrices
    %Acons*a <= Bcons
    Acons = computeCy(ny,n_state+n_input)*Cbar;
    Bcons = computeDy(Cmin, Cmax, ny) ...
            - computeCy(ny,n_state+n_input)*Abar*a_tilde;
    
    %Solve the optimization problem, if impossible to solve : break and
    %display the results
    [delta_uFut, fval, exitflag] = quadprog(Hbar/2+Hbar'/2, ...
                ([a_tilde; r]'*Fbar)',Acons,Bcons,[],[],[],[],[],options);
    if(exitflag == -2)
        break;
    end
    
    delta_u(:,i) = delta_uFut([1 2],1);     %Get the first input increment
    u(:,i) = u(:,i) + delta_u(:,i);         %Compute the input
    
    %Make sure we're using the good Jacobian
    Fx = [1 0 0 0 0 Te*cos(x(3,i)) 0 -Te*sin(x(3,i)) 0;...
          0 1 0 0 0 Te*sin(x(3,i)) 0 Te*cos(x(3,i)) 0;...
          0 0 1 Te 0 0 0 0 0;...
          0 0 0 1 Te 0 0 0 0;...
          0 0 0 0 1  0 0 0 0;...
          0 0 0 0 0  1 Te 0 0;...
          0 0 0 0 0 -k/m 1 0 0;...
          0 0 0 0 0 0 0 1 Te;...
          0 0 0 0 0 0 0 -k/m 1];
    Dk = [0 0 0 0 0 0 (1/m)*Dx*cos(alpha-x(3,i)) ...
          0 (1/m)*Dy*sin(alpha-x(3,i))]';
    
    a(:,i+1) = A*a(:,i) + B*u(:,i) + Dk(3:7,1);     %Compute the next state for the MPC state-space
    x(:,i+1) = Fx*x(:,i)+ Fu*u(:,i) + Dk;           %Compute the next state for the position system
    
    i = i + 1;
    
    %Had we past the current waypoint
    if (x(1,i) < r_list(current_obj,1) + 1 && x(1,i) > r_list(current_obj,1) - 1) && ...
       (x(2,i) < r_list(current_obj,2) + 1 && x(2,i) > r_list(current_obj,2) - 1)
        
        %If the current objective was the last : good job, we're done
        %Break and display the results
        if current_obj == nb_obj
            break;
        end
        
        %Else increment the current_obj
        current_obj = current_obj + 1;
    end
    
end

%Plot the boat trajectory
subplot(4,3,[1 4]);
hold on
plot(x(1,1:i), x(2,1:i), 'k');

%Each 20 timesteps plot the boat at his current position
for j=1:i
   if mod(j,20) == 0
        line([x(1,j)-(h/2)*cos(x(3,j))+(l/2)*sin(x(3,j)) x(1,j)-(h/2)*cos(x(3,j))-(l/2)*sin(x(3,j))],...
             [x(2,j)-(h/2)*sin(x(3,j))-(l/2)*cos(x(3,j)) x(2,j)-(h/2)*sin(x(3,j))+(l/2)*cos(x(3,j))]);
        line([x(1,j)-(h/2)*cos(x(3,j))+(l/2)*sin(x(3,j)) x(1,j)+(h/2)*cos(x(3,j))],...
             [x(2,j)-(h/2)*sin(x(3,j))-(l/2)*cos(x(3,j)) x(2,j)+(h/2)*sin(x(3,j))]);
        line([x(1,j)-(h/2)*cos(x(3,j))-(l/2)*sin(x(3,j)) x(1,j)+(h/2)*cos(x(3,j))],...
             [x(2,j)-(h/2)*sin(x(3,j))+(l/2)*cos(x(3,j)) x(2,j)+(h/2)*sin(x(3,j))]);
   end
end

%Plot the waypoints circles of acceptance
theta = 0:0.01:2*pi;
rayon = 1;
X = rayon*cos(theta);
Y = rayon*sin(theta);
for j=1:nb_obj
    plot(X + r_list(j,1),Y + r_list(j,2), 'r')
end

%Plot the disturbance vector (amplified)
quiver(20,5,Dx,Dy,800, 'm-', 'LineWidth',1);
hold off
axis([min(x_list)-3 max(x_list)+3 min(y_list)-3 max(y_list)+3]);

%Plot the first input
subplot(4,3,2);
plot(1:i, u(1,1:i), 'b');

%Plot the second input
subplot(4,3,5);
plot(1:i, u(2,1:i), 'r');

%Plot the angle from the "x" state vector
subplot(4,3,[7 10]);
plot(1:i, x(3,1:i));

%Plot the x velocity
subplot(4,3,[3 6]);
plot(1:i, x(6,1:i));
hold on
plot(1:i, V_cruise*ones(1,i));
hold off
axis([0 i 0 V_cruise+0.5]);

%Plot the angle from the "a" state vector
subplot(4,3,[8 11]);
plot(1:i, a(3,1:i));
hold on
plot(1:i, a_ref(1,1:i));
hold off

%Plot the y velocity
subplot(4,3,[9 12]);
plot(1:i, x(8,1:i));