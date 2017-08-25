%%%%%%%%%%%%% - USV autonomous control algorithm using MPC - %%%%%%%%%%%%%%
%%% This code use and MPC algorithm to track an angle reference while   %%% 
%%% keeping a constant cruise speed.                                    %%% 
%%% Written by Colin Calas at Cardiff University on August 2017         %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% TODO :
%%%     - Add the cruise velocity to the MPC
%%%     - Add disturbances
%%%     - Test with a testbench path

%Clean the workspace and close the open figures
clear
close all

%Boat and simulation parameters
m = 37;         %Mass of the boat
D = 0.7;        %Distance between the motors and the center of mass
I = 0.1;        %Moment of inertia      (arbitrary value, should be identified)
k = 0.1;        %Viscosity coefficient  (arbitrary value, should be identified)
Tfinal = 150;   %Total simulation time
Te = 0.1;       %Sampling period

%Vectors used to hold simulation data
x = zeros(7, ceil(Tfinal/Te));          %State vector
u = zeros(2, ceil(Tfinal/Te));          %Input vector
delta_u = zeros(2, ceil(Tfinal/Te));    %Input increment vector
a = zeros(3, ceil(Tfinal/Te));          %State vector
i = 1;                                  %Loop index



%Ordered list of waypoints
x_list = [2 4 32 40 25 10 2]';  %X coordinates of the waypoints
y_list = [2 15 17 7 0 -5 2]';   %Y coordinates of the waypoints
a_list = zeros(7,1);            %Angle of the boat between two successive waypoints
current_obj = 2;                %As the boat starts in the first waypoint, the current objective is the next
                                %ie. the second waypoint

%Compute all the angles between two successive waypoints
%The angles returned are between -pi and pi
for j=1:7
   a_list(mod(j,7)+1,1) = angle(complex(x_list(mod(j,7)+1)-x_list(j), y_list(mod(j,7)+1)-y_list(j)));
   
   if a_list(j,1) < 0
      a_list(j,1) = a_list(j,1);
   end
end

%Objectives list containing X,Y and Theta coordinates
r_list = [x_list y_list a_list];
nb_obj = size(r_list,1);                %Number of objectives

%MPC horizons
nu = 2;         %Control horizon (dof of the optimization problem)  
ny = 30;        %Prediction horizon

%State-space system used for the MPC
a(:,i) = [0 0 1.4181]';                 %Initial conditions : the boat is in the correct orientation
                                        %And has null angular speed and acceleration
u(:,i) = [k/2 k/2]';                    %Initial condition on the command : to maintain a speed of 1m/s
delta_u(:,i) = [0 0]';                  %Initial condition on the input increments

%System matrices
A = [1  0 0;...                         %State matrix
     Te 1 0;...
     0 Te 1];
B = [D/(2*I) -D/(2*I); 0 0; 0 0];       %Input matrix

%Augmented system matrices
A_tilde = [A B; zeros(2,3), eye(2)];    %State matrix
B_tilde = [zeros(3,2); eye(2)];         %Input matrix
C_tilde = [0 0 1 0 0];                  %Output matrix
n_output = size(C_tilde,1);                    %Dimension of the output
n_input = size(B,2);            %Number of inputs
n_state = size(B,1);            %Number of state variables

%State-space system used to provide an angle reference vector
x(:,i) = [2 2 1.4181 0 0 1 0]';         %State initial condition : the boat is in the first waypoint
                                        %and correctly oriented
Fx = [1 0 0 0 0 Te*cos(x(3,i)) 0;...    %Linearized state matrix
      0 1 0 0 0 Te*sin(x(3,i)) 0;...
      0 0 1 Te 0 0 0;...
      0 0 0 1 Te 0 0;...
      0 0 0 0 1  0 0;...
      0 0 0 0 0  1 Te;...
      0 0 0 0 0 -k/m 1];
Fu = [0 0; 0 0; 0 0; 0 0;...            %Linearized input matrix               
      D/(2*I) -D/(2*I); 0 0; 1/m 1/m];

r = zeros(n_output*ny,1);                      %Angle reference vector

%Matrices used for optimization
gainS = computeGainS(n_output, 1);             %Steady-state gain
gainQ = computeGainQ(n_output, 1);             %Running cost gain on the output
gainR = computeGainR(2, 1);             %Running cost gain on the input
Qbar = computeQbar(C_tilde, gainQ, gainS, ny);
Tbar = computeTbar(gainQ, C_tilde, gainS, ny);
Rbar = computeRbar(gainR, nu);
Cbar = computeCbar(A_tilde, B_tilde, ny, nu);
Abar = computeAbar(A_tilde, ny);
Hbar = computeHbar(Cbar, Qbar, Rbar);
Fbar = computeFbar(Abar, Qbar, Cbar, Tbar);

%Quadprog solver options
%Keep the solver quiet
options = optimoptions('quadprog');
options = optimoptions(options, 'Display', 'off');

for t=0:Te:(Tfinal-1)
    a_tilde = [a(:,i); u(:,i)]; %Build the augmented state space vector for MPC
    
    %Compute the ny next angle objectives
    tmp = x(:,i);           %Get the current state
    obj = current_obj;      %Get the current objective
    
    %Compute the angle of the line between the boat and the next waypoint
    goal_angle = angle(complex(r_list(obj,1)-tmp(1,1), r_list(obj,2)-tmp(2,1)));
    
    %If the distance between the goal angle and the current angle is
    %greater than pi, it means that there is a shorter path to this angle
    %than getting all the way around the unit circle
    dist = abs(goal_angle - tmp(3,1));
    if dist > pi
        if tmp(3,1) < 0
            goal_angle = tmp(3,1) - (2*pi - dist);
        else
            goal_angle = tmp(3,1) + (2*pi - dist);
        end
    end
    
    %Considering an optimal orientation of the boat and equals inputs
    %compute the Ny next boat position.
    %If the boat is not yet arrived to the waypoint put the goal_angle into
    %the reference vector, else recompute the goal angle.
    tmp(3,1) = goal_angle;
    Fx = [1 0 0 0 0 Te*cos(goal_angle) 0;...
          0 1 0 0 0 Te*sin(goal_angle) 0;...
          0 0 1 Te 0 0 0;...
          0 0 0 1 Te 0 0;...
          0 0 0 0 1  0 0;...
          0 0 0 0 0  1 Te;...
          0 0 0 0 0 -k/m 1];
    j = 1;
    while(j <= ny)
        tmp = Fx*tmp + Fu*[k/2;k/2];    %Make a step forward
        
        %Have we reached the next waypoint ?
        if(tmp(1) > r_list(obj,1)-0.5 && tmp(1) < r_list(obj,1)+0.5 && ... 
           tmp(2) > r_list(obj,2) - 0.5 && tmp(2) < r_list(obj,2) + 0.5)
            
            %If yes, recompute the goal angle and update the jacobian
            obj = mod(obj,7) + 1;
            goal_angle = angle(complex(r_list(obj,1)-tmp(1,1), r_list(obj,2)-tmp(2,1)));
            dist = abs(goal_angle - tmp(3,1));
            if dist > pi
                if tmp(3,1) < 0
                    goal_angle = tmp(3,1) - (2*pi - dist);
                else
                    goal_angle = tmp(3,1) + (2*pi - dist);
                end
            end
            
            tmp(3,1) = goal_angle;
            Fx = [1 0 0 0 0 Te*cos(goal_angle) 0;...
                  0 1 0 0 0 Te*sin(goal_angle) 0;...
                  0 0 1 Te 0 0 0;...
                  0 0 0 1 Te 0 0;...
                  0 0 0 0 1  0 0;...
                  0 0 0 0 0  1 Te;...
                  0 0 0 0 0 -k/m 1];
        end 
        r(j,1) = goal_angle;
        j = j + 1;
    end    
    
    %Constraints related matrices
    Acons = computeCy(ny,n_state+n_input)*Cbar;
    Bcons = computeDy([-100 -100 -2*pi 1*k/2 1*k/2]', [100 100 2*pi 1.2*k/2 1.2*k/2]', ny) - ...
                      computeCy(ny,n_state+n_input)*Abar*a_tilde;
    
    %Solve the optimization problem, if impossible to solve : break and
    %display the results
    [delta_uFut, fval, exitflag] = quadprog(Hbar/2+Hbar'/2,([a_tilde; r]'*Fbar)',Acons,Bcons,[],[],[],[],[],options);
    if(exitflag == -2)
        break;
    end
    
    delta_u(:,i) = delta_uFut([1 2],1);     %Get the input increment to apply
    u(:,i) = u(:,i) + delta_u(:,i);         %Compute the input
    a(:,i+1) = A*a(:,i) + B*u(:,i);         %Compute the next state for the MPC state-space
    x(3,i) = a(3,i);                        %Make the angles equal between both systems
    Fx = [1 0 0 0 0 Te*cos(x(3,i)) 0;...    %Make sure to use the right jacobian
          0 1 0 0 0 Te*sin(x(3,i)) 0;...
          0 0 1 Te 0 0 0;...
          0 0 0 1 Te 0 0;...
          0 0 0 0 1  0 0;...
          0 0 0 0 0  1 Te;...
          0 0 0 0 0 -k/m 1];
    x(:,i+1) = Fx*x(:,i)+ Fu*u(:,i);        %Compute the next state for the position system
    
    i = i + 1;
    
    %Had we past the current waypoint
    if (x(1,i) < r_list(current_obj,1) + 1 && x(1,i) > r_list(current_obj,1) - 1) && ...
       (x(2,i) < r_list(current_obj,2) + 1 && x(2,i) > r_list(current_obj,2) - 1)
        
        %If the current objective was the last : godd job, we're done
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
plot(x(1,1:i), x(2,1:i), 'k');

%Plot the waypoints circles of acceptance
hold on
theta = 0:0.01:2*pi;
rayon = 1;
X = rayon*cos(theta);
Y = rayon*sin(theta);
for j=1:7
    plot(X + r_list(j,1),Y + r_list(j,2), 'r')
end
hold off

%Plot the first input
subplot(4,3,2);
plot(1:i, u(1,1:i), 'b');
axis([0 round(i,-3) 0 0.08]);

%Plot the second input
subplot(4,3,5);
plot(1:i, u(2,1:i), 'r');
axis([0 round(i,-3) 0 0.08]);

%Plot the angle from the "x" state vector
subplot(4,3,[7 10]);
plot(1:i, x(3,1:i));

%Plot the velocity
subplot(4,3,[3 6]);
plot(1:i, x(6,1:i));
axis([0 round(i,-3) 0 1.5]);

%Plot the angle from the "a" state vector
subplot(4,3,[8 11]);
plot(1:i, a(3,1:i));
