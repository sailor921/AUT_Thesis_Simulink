close all 
clear 
clc

%%

% **In this simulation we assume that Leader is connected to agent 1 and leader is dynamic: moving from initial point to target point**

%Variables
%Simulation time
time_steps = 100;
tstep = 0.1;
time_sim = time_steps * tstep;

%Number of agents
N = 3;  

%Adjacency Matrix
A = [zeros(N-1,1) , eye(N-1); ones(1,1) , zeros(1, N-1)];

%Leader Variables
Pleader = zeros(2, 1, time_steps+1);%*
Thetaleader= zeros(1, 1, time_steps+1);%*
Vleader = [1; 1];
Omegaleader = 1;

%Initial Position of Leader
Pleader(:, 1, 1) = [1; 1];
Thetaleader(:, 1, 1) = 0;

%Goal Position
Pgoal = [10; 10];
Thetagoal = 0;

%Error for nearing target point
%Error = Pgoal - Pleader
Error = ones(2, 1);

%Position Vectors
P = zeros(2, N, time_steps+1);  %X and y array*
Theta = zeros(1, N, time_steps+1);  %theta array*

%Random inialization of position and orentation
alpha = 100;
radious_agent = sqrt(3)/6 * alpha;
P(:, :, 1) = radious_agent * (rand(2, N) - 0.5);
Theta(:,:,1) = alpha * rand(1,1);

%Velocity Vectos
%This sould not here because this is the output of functions
%P_dot = zeros(2, N, time_steps);  %Xdot and Ydot array
%Theta_dot = zeros(1, N, time_steps);  %thetadot array

%%
%Simulation

iteration = 1;

while norm(Error) >= 0.2

    %Controlles 
    [U, W] = controller(P(:,:,iteration), Theta(:,:,iteration), Pleader(:, :, iteration), Thetaleader(:, :, iteration), A);

    %Derivative variables, these are velocities of agents
    [P_dot, Theta_dot] = agents(P(:,:,iteration), Theta(:,:,iteration), U, W);

    %Update Position of agents
    P(:, :, iteration+1) = P(:, :, iteration) + tstep * P_dot;
    Theta(:, :, iteration+1) = Theta(:, :, iteration) + tstep * Theta_dot;

    %Update Position of leader
    Pleader(:, :, iteration+1) = Pleader(:, :, iteration) + tstep * Vleader;
    Thetaleader(:, :, iteration+1) = Thetaleader(:, :, iteration) + tstep * Omegaleader;
    
    %Update iteration
    iteration = iteration + 1;

    %Update Error to nearing target point
    Error = Pgoal - Pleader(:, :, iteration);

end

%%
%Plot

figure
hold on
grid on
grid minor

for k = 1:iteration

    for i = 1:N

        plot(P(1, i, k), P(2, i, k), 'ro')  %Plot variation position of agents 
        plot(Pleader(1, :, k), Pleader(2, :, k), 'b*')  %Plot variation position of leader
        axis([-12 12 -12 12])
        pause(0)

    end 

    pause(0)

end

figure
axis equal
axis([-12 12 -12 12])
plot(reshape(P(1,:,:),[N, time_steps+1]).', reshape(P(2,:,:),[N, time_steps+1]).')    %Plot position of agents*
hold on
plot(P(1, :, iteration), P(2, :, iteration), 'k^')    %Final position of agents
plot(P(1, :, 1), P(2, :, 1), 'ko')  %Initial position of agents
grid on 
grid minor
hold on
plot(Pleader(1, :, 1), Pleader(2, :, 1), "k*") %Final position of leader
plot(Pleader(1, :, iteration), Pleader(2, :, iteration), 'k*') %Inital position of leader

%%
%Functions

function [p_dot, theta_dot] = agent(p, theta, u, w)
    p_dot = u;
    theta_dot = w;  
end

function [P_dot, Theta_dot] = agents(P, Theta, U, W)
    N = size(P, 2);
    P_dot = zeros(size(P));
    Theta_dot = zeros(size(Theta));

    for i = 1:N

        [p_dot_tmp, theta_dot_tmp] = agent(P(:, i), Theta(:, i), U(:, i), W(:, i));
%         P_dot = [P_dot, p_dot_tmp];
        P_dot(:, i) = p_dot_tmp;
        Theta_dot(:, i) = theta_dot_tmp; 

    end

end

function [U, W] = controller(P, Theta, Pleader, Thetaleader, A)
    N = size(P, 2);
    U = zeros(size(P));
    W = zeros(size(Theta));

    for i = 1:N

        for j = 1:N

            %if (i == -1)
                U(:, i) = U(:, i) + A(i, j) * (P(:, j) - P(:, i) + Pleader - P(:, i));
                W(:, i) = W(:, i) + A(i, j) * (Theta(:, j) - Theta(:, i) + Thetaleader - Theta(:, i));
            %else
                %U(:, i) = U(:, i) + A(i, j) * (P(:, j) - P(:, i));
                %W(:, i) = W(:, i) + A(i, j) * (Theta(:, j) - Theta(:, i));
            %end

        end

    end

end
