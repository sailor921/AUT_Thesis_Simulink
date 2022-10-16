close all 
clear 
clc

%%

% **In this simulation we assume that Leader is connected to agent 1**

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
Pleader = ones(2,1);
thetaleader= 0;

%Position Vectors
P = zeros(2, N, time_steps+1);  %X and y array
Theta = zeros(1, N, time_steps+1);  %theta array

%Random inialization of position and orentation
alpha = 10;
radious_agent = sqrt(3)/6 * alpha;
P(:, :, 1) = radious_agent * rand(2, N);
Theta(:,:,1) = alpha * rand(1,1);

%Velocity Vectos
%This sould not here because this is the output of functions
%P_dot = zeros(2, N, time_steps);  %Xdot and Ydot array
%Theta_dot = zeros(1, N, time_steps);  %thetadot array

%%
%Simulation

iteration = 1;

while iteration <= time_steps
    %Controlles 
    [U, W] = controller(P(:,:,iteration), Theta(:,:,iteration), A);

    %Derivative variables, these are velocities of agents
    [P_dot, Theta_dot] = agents(P(:,:,iteration), Theta(:,:,iteration), U, W);
    
    %Update
    P(:, :, iteration+1) = P(:, :, iteration) + tstep * P_dot;
    Theta(:, :, iteration+1) = Theta(:, :, iteration) + tstep * Theta_dot;

    iteration = iteration + 1;
end

%%
%Plot
figure 
hold on 
grid on 
grid minor

for k = 1:time_steps
    for i = 1:N
        plot(P(1, i, k), P(2, i, k), 'ro')
        plot(Pleader(1, :),Pleader(2, :), 'b*')
        axis([-2 2 -2 2])
        pause(0)
    end 
    pause(0)
end

figure
axis equal
plot(reshape(P(1,:,:),[N, time_steps+1]).', reshape(P(2,:,:),[N, time_steps+1]).')
hold on
axis([-2 2 -2 2])
plot(P(1, :, time_steps + 1), P(2, :, time_steps + 1), 'k^')    %Final position
plot(P(1, :, 1), P(2, :, 1), 'ko')  %Initial position
plot(Pleader(1, :), Pleader(2, :), 'k*')
grid on 
grid minor

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

function [U, W] = controller(P, Theta, A)
    N = size(P, 2);
    U = zeros(size(P));
    W = zeros(size(Theta));
    Pleader = ones(2,1);
    thetaleader= 0;

    for i = 1:N

        for j = 1:N
            if (i == 3)
                U(:, i) = U(:, i) + A(i, j) * (P(:, j) - P(:, i) + Pleader - P(:, j));
                W(:, i) = W(:, i) + A(i, j) * (Theta(:, j) - Theta(:, i) + thetaleader - Theta(:, i));
            else
                U(:, i) = U(:, i) + A(i, j) * (P(:, j) - P(:, i));
                W(:, i) = W(:, i) + A(i, j) * (Theta(:, j) - Theta(:, i));
            end
        end

    end

end
