close all 
clear 
clc

%%
% **In this simulation we want to  achieve formation dynamic with change position of leader** % 

%Variables
%Simulation time
time_steps = 100;
tstep = 0.1;
time_sim = time_steps * tstep;

%Number of agents
N = 3;  

%Adjacency Matrix
A = [zeros(N-1,1) , eye(N-1); ones(1,1) , zeros(1, N-1)];

%Position Vectors
P = zeros(2, N, time_steps+1);  %X and y array
Theta = zeros(1, N, time_steps+1);  %theta array

%Random inialization of position and orentation
alpha = 10;
radious_agent = sqrt(3)/6 * alpha;
P(:, :, 1) = radious_agent * rand(2, N);
Theta(:,:,1) = alpha * rand(1,1);

%Formation Variables

kp = 10;

%Leader Variables
Pleader = zeros(2, 1, time_steps+1);
Thetaleader= zeros(1, 1, time_steps+1);
Vleader = [0.5; 0.5];
Omegaleader = 0;

%Initial Position of Leader
Pleader(:, 1, 1) = [1; 1];
Thetaleader(:, 1, 1) = 0;

%Goal Position of Leader
Pgoal = [10; 10];
Thetagoal = 0;

offset = 3; %This is for formation in half circle around leader

%Desired position of agents
Pstar = zeros(2, N, time_steps+1);
Thetastar = zeros(1, N, time_steps+1);

%Plot formation 
Pplot = zeros(2, N+1, time_steps+1);
Pplot(:, :,1) = zeros(2, N+1);

%%
%Simulation

iteration = 1;
Error = ones(2,1);
t = tstep;

while t<time_sim %norm(Error) >= 0.2
    %Controlles 
    t = t + tstep;
    [U, W] = controller(P(:,:,iteration), Theta(:,:,iteration), A, Pstar(:,:, iteration), Thetastar(:,:, iteration), kp);

    %Derivative variables, these are velocities of agents
    [P_dot, Theta_dot] = agents(P(:,:,iteration), Theta(:,:,iteration), U, W);
    
    %Update
    %Agent Postion
    P(:, :, iteration+1) = P(:, :, iteration) + tstep * P_dot;
    Theta(:, :, iteration+1) = Theta(:, :, iteration) + tstep * Theta_dot;
    
    %Variables for triangle formation
    Pplot(:, :, iteration+1) = [P(:, :, iteration+1), P(:,1, iteration+1)];

    %Leader Position
    Pleader(:, :, iteration+1) = [t; sin(0.2*t)];%Pleader(:, :, iteration) + tstep * Vleader;
    Thetaleader(:, :, iteration+1) = Thetaleader(:, :, iteration) + tstep * Omegaleader;

    %Desired Position
    Pstar(:,1,iteration+1) = [0;offset] + Pleader(:, :, iteration);
    Pstar(:,2,iteration+1) = [-offset;0] + Pleader(:, :, iteration);
    Pstar(:,3,iteration+1) = [0;-offset] + Pleader(:, :, iteration);
    

    iteration = iteration + 1;

    Error = Pgoal - Pleader(:, :, iteration);

end

%%
%Plot
%Theta varaibles
L = 2;

figure
hold on
grid on
grid minor
axis ([-12 12 -12 12])
anArrow = annotation('arrow');
anArrow.Parent = gca;  % or any other existing axes or figure
for k = 1:time_steps

    plot(Pleader(1, :, k+1), Pleader(2, :, k+1), 'b^')  %Position of Leader
    
    plot(P(1, 1, k), P(2, 1, k), 'b.')  %Position of agent 1
    plot(P(1, 2, k), P(2, 2, k), 'r.')  %Position of agent 2
    plot(P(1, 3, k), P(2, 3, k), 'g.')  %Position of agent 3
    anArrow.Position = [P(1, 1, k), P(2, 1, k), 0.1*cos(0.5), 0.1*sin(0.5)] ;
    %Plot formation
    if (mod(k,20) == 0)
        plot(Pplot(1, :, k), Pplot(2, :, k),'k-');  %Formation polygon
    end

    pause(0)

end


% figure
% plot(reshape(P(1,:,:),[N, time_steps+1]).', reshape(P(2,:,:),[N, time_steps+1]).')
% hold on
% plot(P(1, :, time_steps + 1), P(2, :, time_steps + 1), 'k^')    %Final position
% plot(P(1, :, 1), P(2, :, 1), 'ko')  %Initial position
% hold on 
% plot(Pstar(1,:), Pstar(2,:), 'r*')
% grid on 
% grid minor

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
        P_dot(:, i) = p_dot_tmp;
        Theta_dot(:, i) = theta_dot_tmp;  

    end

end

function [U, W] = controller(P, Theta, A, Pstar, Thetastar, kp)
    N = size(P, 2);
    U = zeros(size(P));
    W = zeros(size(Theta));

    for i = 1:N
        U(:, i) = kp * (Pstar(:, i) - P(:, i));

        for j = 1:N
            U(:, i) = U(:, i) + A(i, j) * (P(:, j) - P(:, i) - Pstar(:, j) + Pstar(:, i) + rand(2,1));
            W(:, i) = W(:, i) + A(i, j) * (Theta(:, j) - Theta(:, i) - Thetastar(:, j) + Thetastar(:, i));            
            
            if (U(1, i) >= 0.5)
                U(1, i) = 0.5;

            elseif (U(2, i) >= 0.5)

                U(2, i) = 0.5;

            end

        end

    end

end
