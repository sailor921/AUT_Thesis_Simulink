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

%Variable for Error of postions
%Xerror vector
x1error = [];
x2error = [];
x3error = [];

%Yerror vector
y1error = [];
y2error = [];
y3error = [];

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
    Pplot(:, :, iteration+1) = [P(:, :, iteration+1), P(:, 1, iteration+1)];

    %Leader Position
    Pleader(:, :, iteration+1) = [t; sin(0.2*t)];%Pleader(:, :, iteration) + tstep * Vleader;
    Thetaleader(:, :, iteration+1) = Thetaleader(:, :, iteration) + tstep * Omegaleader;

    %Desired Position
    Pstar(:,1,iteration+1) = [0;offset] + Pleader(:, :, iteration);
    Pstar(:,2,iteration+1) = [-offset;0] + Pleader(:, :, iteration);
    Pstar(:,3,iteration+1) = [0;-offset] + Pleader(:, :, iteration);
    
    %Desired Theta

    iteration = iteration + 1;

    Error = Pgoal - Pleader(:, :, iteration);

end

%%
%Plot
for k = 2:time_steps
   
    x1error = [x1error Pstar(1, 1, k) - P(1, 1, k)];
    x2error = [x2error Pstar(1, 2, k) - P(1, 2, k)];
    x3error = [x3error Pstar(1, 3, k) - P(1, 3, k)];
    
    y1error = [y1error Pstar(2, 1, k) - P(2, 1, k)];
    y2error = [y2error Pstar(2, 2, k) - P(2, 2, k)];
    y3error = [y3error Pstar(2, 3, k) - P(2, 3, k)];
        
end

fig = figure('Name','Dynamic Formation','NumberTitle','off');;
hold on

%Create arrows of agent
%Theta varaibles
Lvec = 0.01;

Agent_arrows = [];

for i = 1:N
    
    anArrow = annotation('arrow');
    anArrow.Parent = fig.CurrentAxes;
    anArrow.Color = 'r';
    anArrow.Position = [P(1, i, 1), P(2, i, 1), Lvec*cos(Theta(:, i, 1)), Lvec*sin(Theta(:, i, 1))];
    Agent_arrows = [Agent_arrows anArrow];
    
end

grid on
grid minor
axis equal
axis ([-12 12 -12 12])

for k = 2:time_steps

    %Position of Leader and Agents
    plot(Pleader(1, :, k), Pleader(2, :, k), 'b^')  %Position of Leader
    
    plot(P(1, 1, k), P(2, 1, k), 'r.')  %Position of agent 1
    plot(P(1, 2, k), P(2, 2, k), 'g.')  %Position of agent 2
    plot(P(1, 3, k), P(2, 3, k), 'b.')  %Position of agent 3
       
    %Plot triangle formation
    if (mod(k,20) == 0)
        plot(Pplot(1, :, k), Pplot(2, :, k),'k-');  %Formation polygon
    end
    
    for j = 1:N
        
        Agent_arrows(j).Position = [P(1, j, k), P(2, j, k), Lvec*Theta(:, j, k), Lvec*Theta(:, j, k)];
        
    end
    
%     drawnow
    pause(0)

end

%Plot Error of X position of agents
figure('Name', 'Error of X position of agents', 'NumberTitle', 'off');
hold on
grid on
grid minor
plot(x1error,'r-');
plot(x2error,'g--');
plot(x3error, 'b:');
legend({'X error of agent 1', 'X error of agent 2', 'X error of agent 3'}, 'Location', 'southeast')
legend('boxoff')

%Plot Error of Y position of agents
figure('Name', 'Error of Y position of agents', 'NumberTitle', 'off');
hold on
grid on
grid minor
plot(y1error, 'r-');
plot(y2error, 'g--');
plot(y3error, 'b:');
legend({'Y error of agent 1', 'Y error of agent 2', 'Y error of agent 3'}, 'Location', 'southeast')
legend('boxoff')

%Plot Trajectory, Initial, goal position of agents
figure('Name','Trajectory and Special positaion of agents','NumberTitle','off');
plot(reshape(P(1,:,:),[N, time_steps+1]).', reshape(P(2,:,:),[N, time_steps+1]).')
hold on
grid on 
grid minor
plot(P(1, :, time_steps + 1), P(2, :, time_steps + 1), 'k>')    %Final position
plot(P(1, :, 1), P(2, :, 1), 'ko')  %Initial position
plot(Pstar(1, :,time_steps+1), Pstar(2, :, time_steps+1), 'r*');    %Desired Final position
plot(Pleader(1, :,time_steps+1), Pleader(2, :,time_steps+1), 'b^') %Final position of Leader  
axis equal
axis ([-12 12 -12 12])


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
            
            U(:, i) = U(:, i) + A(i, j) * (P(:, j) - P(:, i) - Pstar(:, j) + Pstar(:, i));
            W(:, i) = W(:, i) + A(i, j) * (Theta(:, j) - Theta(:, i) - Thetastar(:, j) + Thetastar(:, i));            
            
            %Input Saturation
%             if (U(1, i) >= 0.5)
%                 U(1, i) = 0.5;
% 
%             elseif (U(2, i) >= 0.5)
% 
%                 U(2, i) = 0.5;
% 
%             end

        end

    end

end
