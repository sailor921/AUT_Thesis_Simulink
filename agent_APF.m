close all
clear
clc
%%
% **The Artificial Potencial Field path planning with single agent**

% Varaibles
global Pgoal Pobstacle katt krep d rho_obstacle

%Simulation time
time_steps = 1000;
tstep = 0.01;
time_sim = time_steps * tstep;

%Agent position
P = zeros(2, 1, time_steps+1);
P(:, :, 1) = [1; 3];

%Create Obstacle
numobs = 3;
Pobstacle = zeros(2, numobs);
Pobstacle(:, 1) = [5; 5];
Pobstacle(:, 2) = [3; 10];
% Pobstacle(:, 3) = [8; 8];   %Good
Pobstacle(:, 3) = [7; 8]; %Local minumum problem can solve with change of katt and krep
Pobstacle(:, 4) = [3; 5];
Pobstacle(:, 5) = [6; 2];

%Create goal point
Pgoal = [10; 10];

%APF variables
katt = -25;%-10;
krep = -20;%-50;
d = 0.2;
rho_obstacle = 10;

%%
%Simulation 
Error = 1;
iteration = 1;

while iteration<= time_steps

    %Controller
    [U] = controller(P(:, :, iteration), Pgoal, katt, d, Pobstacle, rho_obstacle, krep);
    
    %Potential Functions
    %Attractive Potencial
    [attpot] = attractive_pot(P(:, :, iteration), Pgoal, katt, d);
    
    %Repulsive Potencial
    [reppot] = repulsive_pot(P(:, :, iteration), Pobstacle, rho_obstacle, krep);

    %Derivative variables, these are velocities of agents
    [pdot] = agent(P(:, :, iteration), U);
   
    %Update variables
    %Update Position of agent
    P(:, :, iteration+1) = P(:, :, iteration) + tstep * pdot;
    
    Error = norm(Pgoal - P(:, :, iteration));

    iteration = iteration + 1;

end


%%
%Plot

%Plot trajectory of robot
fig = figure('Name', 'Artificial Potential Field', 'NumberTitle', 'off');
hold on
plot(Pgoal(1, :), Pgoal(2, :), 'k*');
plot(Pobstacle(1, :), Pobstacle(2, :), 'b>');
axis([-15 15 -15 15])
grid on 
grid minor

for i = 2:time_steps

    plot(P(1, :, i), P(2, :, i), 'ro');
    pause(0);

end

%Plot meshgrid potential
figure(2)

xx = 0:0.1:12;
yy = 0:0.1:12;

[X, Y] = meshgrid(xx, yy);
Z = zeros(size(X));

for i = 1:numel(xx)

    for j = 1:numel(yy)
        Z(i, j) = potential(X(i, j), Y(i, j));
    end

end
%Plot surface of potential
surf(X, Y, Z)

%%
%Functions

%Agent 
function [pdot] = agent(p,u)
    
    pdot = zeros(size(p));
    pdot = u;

end

%Attractive Force
function [att] = attractive(P, Pgoal, katt, d)

    dist = P - Pgoal;
    temp = norm(dist);

    if (temp <= d)
        att = katt * (dist);
    else 
        att = (d * katt * dist) / temp;
    end

end

%Attractive Potencial
function [attpot] = attractive_pot(P, Pgoal, katt, d)

    dist = P - Pgoal;
    temp = norm(dist);

    if (temp <= d)
        attpot = 0.5 * katt * power(temp, 2);
    else
        attpot = d * katt * temp - 0.5 * katt * power(d, 2);
    end

end

%Repulsive Force
function [rep] = repulsive(P, Pobstacle, rho_obstacle, krep)
    rep = 0;
    for i = 1:size(Pobstacle, 2)
        
        dist = P - Pobstacle(:, i);
        temp = norm(dist);
        Grad_obstacle = dist/temp;
    
        if (temp <= rho_obstacle)
            rep = rep + -krep * (1/temp - 1/rho_obstacle) * (Grad_obstacle)/(temp^2);
        else
            rep = [0; 0];
        end

    end
   
end

%Repulsive Potential
function [reppot] = repulsive_pot(P, Pobstacle, rho_obstacle, krep)
    reppot = 0;
    for i = 1:size(Pobstacle, 2)

        dist = P - Pobstacle(:, i);
        temp = norm(dist);
    
        if (temp <= rho_obstacle)
            reppot = reppot + 0.5 * -krep * (1/temp - 1/rho_obstacle);
        else
            reppot = 0;
        end

    end

end

%Controller
function [U] = controller(P, Pgoal, katt, d,Pobstacle ,rho_obstacle, krep)
    
    U = zeros(size(P));

    attforce = attractive(P, Pgoal, katt, d);
    repforce = repulsive(P, Pobstacle ,rho_obstacle, krep);
    U(:, 1) = attforce + repforce;

end

function [pot] = potential(x, y)

    global Pgoal Pobstacle katt krep d rho_obstacle
    P = [x; y];
    attpot = attractive_pot(P, Pgoal, katt, d);
    reppot = repulsive_pot(P, Pobstacle, rho_obstacle, krep);
    pot = attpot + reppot;

end


