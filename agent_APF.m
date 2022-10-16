close all
clear
clc
%%
% **The Artificial Potencial Field path planning with single agent**

% Varaibles

%Simulation time
time_steps = 1000;
tstep = 0.01;
time_sim = time_steps * tstep;

%Agent position
P = zeros(2, 1, time_steps+1);
P(:, :, 1) = [1; 3];

%Create Obstacle
Pobstacle = [5; 5];

%Create goal point
Pgoal = [10; 10];

%APF variables
katt = -10;
krep = -50;
d = 0.2;
rho_obstacle = 10;
% these variables test for plot
potencial_vec = zeros(1, 1, time_steps+1);
attpot_vec = [];
reppot_vec = [];
att_vec = [];
rep_vec = [];

%%
%Simulation 

Error = 1;
iteration = 1;

while iteration <= time_steps

    %Controller
    [U] = controller(P(:, :, iteration), Pgoal, katt, d, Pobstacle, rho_obstacle, krep);
    
    %Potential Functions
    %Attractive Potencial
    [attpot] = attractive_pot(P(:, :, iteration), Pgoal, katt, d);
    
    %Repulsive Potencial
    [reppot] = repulsive_pot(P(:, :, iteration), Pobstacle, rho_obstacle, krep);

    %test
    [att] = attractive(P(:, :, iteration), Pgoal, katt, d);
    [rep] = repulsive(P(:, :, iteration), Pobstacle, rho_obstacle, krep);
    att_vec = [att_vec, att];
    rep_vec = [rep_vec, rep];
    
    %Derivative variables, these are velocities of agents
    [pdot] = agent(P(:, :, iteration), U);
   
    %Update variables
    %Update Position of agent
    P(:, :, iteration+1) = P(:, :, iteration) + tstep * pdot;
    
    %Update potential vector
    temppot = attpot + reppot;
    potencial_vec(:, :, iteration+1) = potencial_vec(:, :, iteration) + temppot;
    attpot_vec = [attpot_vec, attpot];
    reppot_vec = [reppot_vec, reppot];

    Error = norm(Pgoal - P(:, :, iteration));

    iteration = iteration + 1;

end

%%
%Plot

fig = figure('Name', 'Artificial Potencial Field', 'NumberTitle', 'off');
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

% for i = 1:length(size(Pobstacle, 1))
% 
%     [plots] = potential_plots(x, y, Pobstacle)
% 
% end
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
    
    dist = P - Pobstacle;
    temp = norm(dist);
    Grad_obstacle = dist/temp;

    if (temp <= rho_obstacle)
        rep = -krep * (1/temp - 1/rho_obstacle) * (Grad_obstacle)/(temp^2);
    else
        rep = [0; 0];
    end
   
end

%Repulsive Potential
function [reppot] = repulsive_pot(P, Pobstacle, rho_obstacle, krep)
    
    dist = P - Pobstacle;
    temp = norm(dist);

    if (temp <= rho_obstacle)
        reppot = 0.5 * -krep * (1/temp - 1/rho_obstacle);
    else
        reppot = 0;
    end

end

%Controller
function [U] = controller(P, Pgoal, katt, d,Pobstacle ,rho_obstacle, krep)
    
    U = zeros(size(P));

    attforce = attractive(P, Pgoal, katt, d);
    repforce = repulsive(P, Pobstacle ,rho_obstacle, krep);
    U(:, 1) = attforce + repforce;

end

%Plotting
% function [plots] = potential_plots(x, y, Pobstacle)
%     [xx, yy] = meshgrid(x, y);
% 
%     pos1 = sqrt(power((xx - Pobstacle(1, :)), 2) + power((xx - Pobstacle(2, :)), 2));
% 
%     for i = 1:numel(x)
%         for j = 1:numel(y)
%             if pos1(i, j) > 1.5 && pos1(i, j) <= 0.5
%                 posf1(i, j) = pos1(i, j).^(-1);
%             end
%             if pos1(i, j) >= 5
%                 posf1(i,j) = 0;
%             end
%             if pos1(i,j) <= 1.5
%                 posf1(i,j) = 1/(1.5);
%             end
%         end
%     end
%     negd2 = (xx - 102).^2 + (yy - 105).^2 - 0.002;
%     negd = sqrt(negd2);
%     neg = negd.^(-1);
%     zz = posf1 - neg;
%     figure(2)
%     title('Potential Fields plot')
%     xlabel('x')
%     ylabel('y')
%     hold on
%     mesh(real(zz))
%     pause(1)
%     figure(3)
%     title('Quiver plot of the environment')
%     xlabel('x')
%     ylabel('y')
%     hold on
%     [px, py] = gradient(zz, .1, .1);
%     quiver(x,y,-px,-py,1.2, 'r'), hold on
%     quiver(x(12),y(12),-px(12),-py(12),2, 'g')
%     pause(1)
%     figure(4)
%     title('Contour plot of the environment')
%     xlabel('x')
%     ylabel('y')
%     hold on
%     contour(zz,21)
% 
% end

