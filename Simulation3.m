close all
clear
clc

%% initialization

N = 7; % number of agents

time_samples = 700; % number of samples in time period

radius_agents = 5; % agent random position radius
radius_star = 3; % agent desire position redius

P_g = zeros(2, N, time_samples + 1); % agents position memory
Theta_g = zeros(1, N, time_samples + 1); % agents orientation memory

P_g(:, :, 1) = radius_agents * (rand(2, N) - 0.5); % initialize agents position randomly
Theta_g(:, :, 1) = rand(1, N) - 0.5; % initialize agent orientation randomly

P_star_g = [radius_star / 1 * cos(2 * pi * (0:N - 1) / N); radius_star * sin(2 * pi * (0:N - 1) / N)]; % agents desire position

Theta_g_hat = zeros(1, N, time_samples + 1); % observer memory

kp = 10;
kt = 10;

%%switching
S = 5;
A = zeros(N, N, S);
Aj = [zeros(N - 1, 1), eye(N - 1); 1, zeros(1, N - 1)];

for i = 1:S - 1
    A(:, :, i) = ((Aj .* rand(N, N)) > (1 - 1 / S));
    Aj = Aj - A(:, :, i);
end

A(:, :, S) = Aj;

Switching_sequence = randi(S, time_samples, 1);

%% simulation

time = 0; % initialize time value
time_step = 0.01; % define time step
time_axis = 0:time_step:time_samples * time_step; % calculate all time values from number of steps and time-step
iteration = 1; % define iteration

while iteration <= time_samples

    % controller value
    [U_i, W_i] = Controllers(P_g(:, :, iteration), P_star_g, Theta_g(:, :, iteration), Theta_g_hat(:, :, iteration), kp, A(:, :, Switching_sequence(iteration)));

    % derivatives variables

    [Theta_g_hat_dot] = Observers(Theta_g_hat(:, :, iteration), Theta_g(:, :, iteration), kt, A(:, :, Switching_sequence(iteration)));
    [P_g_dot, Theta_g_dot] = Agents(P_g(:, :, iteration), Theta_g(:, :, iteration), U_i, W_i);

    % update values

    Theta_g_hat(:, :, iteration + 1) = Theta_g_hat(:, :, iteration) + time_step * Theta_g_hat_dot;
    P_g(:, :, iteration + 1) = P_g(:, :, iteration) + time_step * P_g_dot;
    Theta_g(:, :, iteration + 1) = Theta_g(:, :, iteration) + time_step * Theta_g_dot;

    iteration = iteration + 1;
end

%% plot

% p_star_g correction

P_tmp = P_g(:, :, time_samples + 1);
P_end = P_tmp - sum(P_tmp, 2) * ones(1, N) / N;
Theta_g_hat_star = sum(Theta_g_hat(1, :, end), 2) / N;
R = [cos(Theta_g_hat_star), -sin(Theta_g_hat_star); sin(Theta_g_hat_star), cos(Theta_g_hat_star)];
P_star_g_correct = R * P_star_g;
P_star_tmp = P_star_g;
P_error = norm(P_end - P_star_tmp);

for i = 0:0.1:360 - 0.1
    R = [cosd(i), -sind(i); sind(i), cosd(i)];
    P_star_tmp = R * P_star_g;
    P_error_tmp = norm(P_end - P_star_tmp);

    if P_error_tmp < P_error
        P_error = P_error_tmp;
        P_star_g_correct = P_star_tmp;
    end

end

f = figure;
h = scatter(P_g(1, :, 1), P_g(2, :, 1), 'bo', 'LineWidth', 2);
hold on
Arrows = [];

for i = 1:N

    for j = 1:N
        anArrow = annotation('arrow');
        anArrow.Parent = f.CurrentAxes;
        anArrow.Position = [P_g(1, j, 1), P_g(2, j, 1), P_g(1, i, 1) - P_g(1, j, 1), P_g(2, i, 1) - P_g(2, j, 1)];

        if A(i, j, Switching_sequence(1)) == 0
            anArrow.Color = 'none';
        else
            anArrow.Color = 'K';
        end

        Arrows = [Arrows anArrow];
    end

end

T = title(sprintf("time = %2.2f", time_axis(1)));
axis equal
axis([-6 6 -6 6])
grid on
grid minor
fr = getframe(gcf);
im = frame2im(fr);
[AA, map] = rgb2ind(im, 256);
imwrite(AA, map, 'b3.gif', 'gif', 'DelayTime', time_step, 'LoopCount', inf); %save file output

for i = 2:time_samples
    T.String = sprintf("time = %2.2f", time_axis(i));
    h.XData = P_g(1, :, i);
    h.YData = P_g(2, :, i);
    Arrow_index = 1;

    for j = 1:N

        for k = 1:N
            Arrows(Arrow_index).Position = [P_g(1, k, i), P_g(2, k, i), P_g(1, j, i) - P_g(1, k, i), P_g(2, j, i) - P_g(2, k, i)];

            if A(j, k, Switching_sequence(i)) == 0
                Arrows(Arrow_index).Color = 'none';
            else
                Arrows(Arrow_index).Color = 'K';
            end

            Arrow_index = Arrow_index + 1;
        end

    end

    drawnow
    fr = getframe(gcf);
    im = frame2im(fr);
    [AA, map] = rgb2ind(im, 256);
    imwrite(AA, map, 'b3.gif', 'gif', 'WriteMode', 'append', 'DelayTime', time_step); %save file output
    pause(time_step)
end

close all

figure

plot(reshape(P_g(1, :, :), N, time_samples + 1).', reshape(P_g(2, :, :), N, time_samples + 1).')
hold on
plot(P_star_g_correct(1, :).' + sum(P_g(1, :, time_samples + 1)) / N, P_star_g_correct(2, :).' + sum(P_g(2, :, time_samples + 1)) / N, '*')
plot(P_g(1, :, time_samples + 1), P_g(2, :, time_samples + 1), 'k^')
plot(P_g(1, :, 1), P_g(2, :, 1), 'ko')
axis equal

grid on
grid minor

%% define Agents function

function [P_g_dot, Theta_g_dot] = Agents(P_g, Theta_g, U_i, W_i)
    P_g_dot = zeros(size(P_g));
    Theta_g_dot = zeros(size(Theta_g));

    N = size(Theta_g, 2);

    for i = 1:N
        [p_g_dot_tmp, theta_g_dot_tmp] = Agent(P_g(:, i), Theta_g(:, i), U_i(:, i), W_i(:, i));
        P_g_dot(:, i) = p_g_dot_tmp;
        Theta_g_dot(:, i) = theta_g_dot_tmp;
    end

end

function [p_g_dot, theta_g_dot] = Agent(p_g, theta_g, u_i, w_i)
    R_i2g = [cos(theta_g), -sin(theta_g); sin(theta_g), cos(theta_g)];

    p_g_dot = R_i2g * u_i;
    theta_g_dot = w_i;
end

function [U_i, W_i] = Controllers(P_g, P_star_g, Theta_g, Theta_g_hat, kp, A)
    N = size(Theta_g, 2);

    U_i = zeros(size(P_g));
    W_i = zeros(1, N);

    for i = 1:N
        R_i2g = [cos(Theta_g(i)), -sin(Theta_g(i)); sin(Theta_g(i)), cos(Theta_g(i))];
        R_i2g_hat = [cos(Theta_g_hat(i)), -sin(Theta_g_hat(i)); sin(Theta_g_hat(i)), cos(Theta_g_hat(i))];

        for j = 1:N
            U_i(:, i) = U_i(:, i) + kp * A(i, j) * (R_i2g.' * (P_g(:, j) - P_g(:, i)) - R_i2g_hat.' * (P_star_g(:, j) - P_star_g(:, i)));
            W_i(:, i) = W_i(:, i) + A(i, j) * (Theta_g(j) - Theta_g(i));
        end

    end

end

function [Theta_g_hat_dot] = Observers(Theta_g_hat, Theta_g, kt, A)
    N = size(Theta_g, 2);
    Theta_g_hat_dot = zeros(1, N);

    for i = 1:N

        for j = 1:N
            Theta_g_hat_dot(i) = Theta_g_hat_dot(i) + kt * A(i, j) * ((Theta_g_hat(j) - Theta_g_hat(i)) - (Theta_g(j) - Theta_g(i)));
        end

    end

end
