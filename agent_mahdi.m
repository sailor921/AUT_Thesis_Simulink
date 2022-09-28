%parameters
%%
time_steps = 200;
delta_t = 0.1;
time_sample = time_steps*delta_t;
N = 3; % Number of agent

% random initial value
radius = 5;
x1 = radius*cos(rand);
y1 = radius*sin(rand);
theta1 = radius*rand;
x2 = 2*radius*cos(rand);
y2 = radius*sin(rand);
theta2 = radius*rand;
x3 = 3*radius*cos(rand);
y3 = radius*sin(rand);
theta3 = radius*rand;

%State vecor of agent
X1 = [x1 ; y1 ; theta1];
X2 = [x2 ; y2 ; theta2];
X3 = [x3 ; y3 ; theta3];

%Velocity
V1 = ones(3,1);
V2 = ones(3,1);
V3 = ones(3,1);

%%
X1_vec = [];
X2_vec = [];
X3_vec = [];

%Calculation
for i=1:time_steps
    
    X1 = X1 + V1*delta_t;
    X2 = X2 + V2*delta_t;
    X3 = X3 + V3*delta_t;
    X1_vec = [X1_vec , X1];
    X2_vec = [X2_vec , X2];
    X3_vec = [X3_vec , X3];
    
    plot(X1(1) , X1(2) , 'ro')
    grid on
    hold on 
    plot(X2(1) , X2(2), 'bo')
    hold on 
    plot(X3(1), X3(2), 'go')
    xlim([-10 , time_steps / 5])
    ylim([-10 , time_steps / 5])
    pause(delta_t)
    %theta1(:,i) = theta1(:,i-1) + v1*delta_t;
    %theta1(:,i) = [x1(:,i),y1(:,i)];
end

