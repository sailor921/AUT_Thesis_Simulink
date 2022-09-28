function [Xdot] = agent(X,U,P)

% states
x = X(1);
y =X(2);
theta = X(3);

%Inputs 
w1 = U(1);
w2 = U(2);
w3 = U(3);

%Parameters
r_wheel = P(1);
L = P(2);
%Rotation Matrix
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];

% speed of three wheel
v1 = w1*r_wheel;
v2 = w2*r_wheel;
v3 = w3*r_wheel;

% angular speed of robot
omega = v1/L + v2/L + v3/L;
Xdot(3) = omega;

%V in global frame
vg = R*(v1*[-1;0] + v2*[-cos(30);-sin(30)]+ v3*[cos(30);sin(30)]);
Xdot(1) = vg(1);    %vx
Xdot(2) = vg(2);    %vy

end
