clc;
close all;
clear all;

num_drones = 4;
target_pos = [5;5];


%% Setup Initial Conditions 

% x_i = [q_iT, dq_iT]T
%     = [ x; y; dx; dy ]


% Setup the target
xt = [ target_pos; 0; 0 ];

% Setup the agents
x = [ 0; 0; 0; 0 ];
z = [ x - xt ];
sum = zeros(4,1);

d_arclength = 2;

for i = 1:num_drones
    dtheta = 2*pi/num_drones;
    r = d_arclength / dtheta;
    x = [ r*cos(i*dtheta); r*sin(i*dtheta); 0; 0 ];
    sum = sum + x;
    z = [z; x];
end

% Final term is centroid of swarm
z = [z; 1/num_drones*sum ];

disp(z)
disp(size(z))