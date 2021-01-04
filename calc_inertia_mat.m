clc;
close all;


tot_mass = 1.0;
num_rotors = 4;
motor_mass = 0.055;
arm_length = 0.2275;
body_box_x = 0.180;
body_box_y = 0.11;
body_box_z = 0.040;
rotor_z = 0.0025;

box_mass = tot_mass - num_rotors*motor_mass;

for i = 1:num_rotors
    switch i
        case 1
            angle = 45;
        case 2
            angle = 225;
        case 3
            angle = 315;
        case 4
            angle = 135;
    end
    
    pos(i).x = arm_length*cos(deg2rad(angle));
    pos(i).y = arm_length*sin(deg2rad(angle));
    pos(i).z = rotor_z;
    
    disp(pos(i))
end


inertia = zeros(3,3);


%http://farside.ph.utexas.edu/teaching/336k/Newtonhtml/node64.html

inertia(1, 1) = box_mass / 12.0 * (body_box_y()*body_box_y() + body_box_z()*body_box_z());
inertia(2, 2) = box_mass / 12.0 * (body_box_x()*body_box_x() + body_box_z()*body_box_z());
inertia(3, 3) = box_mass / 12.0 * (body_box_x()*body_box_x() + body_box_y()*body_box_y());

for i = 1:num_rotors
    inertia(1, 1) = inertia(1, 1) + (pos(i).y()*pos(i).y() + pos(i).z()*pos(i).z()) * motor_mass;
    inertia(2, 2) = inertia(2, 2) + (pos(i).x()*pos(i).x() + pos(i).z()*pos(i).z()) * motor_mass;
    inertia(3, 3) = inertia(3, 3) + (pos(i).x()*pos(i).x() + pos(i).y()*pos(i).y()) * motor_mass;
end


disp("Inertial Matrix:")
disp(inertia)





C_T = 0.109919;
C_P = 0.040164;
air_density = 1.225;
max_rpm = 6396.667;
prop_diameter = 0.2286;
prop_height = 0.001;

rps = max_rpm/60;

max_thrust = C_T * air_density * rps^2 * prop_diameter^4;
max_torque = C_P * air_density * rps^2 * prop_diameter^5 / (2*pi);

disp("Max Thrust per Rotor")
disp(max_thrust)

disp("Max Torque per Rotor")
disp(max_torque)
