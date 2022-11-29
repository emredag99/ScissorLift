%=====================================================================%
%                Concept Evaluation for Actuator Position             %
%                        Scissor Mechanism                            %
%---------------------------------------------------------------------%
%  Coded by:  EMRE DAĞ                                  27/11/2022    %
%=====================================================================%

clear all
close all
clc

g=9.81; %gravitational acceleration
B=200*g; % weight of the scissor mechanism [N]
W=300*g; % payload [N]
Load=W+B/2; %Effective load at the top plate [N]

n=3; %number of levels in the mechanism
l2=1; %length of an arm [m]

theta_i=10; theta_l=70; %initial and last values for theta [deg]
theta=(theta_i:0.01:theta_l)*pi/180; % acute angle between static end and ground [rad]
theta_d=0.1; % only written to find the rates of other parameters [rad/s]
%%%%%%%Check if the value of theta_d affects the rates of other parameters
%%%%%%%such as dh/dl2--> Checked that it is independent

s1=l2*cos(theta); % distance between static end and translating end [m]
h=n*l2*sin(theta); %the extended length of the mechanism, distance taken from the joints [m]

h_d= n*l2*theta_d*cos(theta); %rate of total extension length [m/s]
s1_d= -l2*theta_d*sin(theta); %rate of s1 [m/s]

% at this point we can study different type of actuator positions

%% Concept 1 --> Directly driving the bottom translating end
travel_1=l2*cos(theta_i*pi/180)-s1; %the extension of the actuator [m]
rate_1=n./tan(theta); %rate of dh/dl, unitless
F1=(Load)*rate_1; %load on the actuator [N]

figure(1)
subplot(3,1,1,"align")
plot(h, F1)
title("Required Actuator Force versus Height")
ylabel("Required Force [N]")
xlabel("Height of the Upper Table [m]")
grid on

subplot(3,1,2,"align")
plot(h, travel_1)
title("Travel of the Actuator versus Height")
ylabel("Travel of the Actuator [m]")
xlabel("Height of the Upper Table [m]")
grid on

subplot(3,1,3, "align")
plot(h, theta*180/pi)
title("Angle of links wrt to ground versus Height")
ylabel("Angle of links wrt to ground [degree]")
xlabel("Height of the Upper Table [m]")
grid on

%% Concept 2 --> Between fixed bottom end and its vertically adjacent joint
travel_2= l2*sin(theta)-l2*sin(theta_i*pi/180); %the extension of the actuator [m]
rate_2= n; %rate of dh/dl, unitless
F2= Load*rate_2*ones(length(theta)); %load on the actuator [N]

figure(2)
subplot(3,1,1,"align")
plot(h, F2)
title("Required Actuator Force versus Height")
ylabel("Required Force [N]")
xlabel("Height of the Upper Table [m]")
grid on

subplot(3,1,2,"align")
plot(h, travel_2)
title("Travel of the Actuator versus Height")
ylabel("Travel of the Actuator [m]")
xlabel("Height of the Upper Table [m]")
grid on

subplot(3,1,3, "align")
plot(h, theta*180/pi)
title("Angle of links wrt to ground versus Height")
ylabel("Angle of links wrt to ground [degree]")
xlabel("Height of the Upper Table [m]")
grid on

%% Concept 3 --> From ground to the joint vertically adjacent to the translating end
l1=l2*cos(theta_i*pi/180); % the position of the motor at the ground
travel_3 = sqrt((l2*sin(theta)).^2+(-l1+l2*cos(theta)).^2)-sqrt((l2*sin(theta_i*pi/180)).^2+(-l1+l2*cos(theta_i*pi/180)).^2); %the extension of the actuator [m]
rate_3= n*cos(theta); %rate of dh/dl, unitless
F3= Load*rate_3; %load on the actuator [N]

figure(3)
subplot(3,1,1,"align")
plot(h, F3)
title("Required Actuator Force versus Height")
ylabel("Required Force [N]")
xlabel("Height of the Upper Table [m]")
grid on

subplot(3,1,2,"align")
plot(h, travel_3)
title("Travel of the Actuator versus Height")
ylabel("Travel of the Actuator [m]")
xlabel("Height of the Upper Table [m]")
grid on

subplot(3,1,3, "align")
plot(h, theta*180/pi)
title("Angle of links wrt to ground versus Height")
ylabel("Angle of links wrt to ground [degree]")
xlabel("Height of the Upper Table [m]")
grid on
%% Comparing the Concepts

maximumForces= [max(F1), F2(1), max(F3)]'; %since F2 is full of constants
actuatorTravel= [max(travel_1), max(travel_2) , max(travel_3)]';

Results = table(maximumForces, actuatorTravel)
max(h)
%% Loads on the revolute joints