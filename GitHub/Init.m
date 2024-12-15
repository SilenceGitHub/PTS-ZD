clear all;
close all;

addpath('functions')

global param robot data

%% Task parameter

param.taskDuration = 60;
param.step = 0.01;
param.scale = 0.2;
param.index = 1;

samplingNumber = round(param.taskDuration/param.step) + 1;

t = 0:param.step:param.taskDuration;

%% Algorithm parameters
param.t_p = 10;
param.rho = 0.6;
param.acf = 'ts_ptc';
% param.e0 = [-0.1,0.1,0.05]';
param.e0 = [-0.1;0.15;0.05];

switch param.acf
    case 'linear'
        param.lambda = 0.2; % convergence rate of the ZNN model
    case 'ptc'
        % controller parameter
%         vt0 = (max(abs(param.e0)))^(param.rho/2);
%         param.lambda = 2*atan(vt0)/(param.rho * param.t_p );
        param.lambda = pi/(param.rho * param.t_p );
    case 'ts_ptc'
        % controller parameter
        vt0 = (param.e0'*param.e0)^(param.rho/2);
        param.lambda =  atan(vt0)/(param.rho * param.t_p );
end

param.gamma = 10000; % convergence rate fo the GND model
param.max_iter_num = 1000;


%% Robot parameters
%Jaco2机械臂标准DH模型参数
% %末端指尖
D = [0.2755, 0.4100, 0.2073, 0.0741, 0.0741, 0.1600];

e2 = 0.0098;
aa = 30*pi/180;
sa = sin(aa);
s2a = sin(2*aa);
d4b = D(3)+sa/s2a*D(4);
d5b = sa/s2a*D(4)+sa/s2a*D(5);
d6b = sa/s2a*D(5)+D(6);

param.robot.alpha_i = [pi/2, pi, pi/2, 2*aa, 2*aa, pi];
param.robot.a_i = [0, D(2), 0, 0, 0, 0];
param.robot.d_i = [D(1), 0, -e2, -d4b, -d5b, -d6b];



% Parameters of the mobile platform
param.robot.d = 0.1;
param.robot.r = 0.2;
param.robot.b = 0.32;

%% Create a robot
robot = MobileRobot(param.robot);

%% Allocate memory

data.actualPath = zeros(robot.taskDimension, samplingNumber);
data.actualBasePath = zeros(2,samplingNumber);
data.inputAngle = zeros(robot.inputNumber, samplingNumber);
data.varphi = zeros(2, samplingNumber);
data.JHat = zeros(robot.taskDimension, robot.inputNumber, samplingNumber);



%% Initialize Jacobian
data.inputAngle(:,1) = [ 0, 0, 0, 3.041, 3.045, -2.498];

data.JHat(:,:,1) = robot.Jacobian2(robot, data.inputAngle(:,1));
data.JReal(:,:,1) = robot.Jacobian2(robot, data.inputAngle(:,1));

%% Desired path
iota = param.scale;
Td = param.taskDuration;
data.initialPosition = robot.Position(robot, data.inputAngle(:,1));

data.desiredPath = [iota*cos(2*pi*(sin(0.5*pi*t/Td)).^2)-iota + data.initialPosition(1) + param.e0(1);
                     iota*sin(2*pi*(sin(0.5*pi*t/Td)).^2) + data.initialPosition(2) + param.e0(2);
                     1/2*iota*sin(2*pi*(sin(0.5*pi*t/Td)).^2) + data.initialPosition(3) + param.e0(3)];
data.desiredVelocity = [-(2*iota*pi^2*cos((pi*t)/(2*Td)).*sin((pi*t)/(2*Td)).*sin(2*pi*sin((pi*t)/(2*Td)).^2))/Td;
                     (2*iota*pi^2*cos((pi*t)/(2*Td)).*sin((pi*t)/(2*Td)).*cos(2*pi*sin((pi*t)/(2*Td)).^2))/Td;
                     1/2*(2*iota*pi^2*cos((pi*t)/(2*Td)).*sin((pi*t)/(2*Td)).*cos(2*pi*sin((pi*t)/(2*Td)).^2))/Td];

% data.desiredPath = [t*0 + data.initialPosition(1) + param.e0(1);
%                      t*0 + data.initialPosition(2) + param.e0(2);
%                      t*0 + data.initialPosition(3) + param.e0(3)];
% data.desiredVelocity = [t*0;
%                      t*0;
%                      t*0];

                