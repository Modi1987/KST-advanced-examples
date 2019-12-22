%% Example of using Kuka Sunrise Toolbox for interfacing with KUKA iiwa robots

% Copyright: Mohammad SAFEEA, 21st of December 2019

% This script is used to:
% Show the motion of the real robot (KUKA iiwa) in V-rep (3D simulation) in
% realtime while performing handguiding in joint space.
% This script works together with the V-rep scene:
% "Sim_J_hg.ttt"
close all;clear;clc;
global t_Kuka;
%% Requirments:
% 1- Matlab
% 2- Vrep
% 3- KUKA Sunrise Toolbox with java code in (KST_1.6_iiwa7R800_pneumatic) synchronized
%    to the robot controller using the Sunrise.WOrkbench
% 4- You also have to update the path of the KST in the variable "cDir" below
cDir = 'C:\Users\ModiSaf\Desktop\KST-advanced-examples\KST-Kuka-Sunrise-Toolbox-master\Matlab_client';

%% To run this example do the following:
% 1- From Vrep run the simulation: Sim_J_hg.ttt.
% 2- Run the (MarlabToolboxServer) application on the robot.
% 3- Run this script from MATLAB.

%% Initiation part of the code
warning('off')
disp('Program started');
disp('--------------------------')
% Add path of KST class to work space
disp('The path specified for the KUKA Sunrise Toolbox (KST) in this script is:');
disp(cDir);
disp('--------------------------')
filename=[cDir,'\KST.m'];
if isfile(filename)
     % KST.m exists in the specified path.
     addpath(cDir);
else
     % KST.m does not exist in the specified path.
     message=['Could not find the KUKA Sunrise Toolbox in the specified path',...
         '\n',...
         'You have to modify the value of the variable "cDir" in this script with the correct path for the KUKA Sunrise Toolbox on your computer',...
         '\n'];
     fprintf(2,message)
     disp('If you do not have the Toolbox available on your PC then download it from:')
     disp('<a href = "https://github.com/Modi1987/KST-Kuka-Sunrise-Toolbox">The GitHub repo: www.github.com/Modi1987/KST-Kuka-Sunrise-Toolbox</a>')
     return;
end

%% Instantiate a V-rep object
message='Make sure that you started the simulation scene "Sim_J_hg" from inside Vrep';
disp(message);
try
    confirmationInput=input('Press enter to continue');
catch
end
disp('Trying to connect to the simulation scene "Sim_J_hg.ttt"')
vrep=remApi('remoteApi');
vrep.simxFinish(-1); 
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)  
    disp('Connected to Vrep successfully')
    disp('--------------------------')
    disp('Run the MatlabToolboxServer application on the SmartPad of the robot')
    try
        confirmationInput=input('Then press enter to continue');
    catch
    end
    % Start a connection with the MatlabToolboxServer application on the
    % SmartPad of the KUKA iiwa robot
    try
        disp('--------------------------')
        disp('Trying to establish a connection with KUKA iiwa robot')
	% Connect to KUKA iiwa
        ip='172.31.1.147'; % The IP of the controller
        t_Kuka=net_establishConnection( ip ); % start a connection with the server
    catch
        fprintf(2,'Error could not connect to KUKA iiwa\n')
        fprintf(2,'Script aborted\n')
        % Stop the simulation
        vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait);
        vrep.simxFinish(clientID);
        vrep.delete();
        return;
    end
    % if connection is established successfully with the robot and the
    % simulation, then get the handles of the joints from the simulation
    jHandles=zeros(7,1);
    for i=1:7
            s=['LBR_iiwa_7_R800_joint',num2str(i)];
            [res, daHandle]=vrep.simxGetObjectHandle(clientID,s,vrep.simx_opmode_oneshot_wait);
            jHandles(i)=daHandle;
    end
else 
    fprintf(2,'Error could not connect to Vrep simulation\n')
    return;
end

disp('--------------------------')
try
    confirmationInput=input('Press enter to start Hand guiding in joint space, press ctrl+c to terminate the program');
catch
end
%% Start the HG in joint space
theCommand='handGuiding';
fprintf(t_Kuka, theCommand);
% Print some info into the screen
fprintf('Hand gudining functionality is started \n');
fprintf('1- Press white button to activate the hand guiding \n');
fprintf('2- Release the white button so the robot stops in its configuration \n');
fprintf('3- Release the white button so the robot stops in its configuration \n');
fprintf('4- To reactivate the handguding functionality, press the green button once, for less than 1 sec  \n');
fprintf('5- Repeat from 1  \n');
fprintf('6- To terminate the hand guding function, press the green button for more than 1.5 sec \n keep pressing until the red light starts to flicker then release your hand, \n the hand guiding mode will be terminate\n ');

% Update Homing Position of simulation according to current configuration
% of the robot
jPos  = getJointsPos(t_Kuka);
for i =1:7
    errorCode=vrep.simxSetJointPosition(clientID,jHandles(i)...
    ,jPos{i},vrep.simx_opmode_buffer);
end
% start the hand-guiding asynchronously
% Control loop
messageAscii=[];
loopFlag=true;
while loopFlag
	% Get the joint angles of the currnet configuration of the robot,
	% and update the configuration of the robot in the simulation accordingly
    try
        jPos  = getJointsPos(t_Kuka);
        for i =1:7
            errorCode=vrep.simxSetJointPosition(clientID,jHandles(i)...
            ,jPos{i},vrep.simx_opmode_streaming);
        end
    catch
        %% when error happens, it is because the user turned off the Hand-Guiding mode
        loopFlag=false;
        disp('Program terminated');
    end
end 
%% Turn off KST server
net_turnOffServer(t_Kuka);
%% Stop simulation
vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait);
%% End simulation
vrep.simxFinish(clientID);
vrep.delete();
