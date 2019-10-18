%% Example of using KST class for interfacing with KUKA iiwa robots

% This script is used to:
% Show the motion of the real robot (KUKA iiwa) in 3D simulation in
% realtime while performing point to point motions.
% This script works together with the V-rep scene:
% "Sim_while_Robot_moving.ttt"
close all;clear;clc;

%% Requirments:
% 1- Matlab
% 2- Vrep
% 3- KUKA Sunrise Toolbox (KST)
% 4- You also have to update the path of the KST in the variable "cDir" below
cDir = '~/Downloads/KST-Kuka-Sunrise-Toolbox-master/Matlab_client';

%% To run this example do the following:
% 1- From Vrep run the simulation: Sim_while_Robot_moving.ttt.
% 2- Run the (MarlabToolboxServer) application on the robot.
% 3- Run this script from MATLAB.

% Copyright: Mohammad SAFEEA, 17th of October 2019

%% Initiation part of the code
warning('off')
disp('Program started');
disp('--------------------------')
% Add path of KST class to work space
disp('The path specified for the KUKA Sunrise Toolbox (KST) in this script is:');
disp(cDir);
disp('--------------------------')
filename=fullfile(cDir,'KST.m');
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
message='Make sure that you started the simulation scene "Sim_while_Robot_moving" from inside Vrep';
disp(message);
try
    confirmationInput=input('Press enter to continue');
catch
end
disp('Trying to connect to the simulation scene "Sim_while_Robot_moving.ttt"')
vrep=remApi('remoteApi');
vrep.simxFinish(-1); 
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

%% Instantiate the KST object
ip='172.31.1.147'; % The IP of the controller
arg1=KST.LBR7R800; % choose the robot iiwa7R800 or iiwa14R820
arg2=KST.Medien_Flansch_elektrisch; % choose the type of flange
Tef_flange=eye(4); % transofrm matrix of EEF with respect to flange
iiwa=KST(ip,arg1,arg2,Tef_flange); % create the object

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
        flag=iiwa.net_establishConnection();
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
disp('The robot will move using point to point motion functions')
disp('into various configurations while updating the simulation in realt-time')
disp('MAKE SURE THAT THE ROBOT IS COLLISION FREE')
try
    confirmationInput=input('Then press enter to continue');
catch
end
%% Get the joint angles of the currnet configuration of the robot,
% and update the configuration of the robot in the simulation accordingly
jPos  = iiwa.getJointsPos();
for i =1:7
    errorCode=vrep.simxSetJointPosition(clientID,jHandles(i)...
        ,jPos{i},vrep.simx_opmode_buffer);
end              
%% Various configuration of first joint
j0_Pos={0,pi/4,-pi/4,0,pi/3,-pi/3,0};
%% Control robot motion, update simulation in real-time
jPos={0., pi / 180 * 20., 0., -pi / 180 * 70., pi / 180 * 90.,...
                        pi / 180 * 90., 0.};
relVel=0.15;
for j=1:7
    jPos{1}=j0_Pos{j};
    iiwa.nonBlocking_movePTPJointSpace(jPos, relVel);
    motionFlag=false;
    while ~motionFlag
        param=4; % sepecify feedback as the joints angles
        [motionFlag,feedBack]=iiwa.nonBlockingCheck_WithFeedback(param);
        for i =1:7
            % Update Simulation with Robot Position 
            errorCode=vrep.simxSetJointPosition(clientID,jHandles(i)...
                ,feedBack{i},vrep.simx_opmode_streaming );
        end
    end
    jPos{5}=0;
end
%% Turn off KST server
iiwa.net_turnOffServer();
%% Stop simulation
vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait);
%% End simulation
vrep.simxFinish(clientID);
vrep.delete();
