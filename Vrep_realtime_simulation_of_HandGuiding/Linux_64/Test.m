
% Use this script to test connection between Vrep and Matlab.
% This project is used only to test the simulation, no connection to the
% real robot is done.

% To run the code:
% 1- First start the simulation scene (Sim_while_Robot_moving) in Vrep 
% 2- Run this script

% You shall see the robot moving its first joint in a sinusoidal cyclic
% motion

% Copyright: Mohammad Safeea 17-Oct-2019

%% Declare V-rep objects
vrep=remApi('remoteApi');
vrep.simxFinish(-1); 
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)     
    jHandles=zeros(7,1);
    for i=1:7
            s=['LBR_iiwa_7_R800_joint',num2str(i)];
            [res, daHandle]=vrep.simxGetObjectHandle(clientID,s,vrep.simx_opmode_oneshot_wait);
            jHandles(i)=daHandle;
    end
else 
    return;
end

jPos={0., pi / 180 * 20., 0., -pi / 180 * 70., 0.,...
                        pi / 180 * 90., 0.};
                    
for i =1:7
    errorCode=vrep.simxSetJointPosition(clientID,jHandles(i)...
        ,jPos{i},vrep.simx_opmode_buffer);
end                    
%% Control Robot Motion & Simulation 
w=2*pi/4; % Period is 4 seconds
alfa=pi/2;

tic
while toc<16
    t=toc;
    q=alfa*sin(w*t);
     jPos{1}=q;
    for i =1:7
%         simx_opmode_oneshot
        errorCode=vrep.simxSetJointPosition(clientID,jHandles(i)...
            ,jPos{i},vrep.simx_opmode_streaming );
    end
end

%% Stop simulation
vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait);
%% End simulation
vrep.simxFinish(clientID);
vrep.delete();
