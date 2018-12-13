%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% all rights reserved
% Author: Dr. Ian Howard
% Associate Professor (Senior Lecturer) in Computational Neuroscience
% Centre for Robotics and Neural Systems
% Plymouth University
% A324 Portland Square
% PL4 8AA
% Plymouth, Devon, UK
% howardlab.com
% 22/09/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% run maze experiments
% you need to expand this script to run the assignment

close all
clear all
clc

% defining initial values
limits = [0 1; 0 1;];
blockPosition = [5 1; 9 1;              %0
                 2 2; 3 2;              %1
                 3 3; 4 3; 6 3; 9 3;    %2
                 1 4; 4 4; 9 4;         %3
                 5 5; 7 5; 9 5;         %4
                 3 6; 5 6; 7 6; 9 6;    %5
                 2 7; 3 7; 7 7; 9 7;    %6
                 3 8; 6 8; 7 8; 9 8;    %7
                 3 9; 7 9;              %8
                 7 10;] %#x2            %9
                    %X by Y
endPoint = [10 10];

% build the maze
maze = CMazeMaze10x10(limits,blockPosition, endPoint);

% draw the maze
maze.DrawMaze();

xy = maze.cursorCentre(2,2, :);

%maze.DrawCircle([xy(1,1) xy(1,2) 0.05 0.05], 'r')

plot(0.5, 0.5, 'xy')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% YOU NEED TO DEFINE THESE VALUES
% init the q-table
minVal = 0.01;
maxVal = 0.1;
maze = maze.InitQTable(minVal, maxVal);

% test values
state = 1;
action = 1;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% this will be used by Q-learning as follows:
q = maze.QValues(state, action);

qTable = maze.QValues;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% YOU NEED TO FINISH OFF THIS FUNCTION
% get the reward from the action on the surrent state
% this will be used by Q-learning as follows:
reward = maze.RewardFunction(state, action);

    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% YOU NEED TO FINISH OFF THIS FUNCTION
% build the transition matrix
maze = maze.BuildTransitionMatrix();
% print out values
maze.tm

% get the next state due to that action
% this will be used by Q-learning as follows:
resultingState = maze.tm(state, action);

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % test random start
% startingState = maze.RandomStartingState();
% % print out value
% startingState



alpha = 0.2;
gamma = 0.9;
explorationRate = 0.1;
episodes = 1000;
trials = 10;
startingPoint = maze.stateStartID;

xEnd = endPoint(1,1);
yEnd = endPoint(1,2);
   xyEnd = xEnd + ((yEnd-1)*10);
terminationState = xyEnd;

[TrialEpisodeSteps, Q] = Experiment(maze, qTable, startingPoint, endPoint, alpha, gamma, trials, episodes, explorationRate);



state = startingPoint;
% loop that runs until the goal state is reached
running=1;
steps=0;

while(running==1)
% your code here in code step
action = GreedyActionSelection(Q, state, 0);

% get the next state due to that action
nextState = maze.tm(state,action);

% get the reward from the action on the current state

% termination if reaches goal state
if (nextState == terminationState);
    running = 0;
end;
    
% update steps
steps = steps +1;


%plotting on maze
StepPlot(steps) = state;
XYstate = maze.IDtoStatePosition(state);
xy = maze.cursorCentre(XYstate', :);
maze.DrawCircle([xy(1,1) xy(2,1) 0.05 0.05], 'r')


%update the state
state = nextState;

if (steps == 400);
    running = 0;
end;
end



 Mu = mean(TrialEpisodeSteps');
StandardDev = std(TrialEpisodeSteps');


figure
hold on
title('Standard Deviation and Mean');
errorbar(Mu, StandardDev)
xlabel('episode number');
ylabel('number of steps taken');





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

armLen = [0.5 0.5];     % L1, L2
theta = [1.5 -1.5];  % thetaOne, thetaTwo  in radians
origin = [0 0];
samples = 10;
massOrigin = origin(1,1).* ones(samples,2);
angles = (3.14).*rand(samples,2);

%manual calibration [P1 P2] = RevoluteForwardKinematics2D(armLen, theta, origin)

   [P1, P2] = RevoluteForwardKinematics2D(armLen, angles, origin);
   
XP = [P1(:,1) P2(:,1)];
YP = [P1(:,2) P2(:,2)];


XOP = [massOrigin(:,1) P1(:,1)];
YOP = [massOrigin(:,2) P1(:,2)];




%%%%% neural network
testSamples = 1000;
testTheta = (3.14).*rand(testSamples,2);  %radians
[PTest1, PTest2] = RevoluteForwardKinematics2D(armLen, testTheta, origin);

%the point of the network is to get the angle from the endpoint
Target = testTheta';
onerow = ones(1,testSamples);
X = [PTest2'; onerow];
hidden = 10;

W1 = 0.2 * randn(hidden,3);
W2 = 0.2 * randn(2,hidden+1);

Alpha = 0.01;
finish = 400;

[OutputA, WO1, WO2, ErrorOut] = TwoLayerNetwork(X, W1, W2, Target, Alpha, finish);



[P1X, P2X] = RevoluteForwardKinematics2D(armLen, OutputA', origin);

%comparing the test data and output reverse kinematic
figure
hold on
title('full arm positions')
h=plot(PTest2(:,1),PTest2(:,2),'b.'); % xy points of the end effector
     h.MarkerSize=20;
     
h=plot(P2X(:,1),P2X(:,2),'r.'); % xy points of the end effector
     h.MarkerSize=20;

xlabel('x[meters]');
ylabel('y[meters]');
h=plot(origin(:,1),origin(:,2),'k*');
     h.MarkerSize=10;
     h.LineWidth=2;

figure
hold on
h=plot(ErrorOut,'b.');

%plot comparing output and target
figure
hold on
title('target vs output')
h=plot(OutputA(1,:), OutputA(2,:),'r.'); % xy points of output angles
     h.MarkerSize=10;
xlabel('radians');
ylabel('radians');
     
h=plot(Target(1,:), Target(2,:),'k.'); % xy points of target angles

testThetaX = (3.14).*rand(1,2);  %radians
[PTest1X, PTest2X] = RevoluteForwardKinematics2D(armLen, testThetaX, origin);

%the point of the network is to get the angle from the endpoint
Target = testThetaX';
Xin = [PTest2X'; 1];

%recognition 
    net2 = WO1*Xin;
    A2 = 1./(1+exp(-net2));
    aA2 = [A2;1];
    OutputX = WO2*aA2;
         

%%%%%%%%%%%%%%%%%%%%%%
% histogram of random starting point options
for arrayPoint = 1:2000
RandomValues(arrayPoint) = maze.RandomStartingState;
end

figure
hold on
histogram(RandomValues,100)



% %true test output
% figure
% hold on
% h=plot(P4(:,1),P4(:,2),'r.');
%      h.MarkerSize=5;
%      
% h=plot(origin(:,1),origin(:,2),'k*');
%      h.MarkerSize=10;
%      h.LineWidth=2;
% 
% %test network output
% figure
% hold on
% j=plot(P6(:,1),P6(:,2),'r.');
%      j.MarkerSize=5;
%      
% j=plot(origin(:,1),origin(:,2),'k*');
%      j.MarkerSize=10;
%      j.LineWidth=2;




%%%%%%%%%%%%%%%%%%
% %arm model graph
% figure
% hold on
% title('full arm positions')
% h=plot(P2(:,1),P2(:,2),'r.'); % xy points of the end effector
%      h.MarkerSize=20;
% h=plot(P1(:,1),P1(:,2),'g.'); % xy points of the elbow
%      h.MarkerSize=20;
% xlabel('x[meters]');
% ylabel('y[meters]');
%      
%      for index = 1:samples %for each sample
%          
% plot(XP(index,:), YP(index,:), 'b'); %plot a line between the end and elbow
% plot(XOP(index,:), YOP(index,:), 'b'); %plot a line between elbow and origin
% 
%      end
%      
%      h=plot(origin(:,1),origin(:,2),'k*'); % xy points of the origin
%      h.MarkerSize=10;
%      h.LineWidth=2;



% %%%%%
% %copy of default plotter
% figure
% hold on
% title('sample arm end-effector positions')
% h=plot(P2(:,1),P2(:,2),'r.'); % xy points of the end effector
%      h.MarkerSize=10;
% xlabel('x[meters]');
% ylabel('y[meters]');
%      
% h=plot(origin(:,1),origin(:,2),'k*'); % xy points of origin
%      h.MarkerSize=10;
%      h.LineWidth=2;
     