function [Q, steps] = Episode(MazeIn, Q, startingPoint, alpha, gamma, explorationRate,terminationState)
% implements a Q-learning episode
% initialize state to a random starting state at the start of each episode
state = startingPoint;
% loop that runs until the goal state is reached
running=1;
steps=0;


while(running==1)
% your code here in code step
action = GreedyActionSelection(Q, state, explorationRate);

% get the next state due to that action
nextState = MazeIn.tm(state,action);

% get the reward from the action on the current state
reward = MazeIn.RewardFunction(state, action);

% update the Q- table
Q = UpdateQ(Q, state, action, nextState, reward, alpha, gamma);

% termination if reaches goal state
if (nextState == terminationState);
    running = 0;
end;
    
% update steps
steps = steps +1;
% update the state
state = nextState;

if (steps == 1000);
    running = 0;
end;
end
