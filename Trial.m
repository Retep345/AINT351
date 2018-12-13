function [steps, Q] = Trial(MazeIn, Q, startingPoint, EndPoint, alpha, gamma, episodes, explorationRate)

% set termination state
xEnd = EndPoint(1,1);
yEnd = EndPoint(1,2);
   xyEnd = xEnd + ((yEnd-1)*10);
terminationState = xyEnd;
            
% trial function that runs episodes.
for tidx = 1:episodes
% run an episode and record steps
[Q, steps(tidx)] = Episode(MazeIn, Q, startingPoint, alpha, gamma, explorationRate,terminationState);
end

end