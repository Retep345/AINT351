function [stepsAcrossTrials, Q] = Experiment(MazeIn, Q, startingPoint, EndPoint, alpha, gamma, trialCnt, episodeCnt, explorationRate)


for trials = 1:trialCnt
       [stepsAcrossTrials(:,trials), Q] = Trial(MazeIn, Q, startingPoint, EndPoint, alpha, gamma, episodeCnt, explorationRate);


end