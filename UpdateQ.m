function Q = UpdateQ(Q, state, action, nextState, reward, alpha, gamma)

%Q(s,a) <--- Q(s,a) + alpha[(r + gammaMaxQ(s',a') - Q(s,a)

bestNextAction = GreedyActionSelection(Q,nextState,0);

bestNextValue = Q(nextState,bestNextAction);

Q(state, action) = Q(state, action) + alpha*(reward + gamma * bestNextValue - Q(state, action));


end
