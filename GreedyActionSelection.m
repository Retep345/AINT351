function action = GreedyActionSelection(QTable, state, explorationRate)


if( rand < explorationRate)
   MaxAction = randi(4);
else
  MAX = QTable(state,1);
    for i = 1:4
        if (MAX <= QTable(state,i));
            MAX = QTable(state,i);
            MaxAction = i;
        end; %if
    end; %for    
    
end; %if-else

action = MaxAction;

end