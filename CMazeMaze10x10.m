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


classdef CMazeMaze10x10
    % define Maze work for RL
    %  Detailed explanation goes here
    
    properties
        
        % parameters for the gmaze grid management
        %scalingXY;
        blockedLocations;
        endLocation;
        cursorCentre;
        limitsXY;
        xStateCnt
        yStateCnt;
        stateCnt;
        stateNumber;
        totalStateCnt
        squareSizeX;
        cursorSizeX;
        squareSizeY;
        cursorSizeY;
        stateOpen;
        stateStart;
        stateStartID;
        stateEnd;
        stateEndID;
        stateX;
        stateY;
        xS;
        yS
        stateLowerPoint;
        textLowerPoint;
        stateName;
        
        % parameters for Q learning
        QValues;
        tm;
        actionCnt;
    end
    
    methods
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % constructor to specity maze
        function f = CMazeMaze10x10(limitsXY, blockage, EndXY)
            
            % set scaling for display
            f.limitsXY = limitsXY;
            f.blockedLocations = blockage;
            f.endLocation = EndXY;
            f.stateStartID = f.RandomStartingState();
            
            % setup actions
            f.actionCnt = 4;
            
            % build the maze
            f = SimpleMaze10x10(f);
            
            % display progress
            disp(sprintf('Building Maze CMazeMaze10x10'));
            
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % build the maze
        function f = SetMaze(f, xStateCnt, yStateCnt, blockedLocations, startLocation, endLocation)
            
            % set size
            f.xStateCnt=xStateCnt;
            f.yStateCnt=yStateCnt;
            f.stateCnt = xStateCnt*yStateCnt;
            
            % compute state countID
            for x =  1:xStateCnt
                for y =  1:yStateCnt
                    
                    % get the unique state identified index
                    ID = x + (y -1) * xStateCnt;
                    
                    % record it
                    f.stateNumber(x,y) = ID;
                    
                    % also record how x and y relate to the ID
                    f.stateX(ID) = x;
                    f.stateY(ID) = y;
                end
            end
            
            % calculate maximum number of states in maze
            % but not all will be occupied
            f.totalStateCnt = f.xStateCnt * f.yStateCnt;
            
            
            % get cell centres
            f.squareSizeX= 1 * (f.limitsXY(1,2) - f.limitsXY(1,1))/f.xStateCnt;
            f.cursorSizeX = 0.5 * (f.limitsXY(1,2) - f.limitsXY(1,1))/f.xStateCnt;
            f.squareSizeY= 1 * (f.limitsXY(2,2) - f.limitsXY(2,1))/f.yStateCnt;
            f.cursorSizeY = 0.5 * (f.limitsXY(2,2) - f.limitsXY(2,1))/f.yStateCnt;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % init maze with no closed cell
            f.stateOpen = ones(xStateCnt, yStateCnt);
            f.stateStart = startLocation;
            f.stateEnd = endLocation;
            f.stateEndID = f.stateNumber(f.stateEnd(1),f.stateEnd(2));
            
            % put in blocked locations
            for idx = 1:size(blockedLocations,1)
                bx = blockedLocations(idx,1);
                by = blockedLocations(idx,2);
                f.stateOpen(bx, by) = 0;
            end
            
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % get locations for all states
            for x=1:xStateCnt
                for y=1:xStateCnt
                    
                    % start at (0,0)
                    xV = x-1;
                    yV = y-1;
                    
                    % pure scaling component
                    % assumes input is between 0 - 1
                    scaleX =  (f.limitsXY(1,2) - f.limitsXY(1,1)) / xStateCnt;
                    scaleY = (f.limitsXY(2,2) - f.limitsXY(2,1)) / yStateCnt;
                    
                    % remap the coordinates and add on the specified orgin
                    f.xS(x) = xV  * scaleX + f.limitsXY(1,1);
                    f.yS(y) = yV  * scaleY + f.limitsXY(2,1);
                    
                    % remap the coordinates, add on the specified orgin and add on half cursor size
                    f.cursorCentre(x,y,1) = xV * scaleX + f.limitsXY(1,1) + f.cursorSizeX/2;
                    f.cursorCentre(x,y,2) = yV * scaleY + f.limitsXY(2,1) + f.cursorSizeY/2;
                    
                    f.stateLowerPoint(x,y,1) = xV * scaleX + f.limitsXY(1,1);  - f.squareSizeX/2;
                    f.stateLowerPoint(x,y,2) = yV * scaleY + f.limitsXY(2,1); - f.squareSizeY/2;
                    
                    f.textLowerPoint(x,y,1) = xV * scaleX + f.limitsXY(1,1)+ 10 * f.cursorSizeX/20;
                    f.textLowerPoint(x,y,2) = yV * scaleY + f.limitsXY(2,1) + 10 * f.cursorSizeY/20;
                end
            end
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % draw rectangle
        function DrawSquare( f, pos, faceColour)
            % Draw rectagle
            rectangle('Position', pos,'FaceColor', faceColour,'EdgeColor','k', 'LineWidth', 3);
        end
        
        % draw circle
        function DrawCircle( f, pos, faceColour)
            % Draw rectagle
            rectangle('Position', pos,'FaceColor', faceColour,'Curvature', [1 1],'EdgeColor','k', 'LineWidth', 3);
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % draw the maze
        function DrawMaze(f)
            figure('position', [100, 100, 1200, 1500]);
            fontSize = 20;
            hold on
            h=title(sprintf('ISH: Maze wth %d x-axis X %d y-axis cells', f.xStateCnt, f.yStateCnt));
            set(h,'FontSize', fontSize);
            
            for x=1:f.xStateCnt
                for y=1:f.yStateCnt
                    pos = [f.stateLowerPoint(x,y,1)  f.stateLowerPoint(x,y,2)  f.squareSizeX f.squareSizeY];
                    
                    % if location open plot as blue
                    if(f.stateOpen(x,y))
                        DrawSquare( f, pos, 'b');
                        % otherwise plot as black
                    else
                        DrawSquare( f, pos, 'k');
                    end
                end
            end
            
            
            % put in start locations
            for idx = 1:size(f.stateStart,1)
                % plot start
                x = f.stateStart(idx, 1);
                y = f.stateStart(idx, 2);
                x
                y
                pos = [f.stateLowerPoint(x,y,1)  f.stateLowerPoint(x,y,2)  f.squareSizeX f.squareSizeY];
                DrawSquare(f, pos,'g');
            end
            
            % put in end locations
            for idx = 1:size(f.stateEnd,1)
                % plot end
                x = f.stateEnd(idx, 1);
                y = f.stateEnd(idx, 2);
                pos = [f.stateLowerPoint(x,y,1)  f.stateLowerPoint(x,y,2)  f.squareSizeX f.squareSizeY];
                DrawSquare(f, pos,'r');
            end
            
            % put on names
            for x=1:f.xStateCnt
                for y=1:f.yStateCnt
                    sidx=f.stateNumber(x,y);
                    stateNameID = sprintf('%s', f.stateName{sidx});
                    text(f.textLowerPoint(x,y,1),f.textLowerPoint(x,y,2), stateNameID, 'FontSize', 20)
                end
            end
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % setup 10x10 maze
        function f = SimpleMaze10x10(f)
            
            xCnt=10;
            yCnt=10;
            
            % specify start location in (x,y) coordinates
            % example only
            chosenstart = f.stateStartID;
            startLocation= f.IDtoStatePosition(chosenstart);
            
            
            % YOUR CODE GOES HERE
            
            
            % specify end location in (x,y) coordinates
            % example only
            endLocation= f.endLocation;
            % YOUR CODE GOES HERE
            
            
            % specify blocked location in (x,y) coordinates
            % example only
            blockedLocations = f.blockedLocations; %[1 1; 2 2; 3 3;];
            % YOUR CODE GOES HERE
            
            % build the maze
            f = SetMaze(f, xCnt, yCnt, blockedLocations, startLocation, endLocation);
            
            % write the maze state
            maxCnt = xCnt * yCnt;
            for idx = 1:maxCnt
                f.stateName{idx} = num2str(idx);
            end
            
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % reward function that takes a stateID and an action
        function reward = RewardFunction(f, stateID, action)
            
            % init to no reward
            reward = 0;
            
            %find the ID of the end point
            xEnd = f.endLocation(1,1);
                    yEnd = f.endLocation(1,2);
                    endID = xEnd + ((yEnd-1)*10);
            
                    
            % with ID 100, only two state-action pairs get to the end point
            % however, a different endpoint can be chosen, so all rewards
            % are set
            if ((stateID == (endID - 10)) && (action == 1))
                reward = 10;
            end;
            
            if ((stateID == (endID - 1)) && (action == 2))
                reward = 10;
            end;
            if ((stateID == (endID + 10)) && (action == 3))
                reward = 10;
            end;
            if ((stateID == (endID + 1)) && (action == 4))
                reward = 10;
            end;
            
            
            
         end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % function  computes a random starting state
        function startingState = RandomStartingState(f)
           
            Value = 1;
            %xBlock = 0;
            %yBlock = 0;
            while (Value == 1)
                valCheck = 0;
                startingState = randi(100) ;
             
                d = size(f.blockedLocations);
               
                for blocked = 1:(d(1))
                    xBlock = f.blockedLocations(blocked,1);
                    yBlock = f.blockedLocations(blocked,2);
                    xyBlocked = xBlock + ((yBlock-1)*10);
                         if (startingState ~= xyBlocked)
                         valCheck = valCheck + 1;
                         end;
                end;
                    xBlock = f.endLocation(1,1);
                    yBlock = f.endLocation(1,2);
                    xyBlocked = xBlock + ((yBlock-1)*10);
                         if (startingState ~= xyBlocked)
                         valCheck = valCheck + 1;
                         end;
                
                    if valCheck == d(1) +1
                        Value = 0;
                    else Value = 1;
                    end;
            end;
        end
        
         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % function  translates the 1-100 ID to the XY matrix
        function stateXY = IDtoStatePosition(f, SpecificstateID)
            
                chosenID = SpecificstateID;
              %  stateX = 0;
                stateY = 1;
            while (chosenID >= 11)
                chosenID = chosenID - 10;
                stateY = stateY + 1;
            end;
                stateX = chosenID;
                
                stateXY = [stateX, stateY];
            
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % look for end state
        function endState = IsEndState(f, x, y)
            
            % default to not found
            endState=0;
            
            % YOUR CODE GOES HERE ....
            
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % init the q-table
        function f = InitQTable(f, minVal, maxVal)
            
            % allocate
            f.QValues = zeros(f.xStateCnt * f.yStateCnt, f.actionCnt);        

            range = maxVal - minVal;
            mean = (minVal+maxVal)/2;

            y = range * (rand((f.xStateCnt * f.yStateCnt),f.actionCnt)-0.5) + mean;
            f.QValues =y;
           
        end
        
        

        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % % build the transition matrix
        % look for boundaries on the grid
        % also look for blocked state
        function f = BuildTransitionMatrix(f)
            
            %actions 1234 are NESW
            % allocate
            f.tm = zeros(f.xStateCnt * f.yStateCnt, f.actionCnt);
                        %makes an XY by A matrix (in current setup 100 x 4)
                                  
            %theoretically findNext = f.tm(state,action)
            for StateIDpoint = 1:(f.xStateCnt * f.yStateCnt)
      
            %step 1: setup a default transition matrix
            %that will have points assume no blockage or wall 
                
                f.tm(StateIDpoint,1) = StateIDpoint + f.xStateCnt;
                f.tm(StateIDpoint,2) = StateIDpoint + 1;
                f.tm(StateIDpoint,3) = StateIDpoint - f.xStateCnt;
                f.tm(StateIDpoint,4) = StateIDpoint - 1;
                
                
                %step 2: over-write with blocked positions as self
                
                %for start
                for ActionMove = 1:4
                    
                    d = size(f.blockedLocations);
                    for blocked = 1:(d(1))
                        xBlock = f.blockedLocations(blocked,1);
                        yBlock = f.blockedLocations(blocked,2);
                        xyBlocked = xBlock + ((yBlock-1)*10);
                             if (f.tm(StateIDpoint,ActionMove) == xyBlocked)
                            f.tm(StateIDpoint,ActionMove) = StateIDpoint;
                            end;
                    end;
                end; 
                %for end
                
                
                %step 3: fix end wall locations
                
                
                stateXY = IDtoStatePosition(f,StateIDpoint);
                
                %go north off map (up)
                    if ((stateXY(1,2)+1) == 11)
                        f.tm(StateIDpoint,1) = StateIDpoint;
                    end;
                %go east off map (right)
                    if ((stateXY(1,1)+1) == 11)
                        f.tm(StateIDpoint,2) = StateIDpoint;
                    end;
                %go south off map (down)
                    if ((stateXY(1,2)-1) == 0)
                        f.tm(StateIDpoint,3) = StateIDpoint;
                    end;
                %go west off map (left)
                    if ((stateXY(1,1)-1) == 0)
                        f.tm(StateIDpoint,4) = StateIDpoint;
                    end;
                
            end;
                      
        end
        
    end
end





















