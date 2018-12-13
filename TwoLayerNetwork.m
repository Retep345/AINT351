function [OutputA, W1, W2, error] = TwoLayerNetwork(X, W1, W2, Target, Alpha, finish)

%X is input
%W's are weights
%Target is desired output
%Alpha is learning rate

%loop until termination
%a2  1/(1+e^-w1x
%Out = w2*a2
%d3 = 2(Out-Target)
%d2 = w2T*d3*(a2(1-a2))
%de/dw1 = xT*d3
%de/dw2 = a2T*d3
%w1 = w1 - A*de/dw1
%w2 = w2 - A*de/dw2
%end loop



samples = size(X, 2);
 
for time = 1:finish
for idx = 1:samples
    
    
    XV = X(:, idx);
    TV = Target(:, idx);
    
    net2 = W1*XV;
    A2 = 1./(1+exp(-net2));
    aA2 = [A2;1];
    Output = W2*aA2;
    
    W2N = W2(:,1:(end-1));
    
    d3 = 2*(Output-TV);
    d2 = (W2N'*d3).*A2.*(1-A2);
    
    deltaeW1 = d2*XV';
    deltaeW2  = d3*aA2';
    
    W1 = W1 - Alpha*deltaeW1;
    W2 = W2 - Alpha*deltaeW2;
    
    OutputA(:,idx) = Output';
    
    errorSample(idx,:) = mean((TV-Output).^2);
end
 error(time) = mean(errorSample);
end



end
