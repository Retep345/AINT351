function [P1 P2] = RevoluteForwardKinematics2D(armLen, theta, origin)
% calculate relative forward kinematics
 
%armLen = [L1 L2]
%theta = [theta1 theta2]
%origin = [x0 y0]   base
%P1 = [xp1 yp1]     elbow
%P2 = [xp2 yp2]     end point

%keyboard
Theta12 = theta(:,1) + theta(:,2);

P1(:,1) = origin(1,1).* ones(size(theta(:,1)))  +armLen(1,1)*cos(theta(:,1));
P1(:,2) = origin(1,1).* ones(size(theta(:,1)))  +armLen(1,1)*sin(theta(:,1));


P2(:,1) = origin(1,1).* ones(size(theta(:,1)))  +(armLen(1,1)*cos(theta(:,1))) + (armLen(1,2)*cos(Theta12));
P2(:,2) = origin(1,2).* ones(size(theta(:,1)))  +(armLen(1,1)*sin(theta(:,1))) + (armLen(1,2)*sin(Theta12));


