%    Title: BirdOidObjectAvoidance
%    Author: Daniel DeBord
%    Date: Last Updated July 21, 2020
%    Code version: 1.1
%    Availability: https://github.com/ddebord3/RobotariumSwarmBots

clear;
close all; 
clc;

%% Initialize Robotarium and Locations
sizeSwarm1 = 3;
sizeSwarm2 = 3;
N=sizeSwarm1+sizeSwarm2+2;                        % Number of agents
setBoids1 = [1, 3, 4, 5]; % the set of boids in swarm 1
setBoids2 = [2, 6, 7, 8]; % set of boids in swarm 2
max_iter = 4500;            % How many iterations to run the code for. Each iteration is about 0.033s.

videoFLag = 1;                        

bound = 0.9;
T = 1/30;
avoidWall = 1;
wObsMax = 2.0;
alphaV = 0.5;

% Set up data to be saved
% the data is stored in such a way that for [n, i, j]
% n represents the nth robot, the i chooses from [x;y;theta]
% j is the time (only takes from one out of every 10
savedRobotData = zeros(N, 3, max_iter/10);
% saves data in [v;omega] format
savedControllerData = zeros(N, 2, max_iter/10);

repeats = 5;
% Path constants
tList = 0:T:(max_iter)*T;
squareBound = 0.5;
%x2Vec = [linspace(squareBound, squareBound, max_iter/(repeats*4)), linspace(squareBound, -squareBound, max_iter/(repeats*4)), linspace(-squareBound, -squareBound, max_iter/(repeats*4)), linspace(-squareBound, squareBound, max_iter/(repeats*4))];
%y2Vec = [linspace(-squareBound, squareBound, max_iter/(repeats*4)), linspace(squareBound, squareBound, max_iter/(repeats*4)), linspace(squareBound, -squareBound, max_iter/(repeats*4)), linspace(-squareBound, -squareBound, max_iter/(repeats*4))];
%x2Vec = [repmat(x2Vec, 1, repeats), 0.8*ones(1,1)];
%y2Vec = [repmat(y2Vec, 1, repeats), -0.8*ones(1,1)];


xOfT = [1.0*cos(tList.*0.1+pi); 1.0*cos(tList.*0.1)];
yOfT = [0.7*sin(tList.*0.1+pi); 0.7*sin(tList.*0.1)];

alpha = 1;
beta = 1;
speedMax = 0.17;
wMax = 0.6;
predatorDetectRange = 0.30;
obsDetectRange = inf;

constant = 1;

initialPos = [-1.2, 1.2, -0.5, -0.5, -1.0, 1.0, 0.5, 0.7;
                  0.0, 0.0,  -0.7, 0.7, 0.4, -0.5, -0.7, -0.4;
                  0, -pi/2, pi/4, -pi/4, -pi/2, pi/2, 0, 0];
              
% Initialize robotarium
r = Robotarium('NumberOfRobots', N, 'InitialConditions', initialPos);

si_to_uni_dyn = create_si_to_uni_dynamics();                      
uni_barrier_cert = create_uni_barrier_certificate_with_boundary(); 

%% Initialize Poses and Weights
% Get initial poses of robots
xuni = r.get_poses();                                    % States of real unicycle robots
x = xuni(1:2,:);                                         % x-y positions only
r.step();                                               % Run robotarium step

speed = 0.07;
gamma = 0.05;
separationDist = 0.3; % distance objects should be from one another
separationConst = 0.1;
alignmentConst = .1;
cohesionDist = 5; % distance objects need to be within to stay cohesive
cohesionConst = 20;

gammaPredator = 1;
predatorMinumumDistance = 0.15; % radius used to avoid boids in another swarm or another
% moving object

targetPos = [0.8; 0.8]; % the final target position for leader robots
numLeaders = 2;

% leader robot i has [x_target; y_target] in the ith column
% Initialize video
if videoFLag 
    vid = VideoWriter('MessyObstacleAvoidance.mp4', 'MPEG-4');
    vid.Quality = 100;
    vid.FrameRate = 72;
    open(vid);
    writeVideo(vid, getframe(gcf));
end

%% Set up Obstacles
gammaObs = 0.05;
obstacleMinimumDistance = 0.2; % minimum distance to an obstacle tolerated
obsNumber = 1;
obs = [0.0, 0.8, 0.0; 
       -0.1, -0.2, 0.3;
       0.1, 0.2,  0.3]; % obstacles are stored in 3xobsNumber array
% the form is [x; y; radius] for each obstacle, all obstacles are circle
line_width = 3; % width of a line in the environment
obsColor = 'g';
for index = 1:obsNumber
    obsSize = 17.5; % size of obstacles in the environment
    obsPlot = plot(obs(1,index) , obs(2,index), 'o','MarkerSize',obsSize,'LineWidth', ...
        line_width,'Color',obsColor);
end

newMarkerSize = 52.5;
newObsMarkerSize = 52.5;
guideMarkerSize = 17.5;
i = 1;
guide(1) = plot(xOfT(1,1), yOfT(1,1), '*','MarkerSize', guideMarkerSize,'LineWidth',5,'Color','k');
guide(2) = plot(xOfT(2,1), yOfT(2,1), '*','MarkerSize', guideMarkerSize,'LineWidth',5,'Color',[0.5, 0.5, 0.99]);

g(1) = plot(x(1,i),x(2,i),'o','MarkerSize', newMarkerSize,'LineWidth',5,'Color','k');
obstacleDrawRadius(1) = plot(x(1,i),x(2,i),'o','MarkerSize', newObsMarkerSize,'LineWidth',5,'Color','k');
g(2) = plot(x(1,i),x(2,i),'o','MarkerSize', newMarkerSize,'LineWidth',5,'Color',[0.9, 1.0, 0.6]);
obstacleDrawRadius(2) = plot(x(1,i),x(2,i),'o','MarkerSize', newObsMarkerSize,'LineWidth',5,'Color',[0.5, 0.5, 0.99]);
for i = numLeaders+1:N
   if(i < numLeaders+sizeSwarm1+1)
       color1 = 'b';
       color2 = 'r';
   else
       color1 = [0.7, 0.5, 0.3];
       color2 = 'c';
   end
   g(i) = plot(x(1,i),x(2,i),'o','MarkerSize', newMarkerSize,'LineWidth',5,'Color',color1);
   obstacleDrawRadius(i) = plot(x(1,i),x(2,i),'o','MarkerSize', newObsMarkerSize,'LineWidth',5,'Color',color2);
end

% Plot "wall" boundary
thetaBound = 0:0.1:(2*pi);
boundFact = ((cos(thetaBound).^2)/(1.6^2) + sin(thetaBound).^2);
rThetaBound = (ones(1, length(boundFact))./boundFact).^0.5;
xBound = rThetaBound .* cos(thetaBound);
yBound = rThetaBound .* sin(thetaBound);
plot(xBound, yBound, 'k', 'LineWidth', 3)

%% Loop through movement
for k = 1:max_iter

    x = r.get_poses();                                
    
    dxi=zeros(3,N);                                 
    speedVec = ones(1, N) * speed;
    omegaVec = zeros(1, N);
    %% handles the path following portion for both leaders
     for i= 1:numLeaders
         thetai = x(3, i);
         xd = xOfT(i, k);
         yd = yOfT(i, k);
         %run concensus on current point and point desired
         intDerivative = -[x(1, i) - xd; x(2, i) - yd];
         dxu = si_to_uni_dyn(intDerivative, x(:, i));
         
         wiObs = 0;
         for obsIndex = 1:(obsNumber+avoidWall) % loop through obstacles
            % calculate distance to objects perimeter
            if(avoidWall && obsIndex == obsNumber+1) %add avoidance to the wall
                dVec = distanceToWall(x(1,i), x(2, i));
            else
                dVec = [obs(1, obsIndex) - x(1, i);
                    obs(2, obsIndex) - x(2, i)];
            end
            dInside = 0;
            %distance to center minus radius minus the distance tolerated
            distanceToObs = norm(dVec) - obs(3, obsIndex) - obstacleMinimumDistance;
            if(distanceToObs < 0.0001)
                distanceToObs = 0.0001;
                dHat = dHat;
                dInside = 1;
            end
            dHat = dVec ./ norm(dVec);
            
            NHat = -dHat; % the vector normal to the obstacle is 
            % inverse to the distance vector
            psiObs = atan2(NHat(2), NHat(1));
            if(distanceToObs < obsDetectRange)
                magF = gammaObs/distanceToObs;
                wiObs = wiObs + magF*angle_difference(thetai, psiObs);
            end
         end
         
         set = setBoids1;
         if(i == 1)
             set = setBoids2;
         end
         % handle the potential of detecting other boids
         for nextSwarmIndex = set
            swIndex = nextSwarmIndex;
            % calculate distance to objects perimeter
            dVec = [x(1, swIndex) - x(1, i);
                    x(2, swIndex) - x(2, i)];
            %distance to center minus radius minus the distance tolerated
            distanceToRobot = norm(dVec) - predatorMinumumDistance;
            if(distanceToRobot < 0)
                distanceToRobot = 0.0001;
            end
            dHat = dVec ./ norm(dVec);
            
            NHat = -dHat; % the vector normal to the obstacle is 
            % inverse to the distance vector
            psiObs = atan2(NHat(2), NHat(1));
            if(distanceToRobot < predatorDetectRange)
                magF = gammaPredator/distanceToRobot;
                wiObs = wiObs + magF*angle_difference(thetai, psiObs);
            end
         end
         
         speedVec(i) = saturate(dxu(1), speedMax);
         if(speedVec(i) < 0)
             speedVec(i) = -speedVec(i)*0.75; % do not go backwards if in obstacle
         end
         omegaVec(i) = dxu(2)+saturate(wiObs, wObsMax);    
     end
     
     %% handle movement of swarm 1
     for i= (numLeaders+1):(numLeaders+sizeSwarm1)
         pI = x(1:2, i);
         thetai = x(3, i);
         piDotSeparation = [0; 0];
         piDotCohesion = [0;0];
         wiAlignment = 0;
         for j= 1:(numLeaders+sizeSwarm1)
            if(j == 2) % if the robot is the second leader, skip it
                 continue;
            end
            thetaj = x(3, j);
            % handle minimum separations
            pj = x(1:2, j);
            % calculate weights from i to j
            if(norm(pI - pj) <= separationDist && i ~= j)  
                wij = (norm(pI - pj) - separationDist) / norm(pI - pj)^2;
            else
                wij = 0;
            end
            piDotSeparation(1:2) = piDotSeparation - wij*(pI-pj);
            
            % handle alignment
            wiAlignment = wiAlignment - (thetai - thetaj);
            
            % handle cohesion
            if(norm(pI - pj) < cohesionDist) % if in the neighborhood
                piDotCohesion(1:2) = piDotCohesion - (pI-pj);
            end
         end
         if(abs(piDotSeparation(1)) < 0.0001)
             piDotSeparation(1) = 0.0001;
         end
         if(abs(piDotCohesion(1)) < 0.0001)
             piDotCohesion(1) = 0.0001;
         end
         
         % adjust weights for obstacles in the vicinity
         wiObs = 0;
         for obsIndex = 1:(obsNumber+avoidWall) % loop through obstacles
            % calculate distance to objects perimeter
            if(avoidWall && obsIndex == obsNumber+1) %add avoidance to the wall
                dVec = distanceToWall(x(1,i), x(2, i));
            else
                dVec = [obs(1, obsIndex) - x(1, i);
                    obs(2, obsIndex) - x(2, i)];
            end
            %distance to center minus radius minus the distance tolerated
            distanceToObs = norm(dVec) - obs(3, obsIndex) - obstacleMinimumDistance;
            if(distanceToObs < 0)
                distanceToObs = 0.0001;
            end
            dHat = dVec ./ norm(dVec);
            
            NHat = -dHat; % the vector normal to the obstacle is 
            % inverse to the distance vector
            psiObs = atan2(NHat(2), NHat(1));
            if(distanceToObs < obsDetectRange)
                magF = gammaObs/distanceToObs;
                wiObs = wiObs + magF*angle_difference(thetai, psiObs);
            end
         end
         
         % handle the potential of detecting other boids
         for nextSwarmIndex = setBoids2
            swIndex = nextSwarmIndex;
            % calculate distance to objects perimeter
            dVec = [x(1, swIndex) - x(1, i);
                    x(2, swIndex) - x(2, i)];
            %distance to center minus radius minus the distance tolerated
            distanceToRobot = norm(dVec) - predatorMinumumDistance;
            if(distanceToRobot < 0)
                distanceToRobot = 0.0001;
            end
            dHat = dVec ./ norm(dVec);
            
            NHat = -dHat; % the vector normal to the obstacle is 
            % inverse to the distance vector
            psiObs = atan2(NHat(2), NHat(1));
            if(distanceToRobot < predatorDetectRange)
                magF = gammaPredator/distanceToRobot;
                wiObs = wiObs + magF*angle_difference(thetai, psiObs);
            end
         end
         
         psi1 = atan2(piDotSeparation(2), piDotSeparation(1));
         psi3 = atan2(piDotCohesion(2), piDotCohesion(1));
         wiAlignment = alignmentConst * wiAlignment;
         wiSeparation = separationConst*angle_difference(thetai, psi1);
         wiCohesion = cohesionConst*angle_difference(thetai, psi3);
         wi = wiAlignment + wiSeparation + wiCohesion; 
         wi = saturate(wi, wMax);
         wi = wi+saturate(wiObs, wObsMax);
         omegaVec(i) = wi;
         
         % handle varying speeds
         vl = max(0.0, speedVec(1)); % leader speed
         leaderDVec =  x(1:2, i) - x(1:2, 1);
         angleLeader = atan2(leaderDVec(2), leaderDVec(1));
         leadDir = atan2(x(2, 1), x(1,1));
         unitDir = atan2(x(2, i), x(1, i));
         angleDifference = angle_difference(leadDir, angleLeader);
         dirDifference = angle_difference(unitDir, angleLeader);
         dLeader = norm(leaderDVec); 
         
         % if the robot is facing a different direction than the leader, it
         % should slow down
         dirFactor = (pi-abs(dirDifference))/pi;
         % if the robot is ahead of the leader it should slow down, if not
         % speed up
         if(angleDifference <= pi/2 && angleDifference >= -pi/2)
             % the robot is in front of the leader
            speedIdeal = vl - alphaV*(dLeader-separationDist)^2 * ... 
                ((abs(angleDifference)-pi/2)/(pi/2))^2 * dirFactor;
            if(speedIdeal < 0)
                speedIdeal = 0;
            end
         else
             % behind the leader
            speedIdeal = vl + alphaV*(dLeader-separationDist)^2 * ... 
                ((abs(angleDifference)-pi/2)/(pi/2))^2 * dirFactor;
         end
         speedVec(i) = saturate(speedIdeal, speedMax);
     end
     
     %% handle movement of swarm 2
     for i= (numLeaders+sizeSwarm1+1):N
         pI = x(1:2, i);
         thetai = x(3, i);
         piDotSeparation = [0; 0];
         piDotCohesion = [0;0];
         wiAlignment = 0;
         for jTotal = (numLeaders+sizeSwarm1+1):(N+1)
            j = jTotal;
            if(j == N+1) % make sure to include the second leader
                 j=2;
            end
            thetaj = x(3, j);
            % handle minimum separations
            pj = x(1:2, j);
            % calculate weights from i to j
            if(norm(pI - pj) <= separationDist && i ~= j)  
                wij = (norm(pI - pj) - separationDist) / norm(pI - pj)^2;
            else
                wij = 0;
            end
            piDotSeparation(1:2) = piDotSeparation - wij*(pI-pj);
            
            % handle alignment
            wiAlignment = wiAlignment - (thetai - thetaj);
            
            % handle cohesion
            if(norm(pI - pj) < cohesionDist) % if in the neighborhood
                piDotCohesion(1:2) = piDotCohesion - (pI-pj);
            end
         end
         if(abs(piDotSeparation(1)) < 0.0001)
             piDotSeparation(1) = 0.0001;
         end
         if(abs(piDotCohesion(1)) < 0.0001)
             piDotCohesion(1) = 0.0001;
         end
         
         % adjust weights for obstacles in the vicinity
         wiObs = 0;
         for obsIndex = 1:(obsNumber+avoidWall) % loop through obstacles
            % calculate distance to objects perimeter
            if(avoidWall && obsIndex == obsNumber+1) %add avoidance to the wall
                dVec = distanceToWall(x(1,i), x(2, i));
            else
                dVec = [obs(1, obsIndex) - x(1, i);
                    obs(2, obsIndex) - x(2, i)];
            end
            %distance to center minus radius minus the distance tolerated
            distanceToObs = norm(dVec) - obs(3, obsIndex) - obstacleMinimumDistance;
            if(distanceToObs < 0)
                distanceToObs = 0.0001;
            end
            dHat = dVec ./ norm(dVec);
            
            NHat = -dHat; % the vector normal to the obstacle is 
            % inverse to the distance vector
            psiObs = atan2(NHat(2), NHat(1));
            if(distanceToObs < obsDetectRange)
                magF = gammaObs/distanceToObs;
                wiObs = wiObs + magF*angle_difference(thetai, psiObs);
            end
         end
         
         % handle the potential of detecting other boids
         for nextSwarmIndex = setBoids1
            swIndex = nextSwarmIndex;
            if(swIndex == 2)
                continue; % avoid using the second leader in this
            end
            % calculate distance to objects perimeter
            dVec = [x(1, swIndex) - x(1, i);
                    x(2, swIndex) - x(2, i)];
            %distance to center minus radius minus the distance tolerated
            distanceToRobot = norm(dVec) - predatorMinumumDistance;
            if(distanceToRobot < 0)
                distanceToRobot = 0.0001;
            end
            dHat = dVec ./ norm(dVec);
            
            NHat = -dHat; % the vector normal to the obstacle is 
            % inverse to the distance vector
            psiObs = atan2(NHat(2), NHat(1));
            if(distanceToRobot < predatorDetectRange)
                magF = gammaPredator/distanceToRobot;
                wiObs = wiObs + magF*angle_difference(thetai, psiObs);
            end
         end
         
         psi1 = atan2(piDotSeparation(2), piDotSeparation(1));
         psi3 = atan2(piDotCohesion(2), piDotCohesion(1));
         wiAlignment = alignmentConst * wiAlignment;
         wiSeparation = separationConst*angle_difference(thetai, psi1);
         wiCohesion = cohesionConst*angle_difference(thetai, psi3);
         wi = wiAlignment + wiSeparation + wiCohesion; 
         wi = saturate(wi, wMax);
         wi = wi+saturate(wiObs, wObsMax);
         omegaVec(i) = wi;
         
         % handle varying speeds
         vl = max(0.0, speedVec(2)); % leader speed
         leaderDVec =  x(1:2, i) - x(1:2, 2);
         angleLeader = atan2(leaderDVec(2), leaderDVec(1));
         
         leadDir = atan2(x(2, 2), x(1,2));
         unitDir = atan2(x(2, i), x(1, i));
         angleDifference = angle_difference(leadDir, angleLeader);
         dirDifference = angle_difference(unitDir, angleLeader);
         dLeader = norm(leaderDVec); 
         
         % if the robot is facing a different direction than the leader, it
         % should slow down
         dirFactor = (pi-abs(dirDifference))/pi;
         
         % if the robot is ahead of the leader it should slow down, if not
         % speed up
         if(angleLeader <= pi/2 && angleLeader >= -pi/2)
             % the robot is in front of the leader
            speedIdeal = vl-alphaV*(dLeader-separationDist)^2 * ... 
                ((abs(angleLeader)-pi/2)/(pi/2))^2 * dirFactor;
            if(speedIdeal < 0)
                speedIdeal = 0;
            end
         else
             % behind the leader
            speedIdeal = vl+alphaV*(dLeader-separationDist)^2 * ... 
                ((abs(angleLeader)-pi/2)/(pi/2))^2 * dirFactor;
         end
         speedVec(i) = saturate(speedIdeal, speedMax);
     end
    
     %% total system update
    % redraw circles
    guide(1).XData = xOfT(1,k);
    guide(1).YData = yOfT(1,k);
    guide(2).XData = xOfT(2,k);
    guide(2).YData = yOfT(2,k);
    for i = 1:N
        g(i).XData = x(1,i);
        g(i).YData = x(2,i);
        obstacleDrawRadius(i).XData = x(1,i);
        obstacleDrawRadius(i).YData = x(2,i);
    end

    % every 10 iterations save data
    if(mod(k, 10) == 0)
        for dataIndex = 1:N
            savedRobotData(dataIndex, :, k/10) = x(:, dataIndex);
        end
        savedControllerData(:, :, k/10) = [speedVec; omegaVec]';
    end
    
    dxu = [speedVec; omegaVec];
    dxu = uni_barrier_cert(dxu,x);              
    r.set_velocities(1:N, dxu);                        
    r.step();                                             
    
    if videoFLag && mod(k,10)                               % Record a video frame every 10 iterations
            writeVideo(vid, getframe(gcf)); 
    end
end

if videoFLag 
    close(vid); 
end

save('CompleteTest.mat', 'savedRobotData');
save('CompleteTest.mat', 'savedControllerData');


r.debug();


%% Helper Functions
% This function takes angle2 - angle1 and adjusts for quadrants
% to make sure the resulting angle is always scaled to be in [-pi, pi]
function difference = angle_difference(angle1, angle2)
    difference = 0;
    if(angle1 <= 0 && angle2 <= 0)
        difference = angle2 - angle1;
    elseif(angle1 >= 0 && angle2 >= 0)
        difference = angle2 - angle1;
    elseif(angle2 >= 0 && angle1 <= 0)
        if(angle2 - angle1 <= pi)
            difference = angle2 - angle1;
        else
            difference = (angle2 - angle1) - 2*pi;
        end
    elseif(angle1 >= 0 && angle2 <= 0)
        if(angle2 - angle1 >= -pi)
            difference = angle2 - angle1;
        else
            difference = (angle2 - angle1) + 2*pi;
        end
    end
end

%This function saturates the value of the input to be in the range
% of [-max, max]
function value = saturate(value, max)
    if(value > max)
        value = max;
    elseif(abs(value) > max)
        value = -max;
    end
end

%This function calculates the distance to the ellipse boundary from point (x,y)
% The controls do not work in environments with sharp corners, so the
% system 
function dVec = distanceToWall(x, y)
    angle = atan2(y, x);
    % get distance from wall
    alpha = (cos(angle)^2)/(1.6^2);
    beta = (sin(angle)^2);
    r = (1/(alpha+beta))^0.5;
    xIntersect = r * cos(angle);
    yIntersect = r * sin(angle);
    distance = ((x - xIntersect)^2 + (y-yIntersect)^2)^0.5;
    dVec = [distance * cos(angle); distance * sin(angle)];
end