% MATLAB script for dynamic task allocation using FCM

% Clear workspace and command window
clear;
clc;

% Number of robots
numRobots = 3;

% Initial number of tasks
numInitialTasks = 10;

% Generate random positions for robots and initial tasks (2D coordinates)
robotPositions = rand(numRobots, 2) * 100;
taskPositions = rand(numInitialTasks, 2) * 100;

% Number of clusters (should be the same as the number of robots)
numClusters = numRobots;

% Perform FCM clustering on the initial task positions
[centers, U] = fcm(taskPositions, numClusters);

% Assign each initial task to the closest robot based on membership values
taskAssignments = zeros(numInitialTasks, 1);

for i = 1:numInitialTasks
    [~, maxIdx] = max(U(:, i));
    taskAssignments(i) = maxIdx;
end

% Plot initial allocation
figure;
subplot(1,2,1);
gscatter(taskPositions(:,1), taskPositions(:,2), taskAssignments, 'rgbmc', 'o');
hold on;
plot(robotPositions(:,1), robotPositions(:,2), 'kx', 'MarkerSize', 10, 'LineWidth', 2);
plot(centers(:,1), centers(:,2), 'ks', 'MarkerSize', 12, 'LineWidth', 2);
title('Initial Task Allocation using FCM');
xlabel('X Position');
ylabel('Y Position');
legend('Cluster 1','Cluster 2','Cluster 3','Cluster 4','Cluster 5','Robots','Cluster Centers');
hold off;

% Introduce new tasks dynamically
numNewTasks = 10;
newTaskPositions = rand(numNewTasks, 2) * 100;
totalTasks = [taskPositions; newTaskPositions];

% Recalculate FCM with the new tasks included
[centers, U] = fcm(totalTasks, numClusters);

% Assign tasks to the closest robot based on updated membership values
totalAssignments = zeros(numInitialTasks + numNewTasks, 1);

for i = 1:numInitialTasks + numNewTasks
    [~, maxIdx] = max(U(:, i));
    totalAssignments(i) = maxIdx;
end

% Plot the dynamic task allocation result
subplot(1,2,2);
gscatter(totalTasks(:,1), totalTasks(:,2), totalAssignments, 'rgbmc', 'o');
hold on;
plot(robotPositions(:,1), robotPositions(:,2), 'kx', 'MarkerSize', 10, 'LineWidth', 2);
plot(centers(:,1), centers(:,2), 'ks', 'MarkerSize', 12, 'LineWidth', 2);
title('Dynamic Task Allocation using FCM');
xlabel('X Position');
ylabel('Y Position');
legend('Cluster 1','Cluster 2','Cluster 3','Cluster 4','Cluster 5','Robots','Cluster Centers');
hold off;
