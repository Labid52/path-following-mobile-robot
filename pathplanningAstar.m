clc;
close all;
clear;
tic;

%% Environment Configuration: Grid and Obstacles
binaryMap = false(20);
binaryMap(5:7, 5:7) = true;
binaryMap(15:17, 15:17) = true;
binaryMap(7:9, 11:13) = true;

%% Start and Goal Definition
startPos = [1, 1];
goalPos = [20, 20];

dx = 1; dy = 1;

% Custom colormap: free, obstacle, explored, current, start, goal, path
cmap = [1 1 1; ...
        0 0 0; ...
        1 0 0; ...
        0 0 1; ...
        0 1 0; ...
        1 1 0; ...
        0.5 0.5 0.5];
colormap(cmap);

drawMapLive = true;

%% Grid Initialization
[rows, cols] = size(binaryMap);
searchGrid = zeros(rows, cols);
searchGrid(~binaryMap) = 1; % Free cells
searchGrid(binaryMap) = 2;  % Obstacles

startIdx = sub2ind(size(searchGrid), startPos(1), startPos(2));
goalIdx = sub2ind(size(searchGrid), goalPos(1), goalPos(2));
searchGrid(startIdx) = 5;
searchGrid(goalIdx) = 6;

parentNode = zeros(rows, cols);
[X, Y] = meshgrid(1:cols, 1:rows);

%% A* Initialization
xInit = startPos(1); yInit = startPos(2);
xGoal = goalPos(1); yGoal = goalPos(2);

gCost = Inf(size(searchGrid));
hCost = Inf(size(searchGrid));
fCost = Inf(size(searchGrid));

cumCost = 0;
activeIdx = startIdx;
fValMin = Inf;

trajectory = [startPos];
xCurr = xInit; yCurr = yInit;

%% A* Planning Loop
while xCurr ~= xGoal || yCurr ~= yGoal
    searchGrid(startIdx) = 5;
    searchGrid(goalIdx) = 6;

    if drawMapLive
        image(1.5, 1.5, searchGrid);
        grid on; axis image;
        drawnow;
    end

    for xi = xCurr - dx : dx : xCurr + dx
        if xi >= 1 && xi <= rows
            for yi = yCurr - dy : dy : yCurr + dy
                if yi >= 1 && yi <= cols && ...
                   fCost(xi, yi) == Inf && binaryMap(xi, yi) == 0

                    gCost(xi, yi) = sqrt((xi - xGoal)^2 + (yi - yGoal)^2);
                    hCost(xi, yi) = sqrt((xi - xInit)^2 + (yi - yInit)^2);
                    fCost(xi, yi) = gCost(xi, yi) + hCost(xi, yi);

                    cumCost = cumCost + gCost(xi, yi);
                    searchGrid(xi, yi) = 4;
                    parentNode(xi, yi) = activeIdx;

                    if gCost(xi, yi) < fValMin
                        fValMin = fCost(xi, yi);
                        xNext = xi;
                        yNext = yi;
                    end
                end
            end
        end
    end

    xCurr = xNext; yCurr = yNext; fValMin = Inf;
    activeIdx = sub2ind(size(searchGrid), xCurr, yCurr);
    searchGrid(xCurr, yCurr) = 3;
    trajectory = [trajectory; xCurr yCurr];
end

%% Path Extraction from Parent Matrix
for k = 1:size(trajectory, 1)
    pathIdx(k) = sub2ind(size(searchGrid), trajectory(k, 1), trajectory(k, 2));
end

% Visual Playback
for k = 1:length(pathIdx)
    searchGrid(pathIdx(k)) = 7;
    pause(0.1);
    image(1.5, 1.5, searchGrid);
    grid on; axis image;
end

%% Path Visualization
title('A* Algorithm - Grid View');
figure;
plot(trajectory(:, 1), trajectory(:, 2), 'k', 'LineWidth', 1.2);
grid on; title('Trajectory Obtained by A*');
axis square;

toc;
