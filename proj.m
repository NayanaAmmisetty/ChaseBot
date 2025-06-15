% === ChaseBot Enhanced Simulation ===

% Step 1: Initialize webcam and screen
cam = webcam;
frameWidth = 640;
frameHeight = 480;

robotPos = [320, 240; 100, 240; 500, 240];
robotRadius = 10;
robotSpeed = 5;

% INSERT HERE:
numObstacles = 8;
obstacleRadius = 15;

% Obstacles
% Initialize obstacle positions within frame boundaries
obstacleCenters = [randi([obstacleRadius, frameWidth - obstacleRadius], numObstacles, 1), ...
                   randi([obstacleRadius, frameHeight - obstacleRadius], numObstacles, 1)];

% Random direction and speed (magnitude between 2 and 4 pixels/frame)
angles = rand(numObstacles, 1) * 2 * pi;
speeds = 2 + rand(numObstacles, 1) * 2; % Speeds between 2 and 4
obstacleVel = [cos(angles), sin(angles)] .* speeds;


% Paths
pathGreen = []; pathPink = []; pathBlue = [];
targetPathGreen = []; targetPathPink = []; targetPathBlue = [];

% Sound effects
chime = audioread('chime.wav'); beep = audioread('beep.wav');
chimePlayer = audioplayer(chime, 44100);
beepPlayer = audioplayer(beep, 44100);


frameCount = 0;
close all;
figure;

while true
    frameCount = frameCount + 1;
    frame = snapshot(cam);
    frame = fliplr(frame);
    hsvFrame = rgb2hsv(frame);

    % === Color Detection ===
    greenMask = (hsvFrame(:,:,1) > 0.25 & hsvFrame(:,:,1) < 0.45) & ...
                (hsvFrame(:,:,2) > 0.4 & hsvFrame(:,:,3) > 0.4);
    pinkMask = (hsvFrame(:,:,1) > 0.85 | hsvFrame(:,:,1) < 0.05) & ...
               (hsvFrame(:,:,2) > 0.3 & hsvFrame(:,:,2) < 0.7) & ...
               (hsvFrame(:,:,3) > 0.5);
    blueMask = (hsvFrame(:,:,1) > 0.55 & hsvFrame(:,:,1) < 0.65) & ...
               (hsvFrame(:,:,2) > 0.4 & hsvFrame(:,:,3) > 0.4);

    greenPos = getLargestRegion(greenMask);
    pinkPos = getLargestRegion(pinkMask);
    bluePos = getLargestRegion(blueMask);

    if ~isempty(greenPos), targetPathGreen = [targetPathGreen; greenPos]; end
    if ~isempty(pinkPos),  targetPathPink = [targetPathPink; pinkPos]; end
    if ~isempty(bluePos),  targetPathBlue = [targetPathBlue; bluePos]; end

    % Update circular obstacle positions
    for i = 1:numObstacles
        obstacleCenters(i,:) = obstacleCenters(i,:) + obstacleVel(i,:);
        if obstacleCenters(i,1) <= obstacleRadius || obstacleCenters(i,1) >= frameWidth - obstacleRadius
            obstacleVel(i,1) = -obstacleVel(i,1);
        end
        if obstacleCenters(i,2) <= obstacleRadius || obstacleCenters(i,2) >= frameHeight - obstacleRadius
            obstacleVel(i,2) = -obstacleVel(i,2);
        end
    end

    % === Update Bot Positions with Improved Obstacle Avoidance ===
    robotPos(1,:) = improvedAvoidance(robotPos(1,:), greenPos, robotSpeed, obstacleCenters, obstacleRadius);
    robotPos(2,:) = improvedAvoidance(robotPos(2,:), pinkPos, robotSpeed, obstacleCenters, obstacleRadius);
    robotPos(3,:) = improvedAvoidance(robotPos(3,:), bluePos, robotSpeed, obstacleCenters, obstacleRadius);

    pathGreen = [pathGreen; robotPos(1,:)];
    pathPink  = [pathPink;  robotPos(2,:)];
    pathBlue  = [pathBlue;  robotPos(3,:)];

    % === Visualization ===
    imshow(frame); hold on;
     % === Display Simulation Time ===
    simTime = frameCount * 0.03; % assuming 0.03s per frame
    rectangle('Position', [5 5 250 95], 'FaceColor', [0 0 0 0.5], 'EdgeColor', 'none');
    text(10, 20, sprintf('Simulation Time: %.2f s', simTime), ...
         'Color', 'w', 'FontSize', 12, 'FontWeight', 'bold');

    % === Display Detection Status for each color ===
    greenStatus = 'Not Detected'; pinkStatus = 'Not Detected'; blueStatus = 'Not Detected';
    if ~isempty(greenPos), greenStatus = 'Detected'; end
    if ~isempty(pinkPos),  pinkStatus  = 'Detected'; end
    if ~isempty(bluePos),  blueStatus  = 'Detected'; end

    text(10, 50, sprintf('Green Object: %s', greenStatus), 'Color', 'g', 'FontSize', 11, 'FontWeight', 'bold');
    text(10, 70, sprintf('Pink Object : %s', pinkStatus),  'Color', 'm', 'FontSize', 11, 'FontWeight', 'bold');
    text(10, 90, sprintf('Blue Object : %s', blueStatus),  'Color', 'b', 'FontSize', 11, 'FontWeight', 'bold');


    % Draw detected positions
    if ~isempty(greenPos), plot(greenPos(1), greenPos(2), 'gx', 'MarkerSize', 10, 'LineWidth', 2); end
    if ~isempty(pinkPos),  plot(pinkPos(1),  pinkPos(2),  'mx', 'MarkerSize', 10, 'LineWidth', 2); end
    if ~isempty(bluePos),  plot(bluePos(1),  bluePos(2),  'bx', 'MarkerSize', 10, 'LineWidth', 2); end

    % Draw round bots
    drawBot(robotPos(1,:), 'g');
    drawBot(robotPos(2,:), 'm');
    drawBot(robotPos(3,:), 'b');

    % Trigger circle when near
    if ~isempty(greenPos) && norm(robotPos(1,:) - greenPos) < 30
        viscircles(robotPos(1,:), 25, 'Color', 'g', 'LineStyle', '--');
    end
    if ~isempty(pinkPos) && norm(robotPos(2,:) - pinkPos) < 30
        viscircles(robotPos(2,:), 25, 'Color', 'm', 'LineStyle', '--');
    end
    if ~isempty(bluePos) && norm(robotPos(3,:) - bluePos) < 30
        viscircles(robotPos(3,:), 25, 'Color', 'b', 'LineStyle', '--');
    end
    % === Check if any bot reached the target ===
    reached1 = ~isempty(greenPos) && norm(robotPos(1,:) - greenPos) < 5;
    reached2 = ~isempty(pinkPos)  && norm(robotPos(2,:) - pinkPos) < 5;
    reached3 = ~isempty(bluePos)  && norm(robotPos(3,:) - bluePos) < 5;

    % === Check if any bot is near any obstacle ===
    nearObs1 = any(vecnorm(obstacleCenters - robotPos(1,:), 2, 2) < (obstacleRadius + 8));
    nearObs2 = any(vecnorm(obstacleCenters - robotPos(2,:), 2, 2) < (obstacleRadius + 8));
    nearObs3 = any(vecnorm(obstacleCenters - robotPos(3,:), 2, 2) < (obstacleRadius + 8));

    % === Sound effects ===
    if (reached1 || reached2 || reached3)
        if ~isplaying(chimePlayer)
        play(chimePlayer);
        end
    elseif (nearObs1 || nearObs2 || nearObs3)
        if ~isplaying(beepPlayer)
        play(beepPlayer);
        end
    end


    % Draw circular obstacles
    for i = 1:numObstacles
        viscircles(obstacleCenters(i,:), obstacleRadius, 'Color', 'k');
    end

    % Plot dotted path
    plot(pathGreen(:,1), pathGreen(:,2), 'g:', 'LineWidth', 1.5);
    plot(pathPink(:,1),  pathPink(:,2),  'm:', 'LineWidth', 1.5);
    plot(pathBlue(:,1),  pathBlue(:,2),  'b:', 'LineWidth', 1.5);

    % Big dot if bot reached target
    if ~isempty(greenPos) && norm(robotPos(1,:) - greenPos) < 5
        plot(robotPos(1,1), robotPos(1,2), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
    end
    if ~isempty(pinkPos) && norm(robotPos(2,:) - pinkPos) < 5
        plot(robotPos(2,1), robotPos(2,2), 'mo', 'MarkerSize', 12, 'MarkerFaceColor', 'm');
    end
    if ~isempty(bluePos) && norm(robotPos(3,:) - bluePos) < 5
        plot(robotPos(3,1), robotPos(3,2), 'bo', 'MarkerSize', 12, 'MarkerFaceColor', 'b');
    end

    hold off;

    % Exit if all bots reached
    if ~isempty(greenPos) && ~isempty(pinkPos) && ~isempty(bluePos)
        if all(vecnorm(robotPos - [greenPos; pinkPos; bluePos], 2, 2) < 5)
            break;
        end
    end
    pause(0.03);
end

% Cleanup
clear cam;

% === Graph 1: 3D Motion Paths ===
timeSteps = 1:size(pathGreen,1);
figure;
hold on;
title('Enhanced 3D Motion Paths of ChaseBots and Targets');
xlabel('X Position'); ylabel('Y Position'); zlabel('Time');
xlim([0, frameWidth]); ylim([0, frameHeight]); zlim([0, max(timeSteps)]);

plot3(pathGreen(:,1), pathGreen(:,2), timeSteps', 'g-', 'LineWidth', 2);
plot3(pathPink(:,1),  pathPink(:,2),  timeSteps', 'm-', 'LineWidth', 2);
plot3(pathBlue(:,1),  pathBlue(:,2),  timeSteps', 'b-', 'LineWidth', 2);

plot3(targetPathGreen(:,1), targetPathGreen(:,2), 1:size(targetPathGreen,1), 'g--', 'LineWidth', 1.5);
plot3(targetPathPink(:,1),  targetPathPink(:,2),  1:size(targetPathPink,1),  'm--', 'LineWidth', 1.5);
plot3(targetPathBlue(:,1),  targetPathBlue(:,2),  1:size(targetPathBlue,1),  'b--', 'LineWidth', 1.5);

plot3(pathGreen(end,1), pathGreen(end,2), timeSteps(end), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot3(pathPink(end,1), pathPink(end,2), timeSteps(end), 'mo', 'MarkerSize', 10, 'MarkerFaceColor', 'm');
plot3(pathBlue(end,1), pathBlue(end,2), timeSteps(end), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');

legend({'Green Bot Path', 'Pink Bot Path', 'Blue Bot Path', ...
        'Green Target Path', 'Pink Target Path', 'Blue Target Path', ...
        'Green Contact Point', 'Pink Contact Point', 'Blue Contact Point'}, ...
        'Location', 'northeastoutside');
grid on;
view(3);
hold off;

% === Graph 2: 3D Position vs Time ===
figure;
tt = 1:5:frameCount;
subplot(3,1,1);
plot3(tt, pathGreen(1:5:end,1), pathGreen(1:5:end,2), 'g');
title('Green Bot - 3D Position vs Time'); xlabel('Time'); ylabel('X'); zlabel('Y'); grid on; view(3);

subplot(3,1,2);
plot3(tt, pathPink(1:5:end,1), pathPink(1:5:end,2), 'm');
title('Pink Bot - 3D Position vs Time'); xlabel('Time'); ylabel('X'); zlabel('Y'); grid on; view(3);

subplot(3,1,3);
plot3(tt, pathBlue(1:5:end,1), pathBlue(1:5:end,2), 'b');
title('Blue Bot - 3D Position vs Time'); xlabel('Time'); ylabel('X'); zlabel('Y'); grid on; view(3);

% === Helper: Get Region ===
function pos = getLargestRegion(mask)
    stats = regionprops(mask, 'Centroid', 'Area');
    if isempty(stats)
        pos = [];
    else
        [~, idx] = max([stats.Area]);
        pos = stats(idx).Centroid;
    end
end

% === Helper: Draw Round Bot ===
function drawBot(pos, color)
    rectangle('Position', [pos - 10, 20, 20], 'Curvature', [1,1], 'FaceColor', color);
end

% === Improved Obstacle Avoidance ===
function newPos = improvedAvoidance(bot, target, speed, centers, rad)
    if isempty(target)
        newPos = bot;
        return;
    end
    dir = target - bot;
    if norm(dir) == 0
        newPos = bot;
        return;
    end
    dir = dir / norm(dir) * speed;
    next = bot + dir;

    for i = 1:size(centers,1)
        dist = norm(next - centers(i,:));
        if dist < rad + 8
            perp = [-dir(2), dir(1)];
            next = bot + perp * 0.7 * speed;
            break;
        end
    end
    newPos = next;
end