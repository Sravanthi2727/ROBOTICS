function assignment1_delivery_station
    % CS359: Artificial Intelligence for Robotics - Lab Assignment 11
    % Assignment 1: Autonomous Robot Navigation to a Delivery Station
    % Real-world application: Warehouse goods delivery
    %
    % Problem Statement: Develop an APF-based simulation in which an autonomous
    % robot moves from its starting position to a specified goal position
    % using an attractive potential field.

    global radius;
    clear M; close all;
    maxSimTime = 1000;
    radius = 5;
    figure('Name', 'Assignment 1: Navigation to Delivery Station (APF)', 'NumberTitle', 'off');
    axis([0 100 0 100]);
    axis equal;
    grid on;
    hold on;
    xlabel('Warehouse X (m)');
    ylabel('Warehouse Y (m)');
    title('Assignment 1: Autonomous Robot Navigation to Delivery Station');

    % Define Start (Storage Location) and Goal (Delivery Station)
    Start = [10, 10];
    Goal = [90, 90];
    k_att = 1.0;
    v_max = 1.5;
    dt = 0.5;
    goal_thresh = 1.0;

    % Draw Start and Goal Markers
    plot(Start(1), Start(2), 'go', 'MarkerSize', 12, 'LineWidth', 2, 'MarkerFaceColor', 'g');
    text(Start(1)+2, Start(2)-2, 'Start (Storage)', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0 0.5 0]);
    plot(Goal(1), Goal(2), 'kp', 'MarkerSize', 16, 'LineWidth', 2, 'MarkerFaceColor', 'y');
    text(Goal(1)-15, Goal(2)+3, 'Goal (Delivery Station)', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.8 0.5 0]);

    positionA = Start;
    trajectoryA = positionA;
    notMovingA = 0;

    for t = 1:maxSimTime
        % Calculate attractive force and next position
        [positionNewA, finishA] = getPositionA_APF(positionA, Goal, k_att, v_max, dt, goal_thresh);

        if norm(positionNewA - positionA) > 1e-4
            positionA = positionNewA;
            notMovingA = 0;
        else
            notMovingA = notMovingA + 1;
        end
        trajectoryA = [trajectoryA; positionA];

        % Graphics and visualization
        handleA = rectangle('Position', [positionA(1)-radius, positionA(2)-radius, 2*radius, 2*radius], ...
                            'Curvature', [1 1], 'FaceColor', 'b', 'EdgeColor', 'k');
        drawnow;

        if finishA == 1
            fprintf('[Assignment 1] Destination reached at step %d!\n', t);
            break;
        end
        if notMovingA > 15
            fprintf('[Assignment 1] Robot reached steady state / stopped.\n');
            break;
        end

        delete(handleA);
    end

    % Plot full path trace
    plot(trajectoryA(:, 1), trajectoryA(:, 2), 'b--', 'LineWidth', 2);
    legend('Start', '', 'Goal', '', 'Robot Final', 'Path Trajectory', 'Location', 'southeast');
    hold off;
end

function [posNew, finish] = getPositionA_APF(pos, Goal, k_att, v_max, dt, thresh)
    dist = norm(Goal - pos);
    if dist <= thresh
        posNew = Goal;
        finish = 1;
        return;
    end

    % Attractive force computation
    F_att = k_att * (Goal - pos);
    f_norm = norm(F_att);
    if f_norm > 1e-6
        speed = min(v_max, f_norm);
        velocity = speed * (F_att / f_norm);
    else
        velocity = [0, 0];
    end

    posNew = pos + velocity * dt;
    if norm(Goal - posNew) <= thresh
        finish = 1;
    else
        finish = 0;
    end
end

