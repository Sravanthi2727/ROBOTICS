function assignment2_static_obstacles
    % CS359: Artificial Intelligence for Robotics - Lab Assignment 11
    % Assignment 2: Robot Navigation in a Warehouse with Static Obstacles
    % Real-world application: Collision-free navigation between storage racks
    %
    % Problem Statement: Simulate Robot A using APF in an environment containing
    % two static rectangular obstacles.

    global radius;
    clear M; close all;
    maxSimTime = 1000;
    radius = 5;
    figure('Name', 'Assignment 2: Navigation with Static Storage Racks (APF)', 'NumberTitle', 'off');
    axis([0 100 0 100]);
    axis equal;
    grid on;
    hold on;
    xlabel('Warehouse X (m)');
    ylabel('Warehouse Y (m)');
    title('Assignment 2: Collision-Free Navigation between 2 Static Storage Racks');

    % Define Start and Goal
    Start = [10, 10];
    Goal = [90, 90];
    k_att = 1.0;
    k_rep = 250.0;
    rho_0 = 16.0;
    v_max = 1.8;
    dt = 0.4;
    goal_thresh = 1.5;

    % Two static rectangular storage racks: [xmin, xmax, ymin, ymax]
    rack1 = [24, 44, 26, 40];
    rack2 = [56, 76, 58, 72];
    obstacles = [rack1; rack2];

    % Draw Obstacles (Rectangles)
    rectangle('Position', [rack1(1), rack1(3), rack1(2)-rack1(1), rack1(4)-rack1(3)], ...
              'FaceColor', [0.85 0.35 0.35], 'EdgeColor', 'k', 'LineWidth', 1.5);
    text((rack1(1)+rack1(2))/2, (rack1(3)+rack1(4))/2, 'Storage Rack 1', ...
         'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'Color', 'w');

    rectangle('Position', [rack2(1), rack2(3), rack2(2)-rack2(1), rack2(4)-rack2(3)], ...
              'FaceColor', [0.9 0.6 0.2], 'EdgeColor', 'k', 'LineWidth', 1.5);
    text((rack2(1)+rack2(2))/2, (rack2(3)+rack2(4))/2, 'Storage Rack 2', ...
         'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'Color', 'w');

    % Draw Start and Goal
    plot(Start(1), Start(2), 'go', 'MarkerSize', 12, 'LineWidth', 2, 'MarkerFaceColor', 'g');
    text(Start(1)+2, Start(2)-2, 'Start [10, 10]', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0 0.5 0]);
    plot(Goal(1), Goal(2), 'kp', 'MarkerSize', 16, 'LineWidth', 2, 'MarkerFaceColor', 'y');
    text(Goal(1)-15, Goal(2)+3, 'Goal [90, 90]', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.8 0.5 0]);

    positionA = Start;
    trajectoryA = positionA;
    notMovingA = 0;

    for t = 1:maxSimTime
        % Calculate attractive and repulsive potential forces
        [positionNewA, finishA] = getPositionA_StaticAPF(positionA, Goal, obstacles, radius, k_att, k_rep, rho_0, v_max, dt, goal_thresh);

        if norm(positionNewA - positionA) > 1e-4
            positionA = positionNewA;
            notMovingA = 0;
        else
            notMovingA = notMovingA + 1;
        end
        trajectoryA = [trajectoryA; positionA];

        % Dynamic footprint
        handleA = rectangle('Position', [positionA(1)-radius, positionA(2)-radius, 2*radius, 2*radius], ...
                            'Curvature', [1 1], 'FaceColor', 'b', 'EdgeColor', 'k');
        drawnow;

        if finishA == 1
            fprintf('[Assignment 2] Goal reached successfully at step %d!\n', t);
            break;
        end
        if notMovingA > 20
            fprintf('[Assignment 2] Robot stopped / reached equilibrium.\n');
            break;
        end

        delete(handleA);
    end

    % Plot full path
    plot(trajectoryA(:, 1), trajectoryA(:, 2), 'b--', 'LineWidth', 2.5);
    legend('Rack 1', 'Rack 2', 'Start', 'Goal', 'Robot Final', 'APF Trajectory', 'Location', 'southeast');
    hold off;
end

function [posNew, finish] = getPositionA_StaticAPF(pos, Goal, obstacles, radius, k_att, k_rep, rho_0, v_max, dt, thresh)
    distToGoal = norm(Goal - pos);
    if distToGoal <= thresh
        posNew = Goal;
        finish = 1;
        return;
    end

    % Attractive Force
    if distToGoal > 12.0
        F_att = k_att * 12.0 * ((Goal - pos) / distToGoal);
    else
        F_att = k_att * (Goal - pos);
    end

    % Repulsive Force from Rectangular Obstacles
    F_rep = [0, 0];
    for i = 1:size(obstacles, 1)
        rack = obstacles(i, :);
        % Closest point on rectangle
        cx = min(max(pos(1), rack(1)), rack(2));
        cy = min(max(pos(2), rack(3)), rack(4));
        cp = [cx, cy];

        diff = pos - cp;
        dist = norm(diff);
        if dist < 1e-6
            dist = 0.1;
            diff = [0.1, 0.1];
        end
        u_vec = diff / norm(diff);
        eff_dist = max(0.5, dist - radius);

        if eff_dist < rho_0
            rep_mag = k_rep * (1.0/eff_dist - 1.0/rho_0) * (1.0 / (eff_dist^2));
            f_rep_rack = rep_mag * u_vec;
            % Tangential steering force to clear edges smoothly
            tang_vec = [-u_vec(2), u_vec(1)];
            f_rep_rack = f_rep_rack + 0.25 * rep_mag * tang_vec;
            F_rep = F_rep + f_rep_rack;
        end
    end

    F_total = F_att + F_rep;
    f_norm = norm(F_total);
    if f_norm > 1e-6
        speed = min(v_max, f_norm);
        vel = speed * (F_total / f_norm);
    else
        vel = [0, 0];
    end

    posNew = pos + vel * dt;
    if norm(Goal - posNew) <= thresh
        finish = 1;
    else
        finish = 0;
    end
end

