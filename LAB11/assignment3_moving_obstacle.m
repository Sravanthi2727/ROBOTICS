function assignment3_moving_obstacle
    % CS359: Artificial Intelligence for Robotics - Lab Assignment 11
    % Assignment 3: Robot Navigation with a Moving Obstacle
    % Real-world application: Collision avoidance in shared robot workspaces
    %
    % Problem Statement: Develop a dynamic APF simulation in which an autonomous
    % robot avoids a moving obstacle while continuing toward its goal.

    global radius;
    clear M; close all;
    maxSimTime = 1000;
    radius = 5;
    radius_obs = 4;
    figure('Name', 'Assignment 3: Dynamic APF with Moving Obstacle', 'NumberTitle', 'off');
    axis([0 100 0 100]);
    axis equal;
    grid on;
    hold on;
    xlabel('Warehouse X (m)');
    ylabel('Warehouse Y (m)');
    title('Assignment 3: Dynamic APF Collision Avoidance with Moving Obstacle');

    % Positions
    startA = [10, 10];
    goalA = [90, 90];
    startObs = [20, 85];
    goalObs = [85, 20];

    k_att = 1.0;
    k_rep_dyn = 300.0;
    rho_0 = 24.0;
    v_max = 1.8;
    v_obs = 1.1;
    dt = 0.5;
    goal_thresh = 1.2;

    % Static markers
    plot(startA(1), startA(2), 'go', 'MarkerSize', 12, 'LineWidth', 2, 'MarkerFaceColor', 'g');
    text(startA(1)+2, startA(2)-2, 'Robot Start [10, 10]', 'FontSize', 9, 'FontWeight', 'bold', 'Color', [0 0.5 0]);
    plot(goalA(1), goalA(2), 'kp', 'MarkerSize', 16, 'LineWidth', 2, 'MarkerFaceColor', 'y');
    text(goalA(1)-15, goalA(2)+3, 'Robot Goal [90, 90]', 'FontSize', 9, 'FontWeight', 'bold', 'Color', [0.8 0.5 0]);

    plot(startObs(1), startObs(2), 'ms', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'm');
    plot(goalObs(1), goalObs(2), 'rx', 'MarkerSize', 12, 'LineWidth', 2.5);

    posA = startA;
    posObs = startObs;
    obs_dir = (goalObs - startObs) / norm(goalObs - startObs);

    trajA = posA;
    trajObs = posObs;
    notMovingA = 0;

    for t = 1:maxSimTime
        % Update moving obstacle
        if norm(goalObs - posObs) > 0.5
            posObs = posObs + obs_dir * v_obs * dt;
        end
        trajObs = [trajObs; posObs];

        % Compute dynamic APF
        [posNewA, finishA] = getPositionA_DynamicAPF(posA, goalA, posObs, obs_dir, radius, radius_obs, k_att, k_rep_dyn, rho_0, v_max, dt, goal_thresh);

        if norm(posNewA - posA) > 1e-4
            posA = posNewA;
            notMovingA = 0;
        else
            notMovingA = notMovingA + 1;
        end
        trajA = [trajA; posA];

        % Draw footprints
        handleA = rectangle('Position', [posA(1)-radius, posA(2)-radius, 2*radius, 2*radius], ...
                            'Curvature', [1 1], 'FaceColor', 'b', 'EdgeColor', 'k');
        handleObs = rectangle('Position', [posObs(1)-radius_obs, posObs(2)-radius_obs, 2*radius_obs, 2*radius_obs], ...
                              'Curvature', [1 1], 'FaceColor', 'r', 'EdgeColor', 'k');
        drawnow;

        if finishA == 1
            fprintf('[Assignment 3] Robot reached destination safely at step %d!\n', t);
            break;
        end
        if notMovingA > 20
            fprintf('[Assignment 3] Robot reached stop condition.\n');
            break;
        end

        delete(handleA);
        delete(handleObs);
    end

    plot(trajA(:, 1), trajA(:, 2), 'b-', 'LineWidth', 2.5);
    plot(trajObs(:, 1), trajObs(:, 2), 'r--', 'LineWidth', 2);
    legend('Start A', '', 'Goal A', '', 'Start Obs', 'Goal Obs', 'Robot A Final', 'Obstacle Final', 'Robot A Path', 'Obstacle Path', 'Location', 'southwest');
    hold off;
end

function [posNew, finish] = getPositionA_DynamicAPF(posA, goalA, posObs, obsDir, rA, rObs, k_att, k_rep, rho_0, v_max, dt, thresh)
    distGoal = norm(goalA - posA);
    if distGoal <= thresh
        posNew = goalA;
        finish = 1;
        return;
    end

    % Goal attraction
    if distGoal > 12.0
        F_att = k_att * 12.0 * ((goalA - posA) / distGoal);
    else
        F_att = k_att * (goalA - posA);
    end

    % Dynamic Obstacle Repulsion
    diffObs = posA - posObs;
    distObs = norm(diffObs);
    clearance = max(0.4, distObs - (rA + rObs));

    F_rep = [0, 0];
    if clearance < rho_0
        u_rep = diffObs / distObs;
        rep_mag = k_rep * (1.0/clearance - 1.0/rho_0) * (1.0 / (clearance^2));
        F_rep = rep_mag * u_rep;

        % Lateral evasion to smoothly deflect around moving obstacle
        evasion_dir = [-obsDir(2), obsDir(1)];
        if dot(evasion_dir, diffObs) < 0
            evasion_dir = -evasion_dir;
        end
        F_rep = F_rep + 0.4 * rep_mag * evasion_dir;
    end

    F_total = F_att + F_rep;
    f_norm = norm(F_total);
    if f_norm > 1e-6
        speed = min(v_max, f_norm);
        vel = speed * (F_total / f_norm);
    else
        vel = [0, 0];
    end

    posNew = posA + vel * dt;
    if norm(goalA - posNew) <= thresh
        finish = 1;
    else
        finish = 0;
    end
end

