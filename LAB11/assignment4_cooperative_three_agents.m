function assignment4_cooperative_three_agents
    % CS359: Artificial Intelligence for Robotics - Lab Assignment 11
    % Assignment 4: Cooperative Navigation of Three Agents
    % Real-world application: Cooperative warehouse robots and team navigation
    %
    % Problem Statement: Simulate three agents using an extended APF model that
    % combines goal attraction, inter-agent attraction, and collision-avoidance repulsion.

    global radius;
    clear M; close all;
    maxSimTime = 1000;
    radius = 5;
    d_pref = 22.0;
    d_safe = 12.0;

    figure('Name', 'Assignment 4: Cooperative Navigation of Three Agents', 'NumberTitle', 'off');
    axis([0 100 0 100]);
    axis equal;
    grid on;
    hold on;
    xlabel('Warehouse X (m)');
    ylabel('Warehouse Y (m)');
    title('Assignment 4: Cooperative 3-Agent Navigation (Extended APF)');

    % Initial Positions and Individual Goal Destinations
    starts = [12, 20; 12, 50; 12, 80];
    goals  = [88, 25; 88, 50; 88, 75];

    k_att = 1.2;
    k_col_rep = 180.0;
    k_team_att = 0.08;
    k_team_rep = 0.15;
    v_max = 1.6;
    dt = 0.4;
    goal_thresh = 1.2;

    colors = {'k', 'r', 'g'};
    names = {'A', 'B', 'C'};

    for i = 1:3
        plot(starts(i, 1), starts(i, 2), 'o', 'Color', colors{i}, 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', colors{i});
        plot(goals(i, 1), goals(i, 2), 'p', 'Color', colors{i}, 'MarkerSize', 14, 'LineWidth', 2, 'MarkerFaceColor', colors{i});
        text(starts(i, 1)-4, starts(i, 2)-3, ['Start ' names{i}], 'FontSize', 9, 'FontWeight', 'bold', 'Color', colors{i});
        text(goals(i, 1)-3, goals(i, 2)+3, ['Goal ' names{i}], 'FontSize', 9, 'FontWeight', 'bold', 'Color', colors{i});
    end

    positions = starts;
    trajectories = cell(3, 1);
    for i = 1:3
        trajectories{i} = positions(i, :);
    end

    for t = 1:maxSimTime
        all_finished = 1;
        new_positions = positions;

        for i = 1:3
            distGoal = norm(goals(i, :) - positions(i, :));
            if distGoal <= goal_thresh
                new_positions(i, :) = goals(i, :);
                continue;
            end
            all_finished = 0;

            % 1. Goal attraction force
            if distGoal > 10.0
                F_goal = k_att * 10.0 * ((goals(i, :) - positions(i, :)) / distGoal);
            else
                F_goal = k_att * (goals(i, :) - positions(i, :));
            end

            % 2. Inter-agent interaction forces (collision repulsion + preferred spacing)
            F_inter = [0, 0];
            for j = 1:3
                if i == j, continue; end
                diff_ij = positions(i, :) - positions(j, :);
                d_ij = norm(diff_ij);
                if d_ij < 1e-4
                    diff_ij = [0.1, 0.1];
                    d_ij = norm(diff_ij);
                end
                u_ij = diff_ij / d_ij;

                % Collision avoidance
                clearance = d_ij - 2 * radius;
                if clearance < (d_safe - 2 * radius)
                    eff_clear = max(0.4, clearance);
                    rep_mag = k_col_rep * (1.0/eff_clear - 1.0/(d_safe - 2*radius)) * (1.0 / (eff_clear^2));
                    F_inter = F_inter + rep_mag * u_ij;
                end

                % Coordination / Preferred distance
                if d_ij > d_pref
                    f_coord = k_team_att * (d_ij - d_pref) * (-u_ij);
                else
                    f_coord = k_team_rep * (d_pref - d_ij) * u_ij;
                end
                F_inter = F_inter + f_coord;
            end

            w_coord = min(1.0, distGoal / 15.0);
            F_total = F_goal + w_coord * F_inter;

            f_norm = norm(F_total);
            if f_norm > 1e-6
                speed = min(v_max, f_norm);
                vel = speed * (F_total / f_norm);
            else
                vel = [0, 0];
            end

            new_positions(i, :) = positions(i, :) + vel * dt;
        end

        positions = new_positions;
        for i = 1:3
            trajectories{i} = [trajectories{i}; positions(i, :)];
        end

        % Draw footprints
        handles = zeros(3, 1);
        for i = 1:3
            handles(i) = rectangle('Position', [positions(i, 1)-radius, positions(i, 2)-radius, 2*radius, 2*radius], ...
                                   'Curvature', [1 1], 'FaceColor', colors{i}, 'EdgeColor', 'k');
        end
        drawnow;

        if all_finished == 1
            fprintf('[Assignment 4] All three agents reached their goals at step %d!\n', t);
            break;
        end

        for i = 1:3
            delete(handles(i));
        end
    end

    % Draw path traces
    for i = 1:3
        plot(trajectories{i}(:, 1), trajectories{i}(:, 2), '--', 'Color', colors{i}, 'LineWidth', 2);
    end
    legend('Start A', 'Goal A', '', '', 'Start B', 'Goal B', '', '', 'Start C', 'Goal C', '', '', ...
           'Robot A Final', 'Robot B Final', 'Robot C Final', 'Path A', 'Path B', 'Path C', 'Location', 'southeast');
    hold off;
end

