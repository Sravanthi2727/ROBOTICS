function assignment5_five_agent_chain
    % CS359: Artificial Intelligence for Robotics - Lab Assignment 11
    % Assignment 5: Five-Agent Chain Formation and Following
    % Real-world application: Convoy-based goods transportation
    %
    % Problem Statement: Design an APF-based multi-agent simulation in which
    % five agents form and maintain a leader–follower chain while moving toward
    % a common destination.

    clear M; close all;
    maxSimTime = 1200;
    radius = 3.5;
    d_chain = 12.0;
    d_safe = 8.0;

    figure('Name', 'Assignment 5: Five-Agent Chain Formation & Following', 'NumberTitle', 'off');
    axis([0 100 0 100]);
    axis equal;
    grid on;
    hold on;
    xlabel('Warehouse X (m)');
    ylabel('Warehouse Y (m)');
    title('Assignment 5: Convoy Chain Formation & Following (APF)');

    % Destination
    destination = [88.0, 85.0];

    % Staggered initial staging positions
    starts = [
        28.0, 18.0;  % A1 (Leader)
        20.0, 14.0;  % A2
        14.0, 22.0;  % A3
        12.0, 36.0;  % A4
        16.0, 52.0   % A5
    ];

    k_leader_att = 1.0;
    k_follow_att = 1.2;
    k_follow_rep = 80.0;
    k_avoid = 120.0;
    v_max_leader = 1.4;
    v_max_follower = 1.7;
    dt = 0.4;

    colors = [
        0.12, 0.47, 0.71;  % Blue (Leader)
        1.00, 0.50, 0.05;  % Orange
        0.17, 0.63, 0.17;  % Green
        0.84, 0.15, 0.16;  % Red
        0.58, 0.40, 0.74   % Purple
    ];

    % Draw Destination
    plot(destination(1), destination(2), 'kp', 'MarkerSize', 16, 'LineWidth', 2, 'MarkerFaceColor', 'y');
    text(destination(1)-15, destination(2)+3, 'Destination [88, 85]', 'FontSize', 9, 'FontWeight', 'bold', 'Color', [0.8 0.5 0]);

    % Draw Starts
    for i = 1:5
        plot(starts(i, 1), starts(i, 2), 'o', 'Color', colors(i, :), 'MarkerSize', 8, 'LineWidth', 1.5, 'MarkerFaceColor', colors(i, :));
        text(starts(i, 1)-3, starts(i, 2)-3, ['Start A' num2str(i)], 'FontSize', 8, 'FontWeight', 'bold', 'Color', colors(i, :));
    end

    positions = starts;
    trajectories = cell(5, 1);
    for i = 1:5
        trajectories{i} = positions(i, :);
    end

    for t = 1:maxSimTime
        new_positions = positions;

        % 1. Leader update
        diff_dest = destination - positions(1, :);
        dist_dest = norm(diff_dest);
        if dist_dest > 1.0
            if dist_dest > 12.0
                f_dest = k_leader_att * 12.0 * (diff_dest / dist_dest);
            else
                f_dest = k_leader_att * diff_dest;
            end
            fn_lead = norm(f_dest);
            speed_lead = min(v_max_leader, fn_lead);
            vel_lead = speed_lead * (f_dest / fn_lead);
            new_positions(1, :) = positions(1, :) + vel_lead * dt;
        else
            new_positions(1, :) = destination;
        end

        % 2. Followers update (A2 to A5)
        for k = 2:5
            pos_k = positions(k, :);
            pos_pred = positions(k-1, :);

            % Vector to predecessor
            diff_pred = pos_pred - pos_k;
            dist_pred = norm(diff_pred);
            if dist_pred < 1e-4
                diff_pred = [0.1, 0];
                dist_pred = 0.1;
            end
            u_pred = diff_pred / dist_pred;

            % Harmonic APF around target gap d_chain
            f_chain = k_follow_att * (dist_pred - d_chain) * u_pred;

            % Critical collision buffer
            clearance = dist_pred - 2 * radius;
            if clearance < (d_safe - 2 * radius)
                eff_clear = max(0.3, clearance);
                rep_mag = k_follow_rep * (1.0/eff_clear - 1.0/(d_safe - 2*radius)) * (1.0 / (eff_clear^2));
                f_chain = f_chain - rep_mag * u_pred;
            end

            % Avoid other non-adjacent convoy members
            f_avoid = [0, 0];
            for j = 1:5
                if j == k || j == k - 1, continue; end
                diff_j = pos_k - positions(j, :);
                d_j = norm(diff_j);
                clear_j = d_j - 2 * radius;
                if clear_j < (d_safe - 2 * radius) && d_j > 1e-4
                    eff_c = max(0.3, clear_j);
                    u_j = diff_j / d_j;
                    rep = k_avoid * (1.0/eff_c - 1.0/(d_safe - 2*radius)) * (1.0 / (eff_c^2));
                    f_avoid = f_avoid + rep * u_j;
                end
            end

            f_total = f_chain + f_avoid;
            fn_k = norm(f_total);
            if fn_k > 1e-6
                speed_k = min(v_max_follower, fn_k);
                vel_k = speed_k * (f_total / fn_k);
            else
                vel_k = [0, 0];
            end
            new_positions(k, :) = positions(k, :) + vel_k * dt;
        end

        positions = new_positions;
        for i = 1:5
            trajectories{i} = [trajectories{i}; positions(i, :)];
        end

        % Visualization handles
        handles = zeros(5, 1);
        for i = 1:5
            handles(i) = rectangle('Position', [positions(i, 1)-radius, positions(i, 2)-radius, 2*radius, 2*radius], ...
                                   'Curvature', [1 1], 'FaceColor', colors(i, :), 'EdgeColor', 'k');
        end
        drawnow;

        % Check stop condition
        if norm(destination - positions(1, :)) <= 1.0
            gaps = zeros(4, 1);
            for k = 1:4
                gaps(k) = norm(positions(k, :) - positions(k+1, :));
            end
            if all(abs(gaps - d_chain) < 1.0)
                fprintf('[Assignment 5] Convoy stabilized at step %d!\n', t);
                break;
            end
        end

        for i = 1:5
            delete(handles(i));
        end
    end

    % Path traces
    for i = 1:5
        plot(trajectories{i}(:, 1), trajectories{i}(:, 2), '--', 'Color', colors(i, :), 'LineWidth', 1.8);
    end
    legend('Destination', 'Start A1', 'Start A2', 'Start A3', 'Start A4', 'Start A5', ...
           'Final A1', 'Final A2', 'Final A3', 'Final A4', 'Final A5', 'Location', 'southeast');
    hold off;
end

