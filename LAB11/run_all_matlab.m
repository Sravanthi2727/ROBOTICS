function run_all_matlab
    % CS359: Artificial Intelligence for Robotics - Lab Assignment 11
    % Master Runner for MATLAB Simulations
    
    fprintf('=================================================================\n');
    fprintf(' CS359: Artificial Intelligence for Robotics - Lab Assignment 11\n');
    fprintf(' Artificial Potential Field (APF) Approach\n');
    fprintf('=================================================================\n\n');
    
    fprintf('Running Assignment 1: Autonomous Robot Navigation to Delivery Station...\n');
    assignment1_delivery_station;
    pause(1);
    
    fprintf('\nRunning Assignment 2: Robot Navigation with Static Storage Racks...\n');
    assignment2_static_obstacles;
    pause(1);
    
    fprintf('\nRunning Assignment 3: Robot Navigation with Moving Obstacle...\n');
    assignment3_moving_obstacle;
    pause(1);
    
    fprintf('\nRunning Assignment 4: Cooperative Navigation of Three Agents...\n');
    assignment4_cooperative_three_agents;
    pause(1);
    
    fprintf('\nRunning Assignment 5: Five-Agent Chain Formation & Following...\n');
    assignment5_five_agent_chain;
    
    fprintf('\nAll MATLAB simulations completed successfully!\n');
end

