function sample_code_from_sheet
    % CS359: Artificial Intelligence for Robotics - Lab Assignment 11
    % Baseline sample code provided in the assignment handout sheet
    
    global radius
    clear M; close all;
    maxSimTime = 1000;
    radius = 5;
    figure('Name', 'Sample Code from Handout', 'NumberTitle', 'off');
    axis([0 100 0 100]);
    axis equal;
    hold on;
    grid on;

    % initialize agents here
    positionB = [10 90];
    positionC = [90 10];
    notMovingA = 0; notMovingB = 0; notMovingC = 0;

    for t = 1:maxSimTime
        % Agent Function Call
        [positionNewA, finishA] = getPositionA(t, positionB, positionC);
        % notMovingA is the time for which A is stationary
        if (~exist('positionA', 'var') || positionNewA(1) ~= positionA(1) || positionNewA(2) ~= positionA(2))
            positionA = positionNewA; notMovingA = 0;
        else
            notMovingA = notMovingA + 1;
        end

        [positionNewB, finishB] = getPositionB(positionB, positionA, positionC);
        if (positionNewB(1) ~= positionB(1) || positionNewB(2) ~= positionB(2))
            positionB = positionNewB; notMovingB = 0;
        else
            notMovingB = notMovingB + 1;
        end

        FinishC = [10 90];
        [positionNewC, finishC] = getPositionC(positionC, FinishC, positionA, positionB);
        if (positionNewC(1) ~= positionC(1) || positionNewC(2) ~= positionC(2))
            positionC = positionNewC; notMovingC = 0;
        else
            notMovingC = notMovingC + 1;
        end

        % Graphics and animation
        handleA = rectangle('Position', [positionA(1)-radius, positionA(2)-radius, 2*radius, 2*radius], 'curvature', [1 1], 'FaceColor', 'black');
        handleB = rectangle('Position', [positionB(1)-radius, positionB(2)-radius, 2*radius, 2*radius], 'curvature', [1 1], 'FaceColor', 'red');
        handleC = rectangle('Position', [positionC(1)-radius, positionC(2)-radius, 2*radius, 2*radius], 'curvature', [1 1], 'FaceColor', 'green');
        drawnow;

        % Temporary Finish Logic
        if finishA == 1 && finishB == 1 && finishC == 1
            break;
        end
        if notMovingA > 10 && notMovingB > 10 && notMovingC > 10
            break;
        end
        delete(handleA); delete(handleB); delete(handleC);
    end
end

function [positionA, finishA] = getPositionA(t, positionB, positionC)
    global radius
    Start = [10 10];
    Finish = [90 90];
    v = 1;
    if t > 0 && t < distance(Start, Finish)/v
        positionTemp = Start + v * t / distance(Start, Finish) * (Finish - Start);
        finishA = 0;
    else
        positionTemp = Finish;
        finishA = 1;
    end
    if distance(positionTemp, positionB) > 2*(radius+1) && distance(positionTemp, positionC) > 2*(radius+1)
        positionA = positionTemp;
    else
        positionA = Start;
        finishA = 0;
    end
end

function [positionB, finishB] = getPositionB(positionB, positionA, positionC)
    global radius
    Finish = [10 10];
    v = 1; dt = 1;
    if distance(positionB, Finish) > v
        positionTemp = positionB + v * dt / distance(positionB, Finish) * (Finish - positionB);
        finishB = 0;
    else
        positionTemp = Finish;
        finishB = 1;
    end
    if distance(positionTemp, positionA) > 2*(radius+1) && distance(positionTemp, positionC) > 2*(radius+1)
        positionB = positionTemp;
    else
        finishB = 0;
    end
end

function [positionC, finishC] = getPositionC(positionC, Finish, positionA, positionB)
    global radius
    v = 1; dt = 1;
    if distance(positionC, Finish) > v
        positionTemp = positionC + v * dt / distance(positionC, Finish) * (Finish - positionC);
        finishC = 0;
    else
        positionTemp = Finish;
        finishC = 1;
    end
    if distance(positionTemp, positionA) > 2*(radius+1) && distance(positionTemp, positionB) > 2*(radius+1)
        positionC = positionTemp;
    else
        finishC = 0;
    end
end

function dis = distance(u, v)
    dis = sqrt(sum((u - v).^2));
end

