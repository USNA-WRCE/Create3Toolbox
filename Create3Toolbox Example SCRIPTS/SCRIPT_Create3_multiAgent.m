crt{1} = Create3_HW('mahan',43);
pause(3)
crt{2} = Create3_HW('sampson',44);

%%
% undock all robots
for ii = 1:length(crt)
    crt{ii}.undock
end



%% reset poses
% undock all robots
for ii = 1:length(crt)
    crt{ii}.resetPose
end

%% turn all robots 90 degrees
for ii = 1:length(crt)
    % note, ROS2 Action calls are blocking in MATLAB, so the next robot
    % will not start the action until the previous robot is finished
    crt{ii}.rotateAngle_rad(-pi)
end


%% drive both straight
for qq = 1:40
    if qq<20
        for ii = 1:length(crt)
            % publishing calls are non-blocking, so these run essentially
            % simultaneously to all robots
            crt{ii}.setVelCmd(0.1,0)
        end
    else
        for ii = 1:length(crt)
            crt{ii}.setVelCmd(0.1,0.4)
        end
    end
    pause(0.2) % run at 5Hz, note, the robots timeout at 1 Hz, so must send commands with atleast 1 Hz
end


%% dock all robots
for ii = 1:length(crt)
    % note, ROS2 Action calls are blocking in MATLAB, so the next robot
    % will not start the action until the previous robot is finished
    crt{ii}.dock
end