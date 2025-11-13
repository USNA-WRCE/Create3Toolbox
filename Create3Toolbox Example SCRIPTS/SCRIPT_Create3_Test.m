% Test Create3 interface in MATLAB using ROS2 Toolbox


%% Establish Create3 object

% real hardware
name = 'delaware';
ID = 58;
crt = Create3_HW(name,ID)

% sim hardware
% clear
% clc
% crt = create3_sim([0;0],pi/3);

%% Drive in a loop
while true
    crt.setVelCmd(0.2,0.1);
    pause(0.1);
    drawnow;
end

%% Drive Open loop in a triangle


notDone = true
count = 0;
while(notDone)
    % calculate desired heading
    des_psi = wrapToPi(crt.odom_eul(3)+60*pi/180)

    % turn to desired heading
    while(abs(des_psi-crt.odom_eul(3))>3*pi/180)
        crt.setVelCmd(0.0,0.1*(des_psi-crt.odom_eul(3)))
        crt.odom_eul
        pause(0.1)
    end
    crt.setVelCmd(0,0)

    % drive straight for a 0.25 m
    % des_dist = crt.odom_pos(1)+0.1;
    % while(abs(des_dist-crt.odom_pos(1))>0.05)
    %     crt.setVelCmd(0.5*(des_dist-crt.odom_pos(1)),0)
    %     crt.odom_pos
    %     pause(0.1)
    %     count  = count + 1;
    % end
    if count==3
        notDone = false
    end
end
