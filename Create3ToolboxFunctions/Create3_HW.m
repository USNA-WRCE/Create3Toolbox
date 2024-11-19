classdef Create3_HW < matlab.mixin.SetGet
    % CREATE3_HW interfaces the Create3 hardware using iRobot ROS2 messages
    %
    %   Initialization
    %       crt = Create3_HW(name,ID);
    %
    %   Input(s)
    %       name - character array defining robot name space for the
    %              Create3 hardware
    %         ID - scalar integer specifying the ROS2 domain ID for the
    %              Create3 hardware
    %
    %   L. DeVries & M. Kutzer, 06Oct2024, USNA
    %

    properties
        node; % primary MATLAB node to communicate over ROS network
        %pos,quat,eul; % placeholder for MoCap data if used
        odom_pos; % position estimate from vehicle's onboard odometry
        odom_vel; % velocity estimate from vehicle's onboard odometry
        odom_quat; % 1x4 orientation estimate from vehicle's onboard odometry
        odom_angVel; % angular velocity measurement from vehicle's gyro
        odom_eul; % 1x3 euler angle measurement from vehicle's onboard IMU
        batteryPercent; % battery status 0-100%, from onboard sensor
        irData; % 1x7 array of infrared proximity sensor measurements
        wheelSpds; % 1x2 array of wheel speeds from onboard sensors
        slipStatus; % boolean variable indicating if wheels have slipped. Indicator of accuracy of odometry data
        accel;  % 1x3 array of accelerometer measurements from onboard sensor
        gyro; % 1x3 array of gyro measurements from onboard sensor
        imu_quat; % 1x4 array quaternion measurement from onboard sensor
        imu_eul; % 1x3 array of Euler angles from onboard sensor
        pose_sub; % pose subscriber object to read ROS network data
        imu_sub; % imu subscriber object to read ROS network data
        odom_sub; % odometry subscriber object to read ROS network data
        ir_sub; % infrared sensor data subscriber object to read ROS network data
        batt_sub; % battery subscriber object to read ROS network data
        wheelVel_sub; % wheel velocity subscriber object to read ROS network data
        slip_sub; % slip status subscriber object to read ROS network data
        cmd_pub; % ROS publisher object to send velocity commands
        led_pub; % ROS publisher object to LED color commands
        opMode; % oeprating mode: 0=basic mode, 1=advanced mode including custom create3 ROS messages
        % action clients

        undockClient; % ROS action client to send undock commands
        undockGoalMsg; % object to package undock command message
        dockClient; % ROS action client to send docking commands
        dockGoalMsg; % object to package dock command message
    end

    methods
        function obj = Create3_HW(robot_namespace,domain_id)
            % CREATE3_HW creates a hardware interface object for the Create3.
            % It uses the ros2 toolbox in matlab to connect to the device.
            %   obj = create3_HW(namespace,domain_id);
            %
            %   Input(s)
            %       namespace - character array defining the namespace of 
            %                   the robot            %
            %       domain_id - positive scalar value specifying the ros2 
            %                   domain ID for communication with the
            %                   Create3
            %
            %   L. DeVries, M. Kutzer 22Oct2024, USNA
            
            % Check input(s)
            narginchk(2,2);
            if isstring(robot_namespace)
                robot_namespace = char(robot_namespace);
            end
            if ~ischar(robot_namespace)
                error('Robot namespace must be defined as a character array.');
            end
            robot_namespace = lower(robot_namespace);
            
            % check that ROS2 toolbox is installed
            tlbxChck = contains(struct2array(ver), 'ROS Toolbox');
            if ~tlbxChck
                error("ROS Toolbox not installed. Create3 interface requires ROS Toolbox. Please install.")
            end
            % check that toolbox containing quat2eul is installed
            tlbxChck2 = contains(struct2array(ver), {'Navigation Toolbox','UAV Toolbox','Aerospace Toolbox','Robotics System Toolbox'});
            if ~tlbxChck2
                error("Missing toolbox containing quat2eul() function. Please install one of the following: Navigation Toolbox,UAV Toolbox, Aerospace Toolbox, or Robotics System Toolbox")
            end


            % objects that will work without custom toolbox
            obj.node = ros2node("node",domain_id);
            obj.pose_sub = ros2subscriber(obj.node,"/"+robot_namespace+"/pose","geometry_msgs/PoseStamped",@obj.poseCallBack);
            obj.imu_sub = ros2subscriber(obj.node,"/"+robot_namespace+"/imu","sensor_msgs/Imu",@obj.imuCallBack,'Reliability','besteffort','Durability','volatile','Depth',1);
            obj.odom_sub = ros2subscriber(obj.node,"/"+robot_namespace+"/odom","nav_msgs/Odometry",@obj.odomCallBack,'Reliability','besteffort','Durability','volatile','Depth',1);
            obj.batt_sub = ros2subscriber(obj.node,"/"+robot_namespace+"/battery_state","sensor_msgs/BatteryState",@obj.battCallBack,'Reliability','besteffort','Durability','volatile','Depth',1);
            obj.cmd_pub = ros2publisher(obj.node,"/"+robot_namespace+"/cmd_vel","geometry_msgs/Twist",'Reliability','besteffort','Durability','volatile','Depth',1);

            % check that custom message support is installed/configured correctly
            msgList = ros2("msg","list");
            chk = strcmp(msgList,'irobot_create_msgs/LedColor');
            if sum(chk)<1
                warning("Missing custom ros2 message support for create3. Proceeding with BASIC implementation. Please see https://www.mathworks.com/help/ros/ug/ros2-custom-message-support.html")
                obj.opMode = 0; % basic operating mode
                obj.slipStatus = "not supported in basic mode";
                obj.irData = "not supported in basic mode";
                obj.wheelSpds  = "not supported in basic mode";
            else
                obj.opMode = 1; % full operating mode
                % subscribers/publishers for custom ros2 messages
                obj.ir_sub = ros2subscriber(obj.node,"/"+robot_namespace+"/ir_intensity","irobot_create_msgs/IrIntensityVector",@obj.irCallback,'Reliability','besteffort','Durability','volatile','Depth',1);
                obj.wheelVel_sub = ros2subscriber(obj.node,"/"+robot_namespace+"/wheel_vels","irobot_create_msgs/WheelVels",@obj.wheelVelCallback,'Reliability','besteffort','Durability','volatile','Depth',1);
                obj.slip_sub = ros2subscriber(obj.node,"/"+robot_namespace+"/slip_status","irobot_create_msgs/SlipStatus",@obj.slipCallback,'Reliability','besteffort','Durability','volatile','Depth',1);
                obj.led_pub = ros2publisher(obj.node,"/"+robot_namespace+"/cmd_lightring","irobot_create_msgs/LightringLeds",'Reliability','besteffort','Durability','volatile','Depth',1);
                [obj.undockClient,obj.undockGoalMsg] = ros2actionclient(obj.node,"/"+robot_namespace+"/undock","irobot_create_msgs/Undock",CancelServiceQoS=struct(Depth=200,History="keeplast"),FeedbackTopicQoS=struct(Depth=200,History="keepall"));
                %[obj.dockClient,obj.dockGoalMsg] = ros2actionclient(obj.node,"/"+robot_namespace+"/dock","irobot_create_msgs/DockServo",CancelServiceQoS=struct(Depth=200,History="keeplast"),FeedbackTopicQoS=struct(Depth=200,History="keepall"));
                
            end


        end

        % --- Destructor
        function delete(obj)
            % Destructor to delete publishers and subscribers, action
            % clients

            fprintf('Destructor Called.\n')
            fprintf('\tDeleting publishers and subscribers...');
            clear obj.pose_sub;
            clear obj.imu_sum;
            clear obj.odom_sum;
            clear obj.cmd_pub;
            clear obj.batt_sub;
            if obj.opMode==1 % only clear if in advanced/full operating mode
                clear obj.ir_sub;
                clear obj.wheelVel_sub;
                clear obj.slip_sub;
                clear obj.led_pub;
            end
            fprintf('[DONE]\n');
            fprintf('\tDeleting objects...');
            fprintf('[DONE]\n')
        end

        function battCallBack(obj,msg)
            % battCallback parses battery messages from create and writes
            % them to object property battery
            %
            %   L. DeVries, M. Kutzer 22Oct2024, USNA

            obj.batteryPercent = msg.percentage;

        end

        function imuCallBack(obj,msg)
            % imuCallback parses IMU messages from create and writes them
            % to object properties accel, gyro, imu_quat, and imu_eul
            %
            %   L. DeVries, M. Kutzer 22Oct2024, USNA

            obj.accel(1) = msg.linear_acceleration.x;
            obj.accel(2) = msg.linear_acceleration.y;
            obj.accel(3) = msg.linear_acceleration.z;

            obj.gyro(1) = msg.angular_velocity.x;
            obj.gyro(2) = msg.angular_velocity.y;
            obj.gyro(3) = msg.angular_velocity.z;

            obj.imu_quat(1) = msg.orientation.w;
            obj.imu_quat(2) = msg.orientation.x;
            obj.imu_quat(3) = msg.orientation.y;
            obj.imu_quat(4) = msg.orientation.z;
            obj.imu_eul = quat2eul(obj.imu_quat,'XYZ');


        end

        function irCallback(obj,msg)
            % irCallback parses infrared intensity messages from create and
            % writes them to object property
            %
            %   L. DeVries, M. Kutzer 22Oct2024, USNA

            % odometry position estimate
            obj.irData = [msg.readings(:).value];

        end


        function odomCallBack(obj,msg)
            % odomCallback parses odometry messages from create and writes
            % them to object properties odom_pos, odom_quat, odom_eul,
            % odom_vel, odom_angVel
            %
            %   L. DeVries, M. Kutzer 22Oct2024, USNA

            % odometry position estimate
            obj.odom_pos(1) = msg.pose.pose.position.x;
            obj.odom_pos(2) = msg.pose.pose.position.y;
            obj.odom_pos(3) = msg.pose.pose.position.z;

            % odometry orientation estimate
            obj.odom_quat(1) = msg.pose.pose.orientation.w;
            obj.odom_quat(2) = msg.pose.pose.orientation.x;
            obj.odom_quat(3) = msg.pose.pose.orientation.y;
            obj.odom_quat(4) = msg.pose.pose.orientation.z;
            obj.odom_eul = quat2eul(obj.odom_quat,'XYZ');

            % odometry translational velocity estimate
            obj.odom_vel(1) = msg.twist.twist.linear.x;
            obj.odom_vel(2) = msg.twist.twist.linear.y;
            obj.odom_vel(3) = msg.twist.twist.linear.z;

            % odometry angyular velocity estimate
            obj.odom_angVel(1) = msg.twist.twist.angular.x;
            obj.odom_angVel(2) = msg.twist.twist.angular.y;
            obj.odom_angVel(3) = msg.twist.twist.angular.z;
        end

        function slipCallback(obj,msg)
            % slipCallback parses wheel slip messages from create3 and
            % writes them to object property slipStatus
            %
            %   L. DeVries, M. Kutzer 30Oct2024, USNA

            obj.slipStatus = msg.is_slipping;
        end

        function wheelVelCallback(obj,msg)
            % wheelVelCallback parses wheel speed messages from create and
            % writes them to object properties
            % wheelSpds = [spd_left;spd_right]
            %
            %   L. DeVries, M. Kutzer 30Oct2024, USNA

            obj.wheelSpds = [msg.velocity_left; msg.velocity_right];
        end


        function [pose,vel] = getOdomPose(obj)
            % getOdomPose queries object pose properties and outputs:
            %       - pose = [position euler_angles]
            %       - vel = [translational_velocity angular_velocity]
            %
            %   L. DeVries, M. Kutzer 22Oct2024, USNA

            pose = [obj.odom_pos obj.odom_eul];
            vel = [obj.odom_vel obj.odom_angVel];
        end

        function [imu_data,quat_data,imu_eul] = getImuData(obj)
            % getImuData queries object imu properties and outputs:
            %       - imu_data: acceleration and gyro measurements
            %       - quat_data: quaternion measurement
            %       - imu_eul: euler angles stemming from
            %                  quaternion
            %
            %   L. DeVries, M. Kutzer 22Oct2024, USNA

            imu_data = [obj.accel obj.gyro];
            quat_data = obj.imu_quat;
            imu_eul = quat2eul(obj.odom_quat,'XYZ');
        end

        function setVelCmd(obj,u,r)
            % setVelCmd(u,r) sends a speed and turn rate
            % command to the create3's command velocity topic.
            %
            %   Inputs:
            %       u: forward speed, scaler: (limited to [0,0.306] m/s)
            %       r: turn rate, scaler: rad/s
            %
            %
            %   L. DeVries, M. Kutzer 22Oct2024, USNA

            msg = ros2message("geometry_msgs/Twist");
            if u>0.306 % enforce onboard speed limit
                u = 0.306;
                disp('u input exceeds maximum speed setting of 0.306 m/s')
            end
            if u<0
                u = 0;
                disp('u input exceeds default safety backup speed of 0 m/s')
            end
            msg.linear.x = u;
            msg.angular.z = r;
            send(obj.cmd_pub,msg)
        end

        function setLEDCmd(obj,color)
            % setLEDCmd(color) sends a command to change the Create3 LED
            % ring to the color specified in the string "color"
            %
            %   Input:
            %       color: 6x3 array containing red,green,blue colors values
            %              each on the scale 0-255 for each of the 6 LEDs
            %              in the lightring
            %   Example: set LEDs to random color
            %       color = 255*rand(6,3);
            %       crt.setLEDCmd(color)
            %   L. DeVries, M. Kutzer 22Oct2024, USNA
            if size(color,1)~=6 && size(color,2)~=3
                error('Incorrect input size, must be 6x3')
            end
            if obj.opMode ==0
                error('LED control not available in basic mode')
            else
                msg = ros2message("irobot_create_msgs/LightringLeds");
                msg.override_system = true;
                for mm = 1:6
                    msg.leds(mm).red = uint8(round(color(mm,1)));
                    msg.leds(mm).green = uint8(round(color(mm,2)));
                    msg.leds(mm).blue = uint8(round(color(mm,3)));
                end
                send(obj.led_pub,msg)
            end
        end

        function setLEDDefault(obj)
            % setLEDDefault sends a command to return the Create3 LED
            % ring to its default behavior
            %
            %   Example:
            %
            %       crt.setLEDDefault()
            %
            %   L. DeVries, M. Kutzer 7Nov2024, USNA
            if obj.opMode==0
                error('LED control not supported in basic mode')
            else
                msg = ros2message("irobot_create_msgs/LightringLeds");
                msg.override_system = false;
                send(obj.led_pub,msg)
            end
        end

        function undock(obj)
            % undock() sends a command to undock the create3
            %
            %
            %   L. DeVries, M. Kutzer 22Oct2024, USNA
            if obj.opMode==0
                error('Functionality not supported in basic mode')
            else
                goalHandle = sendGoal(obj.undockClient,obj.undockGoalMsg);
                pause(5); % pause before allowing user to send another command
            end
        end

    end
end