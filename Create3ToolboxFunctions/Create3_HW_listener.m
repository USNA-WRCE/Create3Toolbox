classdef Create3_HW_listener < matlab.mixin.SetGet
    % CREATE3_HW_listener interfaces the Create3 hardware using iRobot ROS2
    % messages, but only listens to topics and does not allow the user to
    % send any commands.
    %
    %   Initialization
    %       crt = Create3_HW_listener(name,ID);
    %
    %   Input(s)
    %       name - character array defining robot name space for the
    %              Create3 hardware
    %         ID - scalar integer specifying the ROS2 domain ID for the
    %              Create3 hardware
    %
    %   L. DeVries & M. Kutzer, 18Nov2025, USNA
    %

    properties (Access=private)
        node; % primary MATLAB node to communicate over ROS network
        pose_sub; % pose subscriber object to read ROS network data
        imu_sub; % imu subscriber object to read ROS network data
        odom_sub; % odometry subscriber object to read ROS network data
        ir_sub; % infrared sensor data subscriber object to read ROS network data
        batt_sub; % battery subscriber object to read ROS network data
        wheelVel_sub; % wheel velocity subscriber object to read ROS network data
        slip_sub; % slip status subscriber object to read ROS network data
        
        opMode; % oeprating mode: 0=basic mode, 1=advanced mode including custom create3 ROS messages
    end
    properties
        odom_pos; % position estimate from vehicle's onboard odometry with matlab specified user offset
        raw_odom_pos; % position estimate from vehicle's onboard odometry
        raw_odom_eul; % position estimate from vehicle's onboard odometry
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
        reset_offsets; % [x y z yaw pitch roll] position offset since create3 reset pose always zeros out the odometry. TODO: fix quaternion property also
        % action clients

    end

    methods
        function obj = Create3_HW_listener(robot_namespace,domain_id)
            % CREATE3_HW_listener creates a hardware interface object for
            % the Create3 that only listens for published data.
            % It uses the ros2 toolbox in matlab to connect to the device.
            %   obj = create3_HW_listener(namespace,domain_id);
            %
            %   Input(s)
            %       namespace - character array defining the namespace of 
            %                   the robot            %
            %       domain_id - positive scalar value specifying the ros2 
            %                   domain ID for communication with the
            %                   Create3
            %
            %   L. DeVries, M. Kutzer 18Nov2025, USNA
            
            % Check input(s)
            narginchk(2,2);
            if isstring(robot_namespace)
                robot_namespace = char(robot_namespace);
            end
            if ~ischar(robot_namespace)
                error('Robot namespace must be defined as a character array.');
            end
            % correct robot namespace if entered capital letters
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
            
            if strcmpi(robot_namespace,robot_namespace)~=1
                error("Robot namespace must be all lowercase letters.")
            end
            
            % set initial pose offsets to zero
            obj.reset_offsets = zeros(1,6);
            
            % create "random" node name based on time
            tm = datetime('now');
            ndName = string(['node_' num2str(round(second(tm)*1000))]);
            % objects that will work without custom toolbox
            obj.node = ros2node(ndName,domain_id);
            obj.pose_sub = ros2subscriber(obj.node,"/"+robot_namespace+"/pose","geometry_msgs/PoseStamped",@obj.poseCallBack);
            obj.imu_sub = ros2subscriber(obj.node,"/"+robot_namespace+"/imu","sensor_msgs/Imu",@obj.imuCallBack,'Reliability','besteffort','Durability','volatile','Depth',1);
            obj.odom_sub = ros2subscriber(obj.node,"/"+robot_namespace+"/odom","nav_msgs/Odometry",@obj.odomCallBack,'Reliability','besteffort','Durability','volatile','Depth',1);
            obj.batt_sub = ros2subscriber(obj.node,"/"+robot_namespace+"/battery_state","sensor_msgs/BatteryState",@obj.battCallBack,'Reliability','besteffort','Durability','volatile','Depth',1);
            
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
            end

            % TODO: add a check to see if data starts coming through
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
            clear obj.batt_sub;
            if obj.opMode==1 % only clear if in advanced/full operating mode
                clear obj.ir_sub; % clear full mode subs/pubs
                clear obj.wheelVel_sub;
                clear obj.slip_sub;
            end
            clear obj.node % delete ros2 node
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
            obj.odom_pos(1) = msg.pose.pose.position.x+obj.reset_offsets(1);
            obj.odom_pos(2) = msg.pose.pose.position.y+obj.reset_offsets(2);
            obj.odom_pos(3) = msg.pose.pose.position.z+obj.reset_offsets(3);

            % odometry position estimate
            obj.raw_odom_pos(1) = msg.pose.pose.position.x;
            obj.raw_odom_pos(2) = msg.pose.pose.position.y;
            obj.raw_odom_pos(3) = msg.pose.pose.position.z;

            % odometry orientation estimate
            obj.odom_quat(1) = msg.pose.pose.orientation.w;
            obj.odom_quat(2) = msg.pose.pose.orientation.x;
            obj.odom_quat(3) = msg.pose.pose.orientation.y;
            obj.odom_quat(4) = msg.pose.pose.orientation.z;
            obj.odom_eul = wrapToPi(quat2eul(obj.odom_quat,'XYZ')+obj.reset_offsets(4:6)); 
            
            abc(1) = msg.pose.pose.orientation.w;
            abc(2) = msg.pose.pose.orientation.x;
            abc(3) = msg.pose.pose.orientation.y;
            abc(4) = msg.pose.pose.orientation.z;
            obj.raw_odom_eul = wrapToPi(quat2eul(abc,'XYZ'));
        
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
        end % end getImuData function
    end % end methods
end % end class