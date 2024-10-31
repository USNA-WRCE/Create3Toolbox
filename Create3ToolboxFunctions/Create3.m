classdef Create3 < matlab.mixin.SetGet
    % CREATE3 interfaces the Create3 hardware using iRobot ROS2 messages
    %
    %   Initialization
    %       sim = Create3;
    %
    %   L. DeVries & M. Kutzer, 06Oct2024, USNA

    properties
        node;
        %pos,quat,eul; % placeholder for MoCap data if used
        odom_pos,odom_vel,odom_quat,odom_angVel,odom_eul;
        batteryPercent, irData, wheelSpds,slipStatus;
        accel,gyro,imu_quat,imu_eul;
        pose_sub,imu_sub,odom_sub;
        ir_sub,batt_sub,wheelVel_sub,slip_sub;
        cmd_pub, led_pub;
        % action clients
        undockClient, undockGoalMsg;
        dockClient, dockGoalMsg;
    end

    methods
        function obj = create3_HW(robot_namespace,domain_id)
            % CREATE3_HW creates a hardware interface object for the Create3.
            % It uses the ros2 toolbox in matlab to connect to the device.
            %   obj = create3_HW(namespace,domain_id);
            %
            %   Input(s)
            %       namespace - string definintg the namespace the robot is
            %                   operating in
            %                          
            %       domain_id - scalar value specifying the ros2 domain ID
            %                   underwhich the create is communicating
            %
            %   L. DeVries, M. Kutzer 22Oct2024, USNA

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
            % check that custom message support is installed/configured correctly
            msgList = ros2("msg","list");
            chk = strcmp(msgList,'irobot_create_msgs/LedColor');
            if sum(chk)<1
                error("Missing custom ros2 message support for create3. Please see https://www.mathworks.com/help/ros/ug/ros2-custom-message-support.html")
            end

            % objects that will work without custom toolbox
            obj.node = ros2node("node",domain_id);
            obj.pose_sub = ros2subscriber(obj.node,"/"+robot_namespace+"/pose","geometry_msgs/PoseStamped",@obj.poseCallBack);
            obj.imu_sub = ros2subscriber(obj.node,"/"+robot_namespace+"/imu","sensor_msgs/Imu",@obj.imuCallBack,'Reliability','besteffort','Durability','volatile','Depth',1);
            obj.odom_sub = ros2subscriber(obj.node,"/"+robot_namespace+"/odom","nav_msgs/Odometry",@obj.odomCallBack,'Reliability','besteffort','Durability','volatile','Depth',1);
            obj.batt_sub = ros2subscriber(obj.node,"/"+robot_namespace+"/battery_state","sensor_msgs/BatteryState",@obj.battCallBack,'Reliability','besteffort','Durability','volatile','Depth',1);
            obj.cmd_pub = ros2publisher(obj.node,"/"+robot_namespace+"/cmd_vel","geometry_msgs/Twist",'Reliability','besteffort','Durability','volatile','Depth',1);
            
            % subscribers/publishers for custom ros2 messages 
            obj.ir_sub = ros2subscriber(obj.node,"/"+robot_namespace+"/ir_intensity","irobot_create_msgs/IrIntensityVector",@obj.irCallback,'Reliability','besteffort','Durability','volatile','Depth',1);
            obj.wheelVel_sub = ros2subscriber(obj.node,"/"+robot_namespace+"/wheel_vels","irobot_create_msgs/WheelVels",@obj.wheelVelCallback,'Reliability','besteffort','Durability','volatile','Depth',1);
            obj.slip_sub = ros2subscriber(obj.node,"/"+robot_namespace+"/slip_status","irobot_create_msgs/SlipStatus",@obj.slipCallback,'Reliability','besteffort','Durability','volatile','Depth',1);
            obj.led_pub = ros2publisher(obj.node,"/"+robot_namespace+"/cmd_lightring","irobot_create_msgs/LightringLeds",'Reliability','besteffort','Durability','volatile','Depth',1);
            [obj.undockClient,obj.undockGoalMsg] = ros2actionclient(obj.node,"/"+robot_namespace+"/undock","irobot_create_msgs/Undock",CancelServiceQoS=struct(Depth=200,History="keeplast"),FeedbackTopicQoS=struct(Depth=200,History="keepall"));
            %[obj.dockClient,obj.dockGoalMsg] = ros2actionclient(obj.node,"/"+robot_namespace+"/dock","irobot_create_msgs/DockServo",CancelServiceQoS=struct(Depth=200,History="keeplast"),FeedbackTopicQoS=struct(Depth=200,History="keepall"));
        end
        
        % --- Destructor
        function delete(obj)
            fprintf('Destructor Called.\n')
            fprintf('\tDeleting publishers and subscribers...');
            clear obj.pose_sub;
            clear obj.imu_sum;
            clear obj.odom_sum;
            clear obj.cmd_pub;
            clear obj.batt_sub;
            clear obj.ir_sub;
            clear obj.wheelVel_sub;
            clear obj.slip_sub;
            clear obj.led_pub;
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
            %       - pose = [position euler angles]
            %       - vel = [translational velocity angular velocity]
            %
            %   L. DeVries, M. Kutzer 22Oct2024, USNA

            pose = [obj.odom_pos obj.odom_eul];
            vel = [obj.odom_vel obj.odom_angVel];
        end     

        function [imu_data,quat_data,imu_eul] = getImuData(obj)
            % getImuData queries object imu properties and outputs:
            %       - imu_data: acceleration and gyro measurements
            %       - quat_data: quaternion measurement
            %       - imu_eul: euler angle measurement stemming from
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
            %       u: forward speed (limited to 
            %       - quat_data: quaternion measurement
            %       - imu_eul: euler angle measurement stemming from
            %                  quaternion
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

        function undock(obj)

            goalHandle = sendGoal(obj.undockClient,obj.undockGoalMsg)
        end

    end
end