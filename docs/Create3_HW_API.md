# Create3_HW — MATLAB Interface to iRobot Create3 (ROS 2)

### Repository
[https://github.com/USNA-WRCE/Create3Toolbox](https://github.com/USNA-WRCE/Create3Toolbox)

### Authors
**L. DeVries & M. Kutzer**, U.S. Naval Academy  
*Initial release: 06-Oct-2024*

---

## Overview

`Create3_HW` is a MATLAB class that provides a high-level interface to the **iRobot Create3** robot over **ROS 2**.  
It allows users to control motion, LEDs, audio, and docking actions while reading odometry, IMU, and battery data—without needing deep ROS 2 knowledge.

The class supports:
- **Basic Mode** – Standard ROS 2 messages (no custom packages required)  
- **Advanced Mode** – Full functionality using custom iRobot Create3 ROS 2 messages

---

## Syntax

```
crt = Create3_HW(robot_namespace, domain_id)
```

Inputs
| Name | Type | Description |
| --- | --- | --- |
| robot_namespace	|char	|Robot namespace on the ROS 2 network (must be lowercase).|
|domain_id	| integer scalar	| ROS 2 Domain ID for communication with the Create3.|

# Example
```
% Create interface object
crt = Create3_HW('mahan', 43);

% Command motion
crt.setVelCmd(0.2, 0.1);
pause(2);
crt.setVelCmd(0, 0);

% Query odometry
[pose, vel] = crt.getOdomPose();

% Advanced-mode features
crt.beep(440, 1.0);
crt.setLEDCmd(255*rand(6,3));
```


# Requirements
* MATLAB ROS 2 Toolbox
* A toolbox containing quat2eul() (Navigation, UAV, Aerospace, or Robotics System Toolbox)
* For Advanced Mode: Custom iRobot Create3 ROS 2 messages installed (see MathWorks’
ROS 2 Custom Message Support)

# Properties
Core ROS 2 Interfaces

| Property  | Description |
| ------------- | ------------- |
| ```node```  | Primary MATLAB ROS 2 node.  |
| ```cmd_pub```	| Publisher for velocity commands (geometry_msgs/Twist).|
| ```pose_sub```, ```imu_sub```, ```odom_sub```, ```batt_sub```	| Subscribers for telemetry data.|
| ```led_pub```, ```beep_pub```, ```ir_sub```, ```wheelVel_sub```, ```slip_sub```	| Advanced-mode publishers/subscribers.| 


# State & Sensor Data

| Command | Description |
| --- | --- |
| odom_pos	| Estimated position [x y z] (m).|
| odom_eul	| Orientation [yaw pitch roll] (rad).|
| odom_vel	| Linear velocity [vx vy vz] (m/s).|
| odom_angVel	| Angular velocity [wx wy wz] (rad/s).|
| raw_odom_pos, raw_odom_eul	| Uncorrected odometry data.|
|imu_quat, imu_eul	| IMU orientation (quaternion and Euler).|
|accel, gyro	| IMU acceleration and angular rate data.|
|batteryPercent	| Battery charge percentage (0–100%).|
|irData	| Infrared proximity readings (1×7).|
|wheelSpds	| Wheel speeds [left; right] (rad/s).|
|slipStatus	| Boolean indicator for wheel slip.|

Any of these properties can be accessed by invoking the object. For example

```
obj.odom_pos
```

returns the odometry position as a 1x3 array.

# Control & Mode
When the MATLAB object is created, MATLAB checks the version being used before declaring the operating mode of the Create3. Since the custom ROS messages are only compiled for versions 2023a-2024b, if the version is outside this range it will only allow basic mode operations that include the standard ROS message types.

| Property | Description |
| --- | --- |
| opMode	| Operating mode: 0 = basic, 1 = advanced.|

# Action & Service Clients

| Command | Description |
| --- | --- |
| ```undockClient```, ```dockClient```	| Docking/undocking actions.|
|```drvDistClient```, ```rotAngClient```, ```wallClient```, ```navClient```	| Motion and navigation actions.|
|```resetPoseClient```	| Service for resetting odometry frame.|

# Methods
Constructor & Destructor
```
obj = Create3_HW(namespace, domain_id)
```
Creates a new hardware interface, validates toolboxes, and initializes ROS 2 communication.

```
delete(obj)
```
Cleans up ROS 2 publishers, subscribers, and clients on object deletion.

# Sensor & State Access
```
[pose, vel] = obj.getOdomPose()
```
Returns:

pose — [x y z yaw pitch roll]

vel — [vx vy vz wx wy wz]

```
[imu_data, quat_data, imu_eul] = obj.getImuData()
```
Returns:

imu_data = [ax ay az gx gy gz]

quat_data = [qw qx qy qz]

imu_eul = Euler angles from quaternion

## Motion Control
```
setVelCmd(u, r)
```
Send forward speed and turn rate commands.

u — Linear speed (m/s) [0–0.306]

r — Angular rate (rad/s)

```
driveDistance(dist, maxSpd)
```
Drive a specified distance at up to maxSpd.
(Advanced Mode only; ROS 2 Humble or later required.)

```
rotateAngle_rad(ang)
```
Rotate by a specified angle in radians.
(Advanced Mode only.)

```
nav2pos(position, orientation, trackOr)
```
Navigate to a position and orientation.

position = [x y z] target in odom frame

orientation = desired yaw (rad)

trackOr = true to match yaw, false for position only

## Docking Control
```
undock(obj)
```
Undocks the Create3 and waits for completion. (Advanced Mode only.)

```
dock(obj)
```
Commands the Create3 to dock. (Requires updated firmware and message set.)

## LED & Audio Control

```
setLEDCmd(obj, color)
```
Set LED ring colors.
color must be 6×3 RGB matrix (0–255).

```
color = 255*rand(6,3);
crt.setLEDCmd(color);
```

```
crt.setLEDDefault();
```
Restore LED ring to system default behavior.

```
beep(freq, duration)
```
Play a tone at freq (Hz) for duration (seconds).

## Pose Management
```
resetPose(obj, position, yaw, pitch, roll)
```
Reset odometry frame with user-defined offsets.

```
zeroPose(obj)
```
Reset odometry to zero ([0 0 0 0 0 0]).

## Wall Following
```
wallFollow(side, duration)
```
Follow a wall for a specified time. (Advanced Mode only.)

side = 1 (left) or −1 (right)

duration = seconds

## Internal Callbacks

| Method | Purpose |
| --- | --- |
| ```odomCallBack```, ```imuCallBack```, ```irCallback```	| Parse incoming telemetry messages.|
|```battCallBack```, ```wheelVelCallback```, ```slipCallback```	| Update sensor data.|
|```helperDockResultCallback```, ```helperUndockResultCallback```	| Handle ROS 2 action results.|

# Notes
Automatically checks for required toolboxes and message support.

Default speed and QoS settings match Create3 hardware limits.

Advanced features (dock, nav2pos, etc.) require firmware supporting ROS 2 Humble or newer.

# Related Links
* Main Repository (https://github.com/USNA-WRCE/Create3Toolbox)
* ROS 2 Toolbox Documentation (https://www.mathworks.com/help/ros/)
* ROS 2 Custom Message Support Guide (https://www.mathworks.com/help/ros/ug/ros2-custom-message-support.html)
* iRobot Create 3 Developer Site (https://iroboteducation.github.io/create3_docs/)

# License
See the LICENSE file in this repository.
