% SCRIPT_configureROS2_messageCompile.m

% requirements: Windows OS, Python 3.10 or earlier, Microsoft Visual Studio
%               2022 or newer, MATLAB Coder toolbox/add-on


%% Steps:

% 1. make sure python is configured
% 1a. you must use python 3.10 or earlier for MATLAB 2024b and older
% 1b. Go to environment -> preferences -> ROS Toolbox -> Opern ROS Toolbox
%     Preferences
% 1c. Configure path to python installation and press OK

%% 2. Navigate to folder containing custom_create3 folder (i.e. folder
%    containing the irobot_create_msgs definitions, one level back)

folderPath = fullfile(pwd,"custom_create3");


%% 3. run ros2genmsg(folderPath)

ros2genmsg(folderPath,"CreateShareableFile",true)