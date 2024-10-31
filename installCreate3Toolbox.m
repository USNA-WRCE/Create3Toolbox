function installCreate3Toolbox(replaceExisting)
% INSTALLCREATE3TOOLBOX installs Create3 Toolbox for MATLAB
%
%   INSTALLCREATE3TOOLBOX installs Create3 Toolbox for MATLAB into the 
%   following locations:
%                             Source: Destination
%           URToolboxFunctions: matlabroot\toolbox\create3
%
%   INSTALLCREATE3TOOLBOX(true) installs Create3 Toolbox regardless of 
%   whether a copy of the Create3 toolbox exists in the MATLAB root.
%
%   INSTALLCREATE3TOOLBOX(false) installs Create3 Toolbox only if no copy 
%   of the Create3 toolbox exists in the MATLAB root.
%
%   M. Kutzer, 31Oct2024, USNA

% Updates


% TODO - Allow users to create a local version if admin rights are not
% possible.

%% Define support toolboxes
% Updated to specify [REPO],[USER/ORGANIZATION]
supportToolboxes = {...
    'Create3Simulation','USNA-WRCE';...
    };

%% Assign tool/toolbox specific parameters
dirName = 'create3';

%% Check inputs
if nargin == 0
    replaceExisting = [];
end

%% Installation error solution(s)
adminSolution = sprintf(...
    ['Possible solution:\n',...
     '\t(1) Close current instance of MATLAB\n',...
     '\t(2) Open a new instance of MATLAB "as administrator"\n',...
     '\t\t(a) Locate MATLAB shortcut\n',...
     '\t\t(b) Right click\n',...
     '\t\t(c) Select "Run as administrator"\n']);

%% Check for toolbox directory
toolboxRoot  = fullfile(matlabroot,'toolbox',dirName);
isToolbox = exist(toolboxRoot,'file');
if isToolbox == 7
    % Apply replaceExisting argument
    if isempty(replaceExisting)
        choice = questdlg(sprintf(...
            ['MATLAB Root already contains the Create3 Toolbox.\n',...
            'Would you like to replace the existing toolbox?']),...
            'Replace Existing Create3 Toolbox','Yes','No','Cancel','Yes');
    elseif replaceExisting
        choice = 'Yes';
    else
        choice = 'No';
    end
    % Replace existing or cancel installation
    switch choice
        case 'Yes'
            rmpath(toolboxRoot);
            [isRemoved, msg, msgID] = rmdir(toolboxRoot,'s');
            if isRemoved
                fprintf('Previous version of Create3 Toolbox for MATLAB removed successfully.\n');
            else
                fprintf('Failed to remove old Create3 Toolbox for MATLAB folder:\n\t"%s"\n',toolboxRoot);
                fprintf(adminSolution);
                error(msgID,msg);
            end
        case 'No'
            fprintf('Create3 Toolbox for MATLAB currently exists, installation cancelled.\n');
            return
        case 'Cancel'
            fprintf('Action cancelled.\n');
            return
        otherwise
            error('Unexpected response.');
    end
end

%% Create Scorbot Toolbox Path
[isDir,msg,msgID] = mkdir(toolboxRoot);
if isDir
    fprintf('Create3 Toolbox folder created successfully:\n\t"%s"\n',toolboxRoot);
else
    fprintf('Failed to create Create3 Toolbox folder:\n\t"%s"\n',toolboxRoot);
    fprintf(adminSolution);
    error(msgID,msg);
end

%% Migrate toolbox folder contents
toolboxContent = 'Create3ToolboxFunctions';
if ~isdir(toolboxContent)
    error(sprintf(...
        ['Change your working directory to the location of "installCreate3Toolbox.m".\n',...
         '\n',...
         'If this problem persists:\n',...
         '\t(1) Unzip your original download of "URToolbox" into a new directory\n',...
         '\t(2) Open a new instance of MATLAB "as administrator"\n',...
         '\t\t(a) Locate MATLAB shortcut\n',...
         '\t\t(b) Right click\n',...
         '\t\t(c) Select "Run as administrator"\n',...
         '\t(3) Change your "working directory" to the location of "installCreate3Toolbox.m"\n',...
         '\t(4) Enter "installCreate3Toolbox" (without quotes) into the command window\n',...
         '\t(5) Press Enter.']));
end
files = dir(toolboxContent);
wb = waitbar(0,'Copying Create3 Toolbox for MATLAB toolbox contents...');
n = numel(files);
fprintf('Copying Create3 Toolbox for MATLAB contents:\n');
for i = 1:n
    % source file location
    source = fullfile(toolboxContent,files(i).name);
    % destination location
    destination = toolboxRoot;
    if files(i).isdir
        switch files(i).name
            case '.'
                %Ignore
            case '..'
                %Ignore
            otherwise
                fprintf('\t%s...',files(i).name);
                nDestination = fullfile(destination,files(i).name);
                [isDir,msg,msgID] = mkdir(nDestination);
                if isDir
                    [isCopy,msg,msgID] = copyfile(source,nDestination,'f');
                    if isCopy
                        fprintf('[Complete]\n');
                    else
                        bin = msg == char(10);
                        msg(bin) = [];
                        bin = msg == char(13);
                        msg(bin) = [];
                        fprintf('[Failed: "%s"]\n',msg);
                    end
                else
                    bin = msg == char(10);
                    msg(bin) = [];
                    bin = msg == char(13);
                    msg(bin) = [];
                    fprintf('[Failed: "%s"]\n',msg);
                end
        end
    else
        fprintf('\t%s...',files(i).name);
        [isCopy,msg,msgID] = copyfile(source,destination,'f');
        
        if isCopy == 1
            fprintf('[Complete]\n');
        else
            bin = msg == char(10);
            msg(bin) = [];
            bin = msg == char(13);
            msg(bin) = [];
            fprintf('[Failed: "%s"]\n',msg);
        end
    end
    waitbar(i/n,wb);
end
set(wb,'Visible','off');
delete(wb);

%% Migrate and install ROS2 messages
% Get current MATLAB release
ver = version('-release');
% Define folder containing custom ROS2 messages
pnameROS2 = 'Create3ToolboxROS2MsgSupport';
% Define *.zip filename for custom ROS2 messages for MATLAB version
fnameROS2 = sprintf('matlab_msg_gen_%s.zip',ver);

% Check if file exists
source = fullfile(pnameROS2,fnameROS2);
if exist(source,'file')
    % Custon message file exists
    % -> Copy file
    destination = fullfile(toolboxRoot,'matlab_msg_gen.zip');
    tfROS2 = copyfile(source,destination);
    if tfROS2
        % -> Extract messages
        ros2genmsg(toolboxRoot)
    else
        fprintf('Unable to copy custom ROS2 messages.\n');
    end
else
    % No custom message file exists
    fprintf([...
        'Custom messages for MATLAB %s have not been generated,',...
        'the ROS2 messages for the following MATLAB releases are ',...
        'available:\n'],ver);
    d = dir( fullfile(pnameROS2,'*.zip') );
    fnamesROS2 = {d.name};
    for i = 1:numel(fnamesROS2)
        fprintf('\tMATLAB %s\n',fnamesROS2{i}{1}(16:end-4));
    end
end

%% Save toolbox path
%addpath(genpath(toolboxRoot),'-end');
addpath(toolboxRoot,'-end');
savepath;

%% Rehash toolbox cache
fprintf('Rehashing Toolbox Cache...');
rehash TOOLBOXCACHE
fprintf('[Complete]\n');

%% Install/Update required toolboxes
for i = 1:size(supportToolboxes,1)
    ToolboxUpdate( supportToolboxes{i,1},supportToolboxes{i,2} );
end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% TOOLBOX UPDATE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 26Mar2021
function ToolboxUpdate(toolboxName,orgName)

%% Check input(s)
narginchk(1,2);
if nargin < 2
    orgName = 'kutzer';
end

%% Setup functions
ToolboxVer = str2func( sprintf('%sToolboxVer',toolboxName) );
installToolbox = str2func( sprintf('install%sToolbox',toolboxName) );

%% Check current version
try
    A = ToolboxVer;
catch ME
    A = [];
    fprintf('No previous version of %s detected.\n',toolboxName);
end

%% Setup temporary file directory
% TODO - check "ok"
fprintf('Creating %s Toolbox temporary directory...',toolboxName);
tmpFolder = sprintf('%sToolbox',toolboxName);
pname = fullfile(tempdir,tmpFolder);
if isfolder(pname)
    % Remove existing directory
    [ok,msg] = rmdir(pname,'s');
end
% Create new directory
[ok,msg] = mkdir(tempdir,tmpFolder);
fprintf('SUCCESS\n');

%% Download and unzip toolbox (GitHub)
% UPDATED: 07Sep2021, M. Kutzer
%url = sprintf('https://github.com/kutzer/%sToolbox/archive/master.zip',toolboxName); <--- Github removed references to "master"
%url = sprintf('https://github.com/kutzer/%sToolbox/archive/refs/heads/main.zip',toolboxName);

% Check possible branches
defBranches = {'master','main'};
for i = 1:numel(defBranches)
    % Check default branch
    defBranch = defBranches{i};
    url = sprintf('https://github.com/%s/%sToolbox/archive/refs/heads/%s.zip',...
        orgName,toolboxName,defBranch);
    
    % Download and unzip repository
    fprintf('Downloading the %s Toolbox ("%s" branch)...',toolboxName,defBranch);
    try
        %fnames = unzip(url,pname);
        %urlwrite(url,fullfile(pname,tmpFname));
        tmpFname = sprintf('%sToolbox-master.zip',toolboxName);
        websave(fullfile(pname,tmpFname),url);
        fnames = unzip(fullfile(pname,tmpFname),pname);
        delete(fullfile(pname,tmpFname));
        
        fprintf('SUCCESS\n');
        confirm = true;
        break
    catch ME
        fprintf('"%s" branch does not exist\n',defBranch);
        confirm = false;
        %fprintf(2,'ERROR MESSAGE:\n\t%s\n',ME.message);
    end
end

%% Check for successful download
alternativeInstallMsg = [...
    sprintf('Manually download the %s Toolbox using the following link:\n',toolboxName),...
    newline,...
    sprintf('%s\n',url),...
    newline,...
    sprintf('Once the file is downloaded:\n'),...
    sprintf('\t(1) Unzip your download of the "%sToolbox"\n',toolboxName),...
    sprintf('\t(2) Change your "working directory" to the location of "install%sToolbox.m"\n',toolboxName),...
    sprintf('\t(3) Enter "install%sToolbox" (without quotes) into the command window\n',toolboxName),...
    sprintf('\t(4) Press Enter.')];

if ~confirm
    warning('InstallToolbox:FailedDownload','Failed to download updated version of %s Toolbox.',toolboxName);
    fprintf(2,'\n%s\n',alternativeInstallMsg);
    
    msgbox(alternativeInstallMsg, sprintf('Failed to download %s Toolbox',toolboxName),'warn');
    return
end

%% Find base directory
install_pos = strfind(fnames, sprintf('install%sToolbox.m',toolboxName) );
sIdx = cell2mat( install_pos );
cIdx = ~cell2mat( cellfun(@isempty,install_pos,'UniformOutput',0) );

pname_star = fnames{cIdx}(1:sIdx-1);

%% Get current directory and temporarily change path
cpath = cd;
cd(pname_star);

%% Install Toolbox
installToolbox(true);

%% Move back to current directory and remove temp file
cd(cpath);
[ok,msg] = rmdir(pname,'s');
if ~ok
    warning('Unable to remove temporary download folder. %s',msg);
end

%% Complete installation
fprintf('Installation complete.\n');

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% END TOOLBOX UPDATE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% SUPPORT UPDATE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 01Apr2021
function SupportUpdate(toolboxName)

%% Setup functions
ToolboxVer = str2func( sprintf('%sVer',toolboxName) );
installToolbox = str2func( sprintf('install%s',toolboxName) );

%% Check current version
try
    A = ToolboxVer;
catch ME
    A = [];
    fprintf('No previous version of %s detected.\n',toolboxName);
end

%% Setup temporary file directory
fprintf('Downloading the %s ...',toolboxName);
tmpFolder = sprintf('%s',toolboxName);
pname = fullfile(tempdir,tmpFolder);
if isfolder(pname)
    % Remove existing directory
    [ok,msg] = rmdir(pname,'s');
end
% Create new directory
[ok,msg] = mkdir(tempdir,tmpFolder);

%% Download and unzip toolbox (GitHub)
url = sprintf('https://github.com/kutzer/%s/archive/master.zip',toolboxName);
try
    % Original download/unzip method using "unzip"
    fnames = unzip(url,pname);
    
    fprintf('SUCCESS\n');
    confirm = true;
catch
    try
        % Alternative download method using "urlwrite"
        % - This method is flagged as not recommended in the MATLAB
        % documentation.
        % TODO - Consider an alternative to urlwrite.
        tmpFname = sprintf('%s-master.zip',toolboxName);
        %urlwrite(url,fullfile(pname,tmpFname));
        websave(fullfile(pname,tmpFname),url);
        fnames = unzip(fullfile(pname,tmpFname),pname);
        delete(fullfile(pname,tmpFname));
        
        fprintf('SUCCESS\n');
        confirm = true;
    catch
        fprintf('FAILED\n');
        confirm = false;
    end
end

%% Check for successful download
alternativeInstallMsg = [...
    sprintf('Manually download the %s  using the following link:\n',toolboxName),...
    sprintf('\n'),...
    sprintf('%s\n',url),...
    sprintf('\n'),...
    sprintf('Once the file is downloaded:\n'),...
    sprintf('\t(1) Unzip your download of the "%s"\n',toolboxName),...
    sprintf('\t(2) Change your "working directory" to the location of "install%s.m"\n',toolboxName),...
    sprintf('\t(3) Enter "install%s" (without quotes) into the command window\n',toolboxName),...
    sprintf('\t(4) Press Enter.')];
        
if ~confirm
    warning('Install:FailedDownload','Failed to download updated version of %s .',toolboxName);
    fprintf(2,'\n%s\n',alternativeInstallMsg);
    
    msgbox(alternativeInstallMsg, sprintf('Failed to download %s ',toolboxName),'warn');
    return
end

%% Find base directory
install_pos = strfind(fnames, sprintf('install%s.m',toolboxName) );
sIdx = cell2mat( install_pos );
cIdx = ~cell2mat( cellfun(@isempty,install_pos,'UniformOutput',0) );

pname_star = fnames{cIdx}(1:sIdx-1);

%% Get current directory and temporarily change path
cpath = cd;
cd(pname_star);

%% Install ScorBot 
installToolbox(true);

%% Move back to current directory and remove temp file
cd(cpath);
[ok,msg] = rmdir(pname,'s');
if ~ok
    warning('Unable to remove temporary download folder. %s',msg);
end

%% Complete installation
fprintf('Installation complete.\n');

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% END SUPPORT UPDATE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%