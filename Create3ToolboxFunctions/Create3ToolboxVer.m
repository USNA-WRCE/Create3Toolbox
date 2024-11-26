function varargout = Create3ToolboxVer
% CREATE3TOOLBOXVER displays the Create3 Toolbox for MATLAB version 
% information.
%   CREATE3TOOLBOXVER displays the version information to the 
%   command prompt.
%
%   A = CREATE3TOOLBOXVER returns in A the sorted struct array of  
%   version information for the Create3  Toolbox.
%     The definition of struct A is:
%             A.Name      : toolbox name
%             A.Version   : toolbox version number
%             A.Release   : toolbox release string
%             A.Date      : toolbox release date
%
%   L. DeVries & M. Kutzer, 31Oct2024, USNA

% Updates
%   06Nov2024 - Fixed install version check.
%   26Nov2024 - Include Geometry Toolbox.

A.Name = 'Create3  Toolbox';
A.Version = '1.0.2';
A.Release = '(R2024a)';
A.Date = '26-Nov-2024';
A.URLVer = 1;

msg{1} = sprintf('MATLAB %s Version: %s %s',A.Name, A.Version, A.Release);
msg{2} = sprintf('Release Date: %s',A.Date);

n = 0;
for i = 1:numel(msg)
    n = max( [n,numel(msg{i})] );
end

fprintf('%s\n',repmat('-',1,n));
for i = 1:numel(msg)
    fprintf('%s\n',msg{i});
end
fprintf('%s\n',repmat('-',1,n));

if nargout == 1
    varargout{1} = A;
end