function install_HexaMotor_V03(option)
% INSTALL_HEXAMOTOR_V03 Sets up MATLAB for the HexaMotor V03 lab models.
%
%   install_HexaMotor_V03              adds the HexaMotor folder (models) and
%                                      its HexaMotorBlocks subfolder (custom
%                                      Uno R4 blocks) to the MATLAB path,
%                                      saves the path and checks the setup.
%   install_HexaMotor_V03('uninstall') removes both folders from the path.
%
%   The custom blocks (Encoder Velocity, Encoder, PWM, Analog Input) are
%   MATLAB System objects with C++ drivers in HexaMotorBlocks. A Simulink
%   model stores only the block's class name and parameters, not the block
%   code, so the folder must be on the MATLAB path on every machine. The V03
%   models also add HexaMotorBlocks to the path when they open.

if nargin < 1
    option = '';
end
root = fileparts(mfilename('fullpath'));
blocksDir = fullfile(root, 'HexaMotorBlocks');
dirs = {root, blocksDir};

if strcmpi(option, 'uninstall')
    for k = 1:numel(dirs)
        if contains([pathsep path pathsep], [pathsep dirs{k} pathsep], 'IgnoreCase', true)
            rmpath(dirs{k});
        end
    end
    saveThePath(dirs, false);
    fprintf('Removed %s and %s from the MATLAB path.\n', root, blocksDir);
    return;
elseif ~isempty(option)
    error('Unknown option ''%s''. Use install_HexaMotor_V03 or install_HexaMotor_V03(''uninstall'').', option);
end

fprintf('HexaMotor V03 setup\n');
fprintf('  MATLAB %s (R%s)\n', version, version('-release'));
fprintf('  Folder %s\n\n', root);
problems = 0;

%% Required products
if ~license('test', 'Simulink') || isempty(ver('simulink'))
    fprintf('MISSING: Simulink.\n');
    problems = problems + 1;
end
if isempty(ver('sps')) && isempty(ver('physmod'))
    fprintf('MISSING: Simscape Electrical (Filtered Derivative and Lead-Lag blocks in the models).\n');
    problems = problems + 1;
end
spkgs = {};
try
    installed = matlabshared.supportpkg.getInstalled;
    spkgs = {installed.Name};
catch
end
needed = {'Simulink Support Package for Arduino Hardware'};
for k = 1:numel(needed)
    if ~any(strcmp(spkgs, needed{k}))
        fprintf('MISSING: %s (Home > Add-Ons > Get Hardware Support Packages).\n', needed{k});
        problems = problems + 1;
    end
end

%% Path: remove old copies of the blocks, then add the folders
if ~isfolder(blocksDir)
    error('HexaMotorBlocks folder not found next to install_HexaMotor_V03 (%s).', blocksDir);
end
oldDirs = {};
classes = {'EncoderVelocity', 'R4BlockBase', 'R4Encoder', 'R4PWM', 'R4AnalogInput'};
for c = classes
    hits = which('-all', c{1});
    if ischar(hits), hits = {hits}; end
    for h = 1:numel(hits)
        d = fileparts(hits{h});
        if ~strcmpi(d, blocksDir) && ~any(strcmpi(oldDirs, d))
            oldDirs{end+1} = d; %#ok<AGROW>
        end
    end
end
p = [pathsep path pathsep];
for k = 1:numel(oldDirs)
    if ~strcmpi(oldDirs{k}, root) && contains(p, [pathsep oldDirs{k} pathsep], 'IgnoreCase', true)
        rmpath(oldDirs{k});
        fprintf('Removed an old copy of the blocks from the path: %s\n', oldDirs{k});
    end
end
addpath(root, blocksDir);
saveThePath(dirs, true);

%% Blocks
for c = classes
    if ~exist(c{1}, 'class')
        fprintf('PROBLEM: block class %s not found.\n', c{1});
        problems = problems + 1;
    else
        d = fileparts(which(c{1}));
        if ~strcmpi(d, blocksDir)
            fprintf('PROBLEM: %s is found in %s instead of HexaMotorBlocks.\n', c{1}, d);
            problems = problems + 1;
        end
    end
end
if ~isfile(fullfile(blocksDir, 'R4MinimaBlocks.slx'))
    try
        createR4BlocksLibrary();
    catch err
        fprintf('PROBLEM: could not create the R4MinimaBlocks library: %s\n', err.message);
        problems = problems + 1;
    end
end
try
    lb = LibraryBrowser.LibraryBrowser2;
    refresh(lb);
catch
    % Library Browser not open or not available; it reads slblocks.m when it opens
end

%% Models
models = dir(fullfile(root, 'HexaMotor_*_V03.slx'));
fprintf('\nLab models:\n');
for k = 1:numel(models)
    fprintf('  %s\n', models(k).name);
end

fprintf('\n');
if problems == 0
    fprintf('Setup complete. Open a model and use Hardware > Monitor & Tune.\n');
else
    fprintf('Setup finished with %d problem(s); see the messages above.\n', problems);
end
end


function saveThePath(dirs, adding)
% Saves the path. Without write access to pathdef.m (MATLAB not started as
% administrator), puts an addpath line in startup.m in the user folder.
if savepath == 0
    if adding
        fprintf('Added %s to the MATLAB path (saved).\n', strjoin(dirs, ' and '));
    end
    return;
end
up = userpath;
if isempty(up)
    fprintf(['WARNING: Could not save the path. Run install_HexaMotor_V03 again in each ' ...
        'session, or start MATLAB as administrator and run it once.\n']);
    return;
end
startupFile = fullfile(up, 'startup.m');
entry = sprintf('addpath(''%s''); %% HexaMotor V03', strjoin(dirs, ''', '''));
txt = '';
if isfile(startupFile)
    txt = fileread(startupFile);
end
lines = strsplit(txt, {'\r\n', '\n'}, 'CollapseDelimiters', false);
lines = lines(~contains(lines, '% HexaMotor V03'));
if adding
    lines{end+1} = entry;
end
lines = lines(~cellfun(@isempty, lines));
fid = fopen(startupFile, 'w');
if fid < 0
    fprintf('WARNING: Could not save the path or write %s.\n', startupFile);
    return;
end
fprintf(fid, '%s\n', lines{:});
fclose(fid);
if adding
    fprintf('Added %s to the MATLAB path (in %s, since pathdef.m is read-only).\n', ...
        strjoin(dirs, ' and '), startupFile);
end
end
