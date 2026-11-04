function createR4BlocksLibrary()
% CREATER4BLOCKSLIBRARY Builds R4MinimaBlocks.slx with the custom Uno R4 blocks.
%   Encoder Velocity, Encoder, PWM (high resolution) and Analog Input.
%   Called by applyPatch, or run it on its own: the blocks don't need the
%   patch. Safe to re-run; it rebuilds the library.

libName = 'R4MinimaBlocks';
libDir  = fileparts(mfilename('fullpath'));
libFile = fullfile(libDir, [libName '.slx']);

if bdIsLoaded(libName)
    close_system(libName, 0);
end
if isfile(libFile)
    delete(libFile);
end

blocks = {
    'Encoder Velocity', 'EncoderVelocity', [100  80 260 160]
    'Encoder',          'R4Encoder',       [320  80 480 160]
    'PWM',              'R4PWM',           [100 220 260 300]
    'Analog Input',     'R4AnalogInput',   [320 220 480 300]
    };

new_system(libName, 'Library');
for k = 1:size(blocks, 1)
    add_block('simulink/User-Defined Functions/MATLAB System', ...
        [libName '/' blocks{k, 1}], 'System', blocks{k, 2}, 'Position', blocks{k, 3});
end
set_param(libName, 'EnableLBRepository', 'on');
save_system(libName, libFile);
close_system(libName, 0);

fprintf('Created %s\n', libFile);
end
