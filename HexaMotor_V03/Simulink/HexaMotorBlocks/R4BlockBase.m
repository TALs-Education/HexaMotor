classdef (Abstract) R4BlockBase < matlab.System & coder.ExternalDependency
    % R4BlockBase Shared code of the Arduino Uno R4 custom blocks.
    %
    %   Uses only documented interfaces (matlab.System,
    %   coder.ExternalDependency, createSampleTime), so the blocks don't
    %   depend on MathWorks support package internals and carry over to new
    %   releases. Subclasses list their C++ driver in DriverName.

    %#codegen

    properties (Nontunable)
        %Sample time (s), -1 = inherited
        SampleTime = 0.01;
    end

    properties (Abstract, Constant, Hidden)
        DriverName      % C++ driver file name without extension (MW_R4...)
        BlockTitle      % title in the block icon
    end

    methods
        function set.SampleTime(obj, val)
            validateattributes(val, {'numeric'}, {'scalar', 'real', 'finite'}, '', 'Sample time');
            if val ~= -1 && val <= 0
                error('Sample time must be positive, or -1 for inherited.');
            end
            obj.SampleTime = val;
        end
    end

    methods (Access = protected)
        function sts = getSampleTimeImpl(obj)
            if obj.SampleTime == -1
                sts = createSampleTime(obj, 'Type', 'Inherited');
            else
                sts = createSampleTime(obj, 'Type', 'Discrete', 'SampleTime', obj.SampleTime);
            end
        end

        function maskDisplayCmds = getMaskDisplayImpl(obj)
            maskDisplayCmds = [ ...
                ['color(''white'');', newline] ...
                ['plot([100,100,100,100]*1,[100,100,100,100]*1);', newline] ...
                ['plot([100,100,100,100]*0,[100,100,100,100]*0);', newline] ...
                ['color(''blue'');', newline] ...
                ['text(2, 92, ''\fontsize{9} ARDUINO R4'',''texmode'',''on'',''horizontalAlignment'', ''left'');', newline] ...
                ['color(''black'');', newline] ...
                ['text(50, 55, ''\fontsize{11}\bf' obj.BlockTitle ''',''texmode'',''on'',''horizontalAlignment'', ''center'');', newline] ...
                ['text(50, 14, ''\fontsize{9}' obj.iconDetail() ''',''texmode'',''on'',''horizontalAlignment'', ''center'');', newline] ...
                ];
            % port labels (the custom icon hides the default ones)
            inNames = obj.inputLabels();
            for k = 1:numel(inNames)
                maskDisplayCmds = [maskDisplayCmds, ...
                    sprintf('port_label(''input'', %d, ''%s'');', k, inNames{k}), newline]; %#ok<AGROW>
            end
            outNames = obj.outputLabels();
            for k = 1:numel(outNames)
                maskDisplayCmds = [maskDisplayCmds, ...
                    sprintf('port_label(''output'', %d, ''%s'');', k, outNames{k}), newline]; %#ok<AGROW>
            end
        end

        % Icon text and port labels; subclasses override as needed
        function s = iconDetail(~)
            s = '';
        end

        function names = inputLabels(~)
            names = {};
        end

        function names = outputLabels(~)
            names = {};
        end
    end

    methods (Static, Access = protected)
        function simMode = getSimulateUsingImpl
            simMode = 'Interpreted execution';
        end

        function flag = showSimulateUsingImpl
            flag = false;
        end
    end

    methods (Static)
        function tf = isSupportedContext(~)
            tf = true;
        end

        function addDriver(buildInfo, context, driverName)
            % Adds the driver header and source of a block to the build.
            if context.isCodeGenTarget('rtw')
                rootDir = fileparts(mfilename('fullpath'));
                addIncludePaths(buildInfo, fullfile(rootDir, 'include'));
                addIncludeFiles(buildInfo, [driverName '.h']);
                systemTargetFile = get_param(buildInfo.ModelName, 'SystemTargetFile');
                if isequal(systemTargetFile, 'ert.tlc')
                    addSourcePaths(buildInfo, fullfile(rootDir, 'src'));
                    addSourceFiles(buildInfo, [driverName '.cpp'], ...
                        fullfile(rootDir, 'src'), 'BlockModules');
                end
            end
        end
    end
end
