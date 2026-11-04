classdef EncoderVelocity < matlab.System & ...
        matlabshared.svd.BlockSampleTime & ...
        coder.ExternalDependency
    % EncoderVelocity Quadrature encoder velocity from pulse timing.
    %
    %   Timestamps encoder edges in an interrupt (micros()) and outputs the
    %   velocity as the inverse of the moving average of the last N
    %   edge-to-edge intervals. Better resolution than counting pulses per
    %   sample at low and medium speeds.
    %
    %   The average restarts on a direction change or when no edge arrives
    %   within the stall timeout (velocity is 0 while stalled).
    %
    %   Low speed (edges slower than the sample rate):
    %     Maximum window time > 0 - time-bounded window: averages up to N
    %       intervals but spanning at most this time and at least one full
    %       quadrature cycle, rounded to whole cycles (long average at high
    %       speed, low lag at low speed). 0 = fixed window of N intervals.
    %     Decay tolerance - with decay on, the rate is bounded by the time
    %       since the last edge only once it exceeds tolerance x the average
    %       interval (1.5 ignores uneven edge spacing; 1 = original).
    %     Count anchor time constant > 0 - corrects the output so that its
    %       integral follows the edge count (removes the low-speed bias of
    %       edge timing with stick-slip motion). 0 = off.
    %     Output filter time constant > 0 - first-order low-pass on the
    %       velocity output (a few ms smooths the edge-timing ripple). 0 = off.
    %
    %   Outputs (order: velocity first, or ticks first with "Ticks output on top"):
    %     Motor_<units> - single, velocity of the encoder shaft in the selected
    %                     units (rad/s, RPM, Rev/s or Edges/s). The encoder is on
    %                     the motor shaft: divide by the gear ratio for the output.
    %     ticks         - int32, accumulated quadrature edge count (optional);
    %                     4 per encoder line in 4x mode (48 per turn, 12 lines)
    %
    %   Runs on hardware only (generated code). In normal simulation the
    %   outputs are zero.

    %#codegen
    %#ok<*EMCA>

    properties (Nontunable)
        %Encoder pin A
        PinA = 2;
        %Encoder pin B
        PinB = 3;
        %Edges used for timing
        EdgeMode = 'A and B both edges (4x)';
        %Moving average window (intervals)
        WindowLength = 8;
        %Encoder pulses per revolution (lines)
        PulsesPerRevolution = 1;
        %Output units
        Units = 'Edges/s';
        %Stall timeout (s)
        StallTimeout = 0.1;
        %Decay velocity while waiting for the next edge
        DecayWhenSlowing (1,1) logical = false;
        %Decay tolerance (x average interval)
        DecayTolerance = 1;
        %Maximum window time (s), 0 = fixed window of N intervals
        MaxWindowTime = 0;
        %Count anchor time constant (s), 0 = off
        CountAnchorTime = 0;
        %Output filter time constant (s), 0 = off
        OutputFilterTime = 0;
        %Output edge count (ticks)
        OutputCount (1,1) logical = true;
        %Ticks output on top
        TicksFirst (1,1) logical = false;
        %Sample time
        SampleTime = 0.01;
    end

    properties (Constant, Hidden)
        EdgeModeSet = matlab.system.StringSet({ ...
            'A rising (1x)', 'A both edges (2x)', 'A and B both edges (4x)'});
        UnitsSet = matlab.system.StringSet({'Edges/s', 'Rev/s', 'RPM', 'rad/s'});
        MaxWindow = 48;
    end

    properties (Access = private)
        Index = uint8(255);
    end

    methods
        function obj = EncoderVelocity(varargin)
            coder.allowpcode('plain');
            setProperties(obj, nargin, varargin{:});
        end

        function set.PinA(obj, val)
            validateattributes(val, {'numeric'}, ...
                {'scalar', 'integer', 'nonnegative', '<=', 255}, '', 'Encoder pin A');
            obj.PinA = val;
        end

        function set.PinB(obj, val)
            validateattributes(val, {'numeric'}, ...
                {'scalar', 'integer', 'nonnegative', '<=', 255}, '', 'Encoder pin B');
            obj.PinB = val;
        end

        function set.WindowLength(obj, val)
            validateattributes(val, {'numeric'}, ...
                {'scalar', 'integer', '>=', 1, '<=', 48}, '', 'Moving average window');
            obj.WindowLength = val;
        end

        function set.PulsesPerRevolution(obj, val)
            validateattributes(val, {'numeric'}, ...
                {'scalar', 'real', 'finite', 'positive'}, '', 'Pulses per revolution');
            obj.PulsesPerRevolution = val;
        end

        function set.DecayTolerance(obj, val)
            validateattributes(val, {'numeric'}, ...
                {'scalar', 'real', 'finite', '>=', 1, '<=', 10}, '', 'Decay tolerance');
            obj.DecayTolerance = val;
        end

        function set.MaxWindowTime(obj, val)
            validateattributes(val, {'numeric'}, ...
                {'scalar', 'real', 'finite', '>=', 0, '<=', 10}, '', 'Maximum window time');
            obj.MaxWindowTime = val;
        end

        function set.CountAnchorTime(obj, val)
            validateattributes(val, {'numeric'}, ...
                {'scalar', 'real', 'finite', '>=', 0, '<=', 100}, '', 'Count anchor time constant');
            obj.CountAnchorTime = val;
        end

        function set.OutputFilterTime(obj, val)
            validateattributes(val, {'numeric'}, ...
                {'scalar', 'real', 'finite', '>=', 0, '<=', 10}, '', 'Output filter time constant');
            obj.OutputFilterTime = val;
        end

        function set.StallTimeout(obj, val)
            validateattributes(val, {'numeric'}, ...
                {'scalar', 'real', '>=', 1e-4, '<=', 10}, '', 'Stall timeout');
            obj.StallTimeout = val;
        end

        function set.SampleTime(obj, newTime)
            coder.extrinsic('error');
            coder.extrinsic('message');
            newTime = matlabshared.svd.internal.validateSampleTime(newTime);
            obj.SampleTime = newTime;
        end
    end

    methods (Access = protected)
        function validatePropertiesImpl(obj)
            if obj.PinA == obj.PinB
                error('Encoder pin A and pin B must be different.');
            end
        end

        function setupImpl(obj)
            if coder.target('Rtw')
                coder.cinclude('MW_EncoderVelocity.h');
                idx = uint8(255);
                status = int32(0);
                status = coder.ceval('MW_EncVel_Setup', uint8(obj.PinA), uint8(obj.PinB), ...
                    uint8(obj.edgeMultiplier()), uint8(obj.WindowLength), ...
                    uint32(round(obj.StallTimeout * 1e6)), uint8(obj.DecayWhenSlowing), ...
                    single(obj.DecayTolerance), uint32(round(obj.MaxWindowTime * 1e6)), ...
                    single(obj.CountAnchorTime), single(obj.OutputFilterTime), coder.wref(idx));
                if status == 0
                    obj.Index = idx;
                end
            end
        end

        function varargout = stepImpl(obj)
            edgeRate = single(0);
            count = int32(0);
            if coder.target('Rtw')
                edgeRate = coder.ceval('MW_EncVel_Read', obj.Index, coder.wref(count));
            end
            vel = edgeRate * single(obj.unitScale());
            if obj.OutputCount && obj.TicksFirst
                varargout{1} = count;
                varargout{2} = vel;
            else
                varargout{1} = vel;
                if obj.OutputCount
                    varargout{2} = count;
                end
            end
        end

        function releaseImpl(~)
            if coder.target('Rtw')
                coder.ceval('MW_EncVel_Release');
            end
        end

        function n = getNumInputsImpl(~)
            n = 0;
        end

        function n = getNumOutputsImpl(obj)
            n = 1 + double(obj.OutputCount);
        end

        function [o1, o2] = getOutputNamesImpl(obj)
            velName = obj.velocityName();
            if obj.OutputCount && obj.TicksFirst
                o1 = 'ticks';
                o2 = velName;
            else
                o1 = velName;
                o2 = 'ticks';
            end
        end

        function [s1, s2] = getOutputSizeImpl(~)
            s1 = [1 1];
            s2 = [1 1];
        end

        function [t1, t2] = getOutputDataTypeImpl(obj)
            if obj.OutputCount && obj.TicksFirst
                t1 = 'int32';
                t2 = 'single';
            else
                t1 = 'single';
                t2 = 'int32';
            end
        end

        function [c1, c2] = isOutputComplexImpl(~)
            c1 = false;
            c2 = false;
        end

        function [f1, f2] = isOutputFixedSizeImpl(~)
            f1 = true;
            f2 = true;
        end

        function flag = isInactivePropertyImpl(obj, prop)
            flag = (strcmp(prop, 'PulsesPerRevolution') && strcmp(obj.Units, 'Edges/s')) || ...
                (strcmp(prop, 'DecayTolerance') && ~obj.DecayWhenSlowing) || ...
                (strcmp(prop, 'TicksFirst') && ~obj.OutputCount);
        end

        function sts = getSampleTimeImpl(obj)
            sts = getSampleTimeImpl@matlabshared.svd.BlockSampleTime(obj);
        end

        function maskDisplayCmds = getMaskDisplayImpl(obj)
            pins = sprintf('A:%d  B:%d', obj.PinA, obj.PinB);
            maskDisplayCmds = [ ...
                ['color(''white'');', newline] ...
                ['plot([100,100,100,100]*1,[100,100,100,100]*1);', newline] ...
                ['plot([100,100,100,100]*0,[100,100,100,100]*0);', newline] ...
                ['color(''blue'');', newline] ...
                ['text(2, 92, ''\fontsize{9} ARDUINO'',''texmode'',''on'',''horizontalAlignment'', ''left'');', newline] ...
                ['color(''black'');', newline] ...
                ['text(50, 55, ''\fontsize{11}\bfHexaMotor Encoder'',''texmode'',''on'',''horizontalAlignment'', ''center'');', newline] ...
                ['text(50, 14, ''\fontsize{9}' pins ''',''texmode'',''on'',''horizontalAlignment'', ''center'');', newline] ...
                ];
            % output port labels (the custom icon hides the default ones)
            [o1, o2] = obj.getOutputNamesImpl();
            names = {o1, o2};
            for k = 1:obj.getNumOutputsImpl()
                maskDisplayCmds = [maskDisplayCmds, ...
                    sprintf('port_label(''output'', %d, ''%s'');', k, names{k}), newline]; %#ok<AGROW>
            end
        end
    end

    methods (Access = private)
        function name = velocityName(obj)
            % Port label of the velocity output, from the selected units
            switch obj.Units
                case 'rad/s'
                    name = 'Motor_Rad/s';
                case 'RPM'
                    name = 'Motor_RPM';
                case 'Rev/s'
                    name = 'Motor_Rev/s';
                otherwise
                    name = 'Edges/s';
            end
        end

        function m = edgeMultiplier(obj)
            switch obj.EdgeMode
                case 'A rising (1x)'
                    m = 1;
                case 'A both edges (2x)'
                    m = 2;
                otherwise
                    m = 4;
            end
        end

        function k = unitScale(obj)
            % Converts edges/s to the selected output units
            edgesPerRev = obj.PulsesPerRevolution * obj.edgeMultiplier();
            switch obj.Units
                case 'Rev/s'
                    k = 1 / edgesPerRev;
                case 'RPM'
                    k = 60 / edgesPerRev;
                case 'rad/s'
                    k = 2 * pi / edgesPerRev;
                otherwise
                    k = 1;
            end
        end
    end

    methods (Static, Access = protected)
        function header = getHeaderImpl
            header = matlab.system.display.Header(mfilename('class'), ...
                'Title', 'Encoder Velocity (pulse timing)', ...
                'Text', ['Measures quadrature encoder velocity from the time between ' ...
                'edges, averaged over the last N intervals. Velocity is 0 after the ' ...
                'stall timeout. Pins in the interrupt must be interrupt-capable; for ' ...
                '4x on Uno R4, A and B must be on different IRQ channels (e.g. 2 and 3).' newline newline ...
                'Outputs:' newline ...
                '  Motor_<units> (single): velocity of the encoder shaft in the selected units ' ...
                '(rad/s, RPM, Rev/s or Edges/s). With the encoder on the motor shaft, divide by ' ...
                'the gear ratio for the output shaft (HexaMotor: 20.4).' newline ...
                '  ticks (int32): accumulated quadrature edge count, 4 per line in 4x mode ' ...
                '(HexaMotor: 48 per motor turn, 979.2 per output turn).'], ...
                'ShowSourceLink', false);
        end

        function groups = getPropertyGroupsImpl
            pins = matlab.system.display.Section('Title', 'Encoder', ...
                'PropertyList', {'PinA', 'PinB', 'EdgeMode', 'PulsesPerRevolution'});
            meas = matlab.system.display.Section('Title', 'Measurement', ...
                'PropertyList', {'WindowLength', 'MaxWindowTime', 'StallTimeout', ...
                'DecayWhenSlowing', 'DecayTolerance', 'CountAnchorTime', 'OutputFilterTime', ...
                'Units', 'OutputCount', 'TicksFirst', 'SampleTime'});
            groups = [pins, meas];
        end

        function simMode = getSimulateUsingImpl
            simMode = 'Interpreted execution';
        end

        function flag = showSimulateUsingImpl
            flag = false;
        end
    end

    methods (Static)
        function name = getDescriptiveName(~)
            name = 'Encoder Velocity (pulse timing)';
        end

        function tf = isSupportedContext(~)
            tf = true;
        end

        function updateBuildInfo(buildInfo, context)
            if context.isCodeGenTarget('rtw')
                rootDir = fileparts(mfilename('fullpath'));
                addIncludePaths(buildInfo, fullfile(rootDir, 'include'));
                addIncludeFiles(buildInfo, 'MW_EncoderVelocity.h');
                systemTargetFile = get_param(buildInfo.ModelName, 'SystemTargetFile');
                if isequal(systemTargetFile, 'ert.tlc')
                    addSourcePaths(buildInfo, fullfile(rootDir, 'src'));
                    addSourceFiles(buildInfo, 'MW_EncoderVelocity.cpp', ...
                        fullfile(rootDir, 'src'), 'BlockModules');
                end
            end
        end
    end
end
