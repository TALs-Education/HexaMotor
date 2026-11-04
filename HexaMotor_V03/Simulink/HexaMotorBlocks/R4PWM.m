classdef R4PWM < R4BlockBase
    % R4PWM High-resolution PWM with adjustable frequency for Arduino Uno R4.
    %
    %   Drives a GPT timer output with the duty set in raw timer counts, so
    %   the resolution is the full timer period (48 MHz / prescaler /
    %   frequency): 2400 counts (11.2 bits) at 20 kHz, 48000 counts
    %   (15.6 bits) at 1 kHz. The stock PWM block uses 8 bits.
    %
    %   Frequency: a fixed parameter, or an input port that changes it at
    %   run time down to the minimum frequency (the prescaler is chosen for
    %   the minimum frequency, so a lower minimum means a coarser duty
    %   resolution at high frequencies).
    %
    %   Inputs:  duty (0 to 1, or 0 to 100 %, clamped), freq (Hz, optional)
    %   Output (optional): counts - timer counts per period (duty resolution)
    %
    %   Runs on hardware only (generated code). In normal simulation the
    %   output is zero.

    %#codegen
    %#ok<*EMCA>

    properties (Nontunable)
        %Pin
        Pin = 9;
        %Duty cycle input range
        DutyRange = '0 to 1';
        %Frequency source
        FrequencySource = 'Parameter';
        %Frequency (Hz)
        Frequency = 20000;
        %Minimum frequency (Hz)
        MinFrequency = 1000;
        %Output timer counts per period
        OutputCounts (1,1) logical = false;
    end

    properties (Constant, Hidden)
        DriverName = 'MW_R4PWM';
        BlockTitle = 'PWM';
        DutyRangeSet = matlab.system.StringSet({'0 to 1', '0 to 100 (%)'});
        FrequencySourceSet = matlab.system.StringSet({'Parameter', 'Input port'});
    end

    properties (Access = private)
        Index = uint8(255);
    end

    methods
        function obj = R4PWM(varargin)
            coder.allowpcode('plain');
            setProperties(obj, nargin, varargin{:});
        end

        function set.Pin(obj, val)
            validateattributes(val, {'numeric'}, ...
                {'scalar', 'integer', 'nonnegative', '<=', 255}, '', 'Pin');
            obj.Pin = val;
        end

        function set.Frequency(obj, val)
            validateattributes(val, {'numeric'}, ...
                {'scalar', 'real', 'finite', '>=', 1, '<=', 1e6}, '', 'Frequency');
            obj.Frequency = val;
        end

        function set.MinFrequency(obj, val)
            validateattributes(val, {'numeric'}, ...
                {'scalar', 'real', 'finite', '>=', 1, '<=', 1e6}, '', 'Minimum frequency');
            obj.MinFrequency = val;
        end
    end

    methods (Access = protected)
        function validatePropertiesImpl(obj)
            if obj.useFreqInput() && obj.MinFrequency > obj.Frequency
                error('Minimum frequency must not be above the (initial) frequency.');
            end
        end

        function setupImpl(obj)
            if coder.target('Rtw')
                coder.cinclude('MW_R4PWM.h');
                idx = uint8(255);
                status = int32(0);
                if obj.useFreqInput()
                    fmin = single(obj.MinFrequency);
                else
                    fmin = single(obj.Frequency);
                end
                status = coder.ceval('MW_R4PWM_Setup', uint8(obj.Pin), ...
                    single(obj.Frequency), fmin, coder.wref(idx));
                if status == 0
                    obj.Index = idx;
                end
            end
        end

        function varargout = stepImpl(obj, duty, varargin)
            d = single(duty);
            if strcmp(obj.DutyRange, '0 to 100 (%)')
                d = d / single(100);
            end
            counts = uint32(0);
            if coder.target('Rtw')
                if obj.useFreqInput()
                    coder.ceval('MW_R4PWM_SetFrequency', obj.Index, single(varargin{1}));
                end
                coder.ceval('MW_R4PWM_SetDuty', obj.Index, d);
                if obj.OutputCounts
                    counts = coder.ceval('MW_R4PWM_PeriodCounts', obj.Index);
                end
            end
            if obj.OutputCounts
                varargout{1} = counts;
            end
        end

        function releaseImpl(~)
            if coder.target('Rtw')
                coder.ceval('MW_R4PWM_Release');
            end
        end

        function n = getNumInputsImpl(obj)
            n = 1 + double(obj.useFreqInput());
        end

        function n = getNumOutputsImpl(obj)
            n = double(obj.OutputCounts);
        end

        function [i1, i2] = getInputNamesImpl(~)
            i1 = 'duty';
            i2 = 'freq';
        end

        function name = getOutputNamesImpl(~)
            name = 'counts';
        end

        function s = getOutputSizeImpl(~)
            s = [1 1];
        end

        function t = getOutputDataTypeImpl(~)
            t = 'uint32';
        end

        function c = isOutputComplexImpl(~)
            c = false;
        end

        function f = isOutputFixedSizeImpl(~)
            f = true;
        end

        function flag = isInactivePropertyImpl(obj, prop)
            flag = strcmp(prop, 'MinFrequency') && ~obj.useFreqInput();
        end

        function s = iconDetail(obj)
            if obj.useFreqInput()
                s = sprintf('Pin %d  >= %g Hz', obj.Pin, obj.MinFrequency);
            else
                s = sprintf('Pin %d  %g Hz', obj.Pin, obj.Frequency);
            end
        end

        function names = inputLabels(obj)
            if obj.useFreqInput()
                names = {'duty', 'freq'};
            else
                names = {'duty'};
            end
        end

        function names = outputLabels(obj)
            if obj.OutputCounts
                names = {'counts'};
            else
                names = {};
            end
        end
    end

    methods (Access = private)
        function tf = useFreqInput(obj)
            tf = strcmp(obj.FrequencySource, 'Input port');
        end
    end

    methods (Static, Access = protected)
        function header = getHeaderImpl
            header = matlab.system.display.Header(mfilename('class'), ...
                'Title', 'PWM (Uno R4, high resolution)', ...
                'Text', ['PWM on a GPT timer pin with the duty in raw timer counts: ' ...
                'resolution = 48 MHz / prescaler / frequency (2400 counts at 20 kHz, ' ...
                '48000 at 1 kHz). With Frequency source = Input port the frequency can ' ...
                'change at run time, down to the minimum frequency, which sets the ' ...
                'prescaler.' newline newline ...
                'Each GPT timer channel has one counter, so only one pin per channel can ' ...
                'be used (a second pin on the same channel fails setup and stays off). ' ...
                'Uno R4 Minima channels: GPT0 D6/D7, GPT1 D2/D3/D11/D12 (both 32-bit), ' ...
                'GPT2 D4/D5, GPT3 D10/D13, GPT4 D0/D1, GPT5 A4/A5, GPT7 D8/D9 (16-bit). ' ...
                'Don''t use the same pin or channel in a stock PWM block.' newline newline ...
                'Inputs: duty (clamped to the range), freq (Hz, optional).' newline ...
                'Output (optional): counts - timer counts per period.'], ...
                'ShowSourceLink', false);
        end

        function groups = getPropertyGroupsImpl
            groups = matlab.system.display.Section('Title', 'Parameters', ...
                'PropertyList', {'Pin', 'DutyRange', 'FrequencySource', 'Frequency', ...
                'MinFrequency', 'OutputCounts', 'SampleTime'});
        end
    end

    methods (Static)
        function name = getDescriptiveName(~)
            name = 'PWM (Uno R4, high resolution)';
        end

        function updateBuildInfo(buildInfo, context)
            R4BlockBase.addDriver(buildInfo, context, 'MW_R4PWM');
        end
    end
end
