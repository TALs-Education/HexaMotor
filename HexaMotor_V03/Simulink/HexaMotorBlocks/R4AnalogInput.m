classdef R4AnalogInput < R4BlockBase
    % R4AnalogInput Analog input with reference and resolution for Arduino Uno R4.
    %
    %   Reads an analog pin with analogRead at 10, 12 or 14 bits and sets the
    %   ADC reference: Default (AVCC, 5 V), External (AREF pin) or the
    %   internal 1.43 V reference (IVREF, measured 1.43-1.46 V on a Minima).
    %   The core's 1.5/2.0/2.5 V modes aren't offered: they read 0 on the
    %   Minima. The stock block only offers 10 bits, and its reference
    %   setting has no effect on the Minima without the patch.
    %
    %   The reference is one setting for the whole board: all R4 Analog
    %   Input blocks in a model must use the same one (the first block's
    %   setting is kept). It also applies to stock Analog Input blocks.
    %
    %   Output: counts (uint16) or voltage (single) = counts / full scale *
    %   reference voltage.
    %
    %   Runs on hardware only (generated code). In normal simulation the
    %   output is zero.

    %#codegen
    %#ok<*EMCA>

    properties (Nontunable)
        %Pin
        Pin = 'A0';
        %Reference
        Reference = 'Default (AVCC, 5 V)';
        %Reference voltage (V)
        ReferenceVoltage = 5;
        %Resolution
        Resolution = '14 bits';
        %Output
        OutputType = 'Counts';
    end

    properties (Constant, Hidden)
        DriverName = 'MW_R4AnalogIn';
        BlockTitle = 'Analog Input';
        PinSet = matlab.system.StringSet({'A0', 'A1', 'A2', 'A3', 'A4', 'A5'});
        ReferenceSet = matlab.system.StringSet({'Default (AVCC, 5 V)', ...
            'External (AREF pin)', 'Internal 1.43 V'});
        ResolutionSet = matlab.system.StringSet({'10 bits', '12 bits', '14 bits'});
        OutputTypeSet = matlab.system.StringSet({'Counts', 'Voltage'});
    end

    methods
        function obj = R4AnalogInput(varargin)
            coder.allowpcode('plain');
            setProperties(obj, nargin, varargin{:});
        end

        function set.ReferenceVoltage(obj, val)
            validateattributes(val, {'numeric'}, ...
                {'scalar', 'real', 'finite', 'positive', '<=', 5.5}, '', 'Reference voltage');
            obj.ReferenceVoltage = val;
        end
    end

    methods (Access = protected)
        function setupImpl(obj)
            if coder.target('Rtw')
                coder.cinclude('MW_R4AnalogIn.h');
                coder.ceval('MW_R4AnalogIn_Setup', obj.pinNumber(), obj.refCode());
            end
        end

        function y = stepImpl(obj)
            raw = uint16(0);
            if coder.target('Rtw')
                raw = coder.ceval('MW_R4AnalogIn_Read', obj.pinNumber(), obj.bits());
            end
            if strcmp(obj.OutputType, 'Voltage')
                y = single(raw) * single(obj.refVolts() / (2^double(obj.bits()) - 1));
            else
                y = raw;
            end
        end

        function n = getNumInputsImpl(~)
            n = 0;
        end

        function n = getNumOutputsImpl(~)
            n = 1;
        end

        function name = getOutputNamesImpl(obj)
            if strcmp(obj.OutputType, 'Voltage')
                name = 'V';
            else
                name = 'counts';
            end
        end

        function s = getOutputSizeImpl(~)
            s = [1 1];
        end

        function t = getOutputDataTypeImpl(obj)
            if strcmp(obj.OutputType, 'Voltage')
                t = 'single';
            else
                t = 'uint16';
            end
        end

        function c = isOutputComplexImpl(~)
            c = false;
        end

        function f = isOutputFixedSizeImpl(~)
            f = true;
        end

        function flag = isInactivePropertyImpl(obj, prop)
            flag = strcmp(prop, 'ReferenceVoltage') && ...
                (~strcmp(obj.OutputType, 'Voltage') || obj.refCode() >= 2);
        end

        function s = iconDetail(obj)
            switch obj.refCode()
                case 0
                    r = 'AVCC';
                case 1
                    r = 'AREF';
                otherwise
                    r = sprintf('%.2f V', obj.refVolts());
            end
            s = sprintf('%s  %s  %s', obj.Pin, r, obj.Resolution);
        end

        function names = outputLabels(obj)
            names = {obj.getOutputNamesImpl()};
        end
    end

    methods (Access = private)
        function p = pinNumber(obj)
            % A0..A5 are pins 14..19 on the Uno R4
            p = uint8(14 + (double(obj.Pin(2)) - double('0')));
        end

        function r = refCode(obj)
            switch obj.Reference
                case 'External (AREF pin)'
                    r = uint8(1);
                case 'Internal 1.43 V'
                    r = uint8(2);
                otherwise
                    r = uint8(0);
            end
        end

        function v = refVolts(obj)
            if obj.refCode() == 2
                v = 1.43;   % RA4M1 IVREF, nominal (measured 1.43-1.46 V)
            else
                v = obj.ReferenceVoltage;
            end
        end

        function b = bits(obj)
            switch obj.Resolution
                case '10 bits'
                    b = uint8(10);
                case '12 bits'
                    b = uint8(12);
                otherwise
                    b = uint8(14);
            end
        end
    end

    methods (Static, Access = protected)
        function header = getHeaderImpl
            header = matlab.system.display.Header(mfilename('class'), ...
                'Title', 'Analog Input (Uno R4)', ...
                'Text', ['Reads an analog pin at 10, 12 or 14 bits with the selected ADC ' ...
                'reference. External uses the voltage on the AREF pin: enter it as the ' ...
                'reference voltage for the Voltage output. The reference is one setting ' ...
                'for the whole board, so all Analog Input blocks in a model must use the ' ...
                'same one.' newline newline ...
                'Output: counts (uint16, full scale 2^bits - 1) or voltage (single).'], ...
                'ShowSourceLink', false);
        end

        function groups = getPropertyGroupsImpl
            groups = matlab.system.display.Section('Title', 'Parameters', ...
                'PropertyList', {'Pin', 'Reference', 'Resolution', 'OutputType', ...
                'ReferenceVoltage', 'SampleTime'});
        end
    end

    methods (Static)
        function name = getDescriptiveName(~)
            name = 'Analog Input (Uno R4)';
        end

        function updateBuildInfo(buildInfo, context)
            R4BlockBase.addDriver(buildInfo, context, 'MW_R4AnalogIn');
        end
    end
end
