classdef R4Encoder < R4BlockBase
    % R4Encoder Quadrature encoder count for Arduino Uno R4 (Renesas RA4M1).
    %
    %   Counts every edge of A and B (4 counts per encoder line) with
    %   interrupts. Standalone replacement for the stock Encoder block on
    %   the Uno R4, which doesn't build for the RA4M1 without the patch.
    %
    %   A and B must be interrupt-capable pins on different IRQ channels
    %   (for example 2 and 3). If setup fails, the count stays 0.
    %
    %   Input (optional): reset - the count is set to 0 while it is nonzero.
    %   Output: count (int32).
    %
    %   Runs on hardware only (generated code). In normal simulation the
    %   output is zero.

    %#codegen
    %#ok<*EMCA>

    properties (Nontunable)
        %Encoder pin A
        PinA = 2;
        %Encoder pin B
        PinB = 3;
        %Enable internal pull-ups
        PullUp (1,1) logical = false;
        %Reverse count direction
        Reverse (1,1) logical = false;
        %Reset input
        ResetInput (1,1) logical = false;
    end

    properties (Constant, Hidden)
        DriverName = 'MW_R4Encoder';
        BlockTitle = 'Encoder';
    end

    properties (Access = private)
        Index = uint8(255);
    end

    methods
        function obj = R4Encoder(varargin)
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
    end

    methods (Access = protected)
        function validatePropertiesImpl(obj)
            if obj.PinA == obj.PinB
                error('Encoder pin A and pin B must be different.');
            end
        end

        function setupImpl(obj)
            if coder.target('Rtw')
                coder.cinclude('MW_R4Encoder.h');
                idx = uint8(255);
                status = int32(0);
                status = coder.ceval('MW_R4Enc_Setup', uint8(obj.PinA), uint8(obj.PinB), ...
                    uint8(obj.PullUp), coder.wref(idx));
                if status == 0
                    obj.Index = idx;
                end
            end
        end

        function count = stepImpl(obj, varargin)
            count = int32(0);
            if coder.target('Rtw')
                if obj.ResetInput && varargin{1} ~= 0
                    coder.ceval('MW_R4Enc_Reset', obj.Index);
                end
                count = coder.ceval('MW_R4Enc_Read', obj.Index);
            end
            if obj.Reverse
                count = -count;
            end
        end

        function releaseImpl(~)
            if coder.target('Rtw')
                coder.ceval('MW_R4Enc_Release');
            end
        end

        function n = getNumInputsImpl(obj)
            n = double(obj.ResetInput);
        end

        function n = getNumOutputsImpl(~)
            n = 1;
        end

        function name = getInputNamesImpl(~)
            name = 'reset';
        end

        function name = getOutputNamesImpl(~)
            name = 'count';
        end

        function s = getOutputSizeImpl(~)
            s = [1 1];
        end

        function t = getOutputDataTypeImpl(~)
            t = 'int32';
        end

        function c = isOutputComplexImpl(~)
            c = false;
        end

        function f = isOutputFixedSizeImpl(~)
            f = true;
        end

        function s = iconDetail(obj)
            s = sprintf('A:%d  B:%d', obj.PinA, obj.PinB);
        end

        function names = inputLabels(obj)
            if obj.ResetInput
                names = {'reset'};
            else
                names = {};
            end
        end

        function names = outputLabels(~)
            names = {'count'};
        end
    end

    methods (Static, Access = protected)
        function header = getHeaderImpl
            header = matlab.system.display.Header(mfilename('class'), ...
                'Title', 'Encoder (Uno R4)', ...
                'Text', ['Counts quadrature encoder edges with interrupts: 4 counts per ' ...
                'encoder line. A and B must be interrupt-capable pins on different IRQ ' ...
                'channels (for example 2 and 3). Up to 4 encoders. Don''t use the same pins ' ...
                'in another encoder block.' newline newline ...
                'Input (optional): reset - the count is set to 0 while it is nonzero.' newline ...
                'Output: count (int32).'], ...
                'ShowSourceLink', false);
        end

        function groups = getPropertyGroupsImpl
            groups = matlab.system.display.Section('Title', 'Parameters', ...
                'PropertyList', {'PinA', 'PinB', 'PullUp', 'Reverse', 'ResetInput', 'SampleTime'});
        end
    end

    methods (Static)
        function name = getDescriptiveName(~)
            name = 'Encoder (Uno R4)';
        end

        function updateBuildInfo(buildInfo, context)
            R4BlockBase.addDriver(buildInfo, context, 'MW_R4Encoder');
        end
    end
end
