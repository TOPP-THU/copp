classdef Options
    %OPTIONS Shared Clarabel backend policy and numerical settings.
    %
    % All MATLAB Clarabel-backed solvers accept this type. Solver-local
    % Options classes subclass it only for namespace convenience.

    properties
        %VERBOSITY Native COPP diagnostic verbosity.
        verbosity (1,1) string = "silent"

        %ALLOW_ALMOST_SOLVED Accept Clarabel AlmostSolved as usable.
        allow_almost_solved (1,1) logical = true

        %ALLOW_MAX_ITERATIONS Accept Clarabel MaxIterations as usable.
        allow_max_iterations (1,1) logical = false

        %ALLOW_MAX_TIME Accept Clarabel MaxTime as usable.
        allow_max_time (1,1) logical = false

        %ALLOW_CALLBACK_TERMINATED Accept Clarabel CallbackTerminated as usable.
        allow_callback_terminated (1,1) logical = false

        %ALLOW_INSUFFICIENT_PROGRESS Accept Clarabel InsufficientProgress as usable.
        allow_insufficient_progress (1,1) logical = false

        %CLARABEL_SETTINGS Advanced raw Clarabel numerical settings.
        clarabel_settings
    end

    methods
        function obj = Options(varargin)
            %OPTIONS Construct shared Clarabel options.
            %
            % Examples:
            %   opts = copp.clarabel.Options(max_iter=120);
            %   opts = copp.clarabel.Options(clarabel_settings=settings);
            %   opts = copp.clarabel.Options(struct("tol_gap_rel", 1e-7));
            obj = obj.apply_native_values(copp.clarabel.Options.default_native_values());

            if isempty(varargin)
                return
            end

            if isa(varargin{1}, 'copp.clarabel.Options')
                obj = varargin{1};
                varargin(1) = [];
            elseif isstruct(varargin{1})
                obj = obj.apply_struct(varargin{1});
                varargin(1) = [];
            end

            if ~isempty(varargin)
                obj = obj.apply_overrides(varargin{:});
            end
        end

        function obj = apply_overrides(obj, varargin)
            %APPLY_OVERRIDES Return a copy with name-value overrides applied.
            if mod(numel(varargin), 2) ~= 0
                error("copp:InvalidArgument", ...
                    "Clarabel options overrides must be name-value pairs.");
            end
            for k = 1:2:numel(varargin)
                obj = obj.set_named_value(varargin{k}, varargin{k + 1});
            end
        end

        function values = native_descriptor(obj)
            %NATIVE_DESCRIPTOR Return options payload in C ABI field order.
            settings_values = obj.clarabel_settings.native_descriptor();
            values = [{ ...
                char(copp.internal.normalize_verbosity(obj.verbosity)), ...
                double(obj.allow_almost_solved), ...
                double(obj.allow_max_iterations), ...
                double(obj.allow_max_time), ...
                double(obj.allow_callback_terminated), ...
                double(obj.allow_insufficient_progress)}, settings_values];
        end
    end

    methods (Access = private)
        function obj = apply_native_values(obj, values)
            obj.verbosity = verbosity_name(values{1});
            obj.allow_almost_solved = logical(values{2});
            obj.allow_max_iterations = logical(values{3});
            obj.allow_max_time = logical(values{4});
            obj.allow_callback_terminated = logical(values{5});
            obj.allow_insufficient_progress = logical(values{6});
            obj.clarabel_settings = copp.clarabel.Settings();

            function name = verbosity_name(code)
                names = ["silent", "summary", "debug", "trace"];
                idx = double(code) + 1;
                if idx >= 1 && idx <= numel(names) && idx == floor(idx)
                    name = names(idx);
                else
                    name = "silent";
                end
            end
        end

        function obj = apply_struct(obj, source)
            fields = string(fieldnames(source));
            for k = 1:numel(fields)
                obj = obj.set_named_value(fields(k), source.(fields(k)));
            end
        end

        function obj = set_named_value(obj, name, value)
            name = string(name);
            if strlength(name) == 0
                error("copp:InvalidArgument", "Clarabel options field name must not be empty.");
            end

            switch name
                case "verbosity"
                    obj.verbosity = copp.internal.normalize_verbosity(value);
                case "allow_almost_solved"
                    obj.allow_almost_solved = logical(value);
                case "allow_max_iterations"
                    obj.allow_max_iterations = logical(value);
                case "allow_max_time"
                    obj.allow_max_time = logical(value);
                case "allow_callback_terminated"
                    obj.allow_callback_terminated = logical(value);
                case "allow_insufficient_progress"
                    obj.allow_insufficient_progress = logical(value);
                case "clarabel_settings"
                    if isa(value, 'copp.clarabel.Settings')
                        obj.clarabel_settings = value;
                    elseif isstruct(value)
                        obj.clarabel_settings = copp.clarabel.Settings(value);
                    else
                        error("copp:InvalidArgument", ...
                            "clarabel_settings must be a copp.clarabel.Settings object or struct.");
                    end
                otherwise
                    obj.clarabel_settings = obj.clarabel_settings.apply_overrides(name, value);
            end
        end
    end

    methods (Static, Hidden)
        function values = default_native_values()
            %DEFAULT_NATIVE_VALUES Return raw default payload from the MEX layer.
            values = cell(1, 45);
            [values{:}] = copp.internal.copp_mex('clarabel_default_options');
        end
    end
end
