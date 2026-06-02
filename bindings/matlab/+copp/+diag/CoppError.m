classdef CoppError
    %COPPERROR Facade for MATLAB exceptions raised by COPP MEX calls.
    %
    % MATLAB catch blocks receive MException objects. CoppError wraps those
    % exceptions so user code can inspect COPP-specific category/detail fields.

    properties (SetAccess = private)
        identifier (1,1) string
        message (1,1) string
        category (1,1) string
        native_status
        detail (1,1) string
    end

    methods
        function obj = CoppError(identifier, message, category, native_status, detail)
            arguments
                identifier (1,1) string = "copp:NativeError"
                message (1,1) string = ""
                category (1,1) string = "NativeError"
                native_status = []
                detail (1,1) string = ""
            end
            obj.identifier = identifier;
            obj.message = message;
            obj.category = category;
            obj.native_status = native_status;
            obj.detail = detail;
        end

        function err = to_exception(obj)
            %TO_EXCEPTION Convert this facade back to a MATLAB MException.
            err = MException(obj.identifier, "%s", obj.message);
        end

        function throw_as_caller(obj)
            %THROW_AS_CALLER Throw this error as a MATLAB exception.
            throwAsCaller(obj.to_exception());
        end
    end

    methods (Static)
        function tf = is_copp_error(err)
            %IS_COPP_ERROR Return true for COPP MException/CoppError values.
            if isa(err, 'copp.diag.CoppError')
                tf = true;
                return
            end
            tf = isa(err, 'MException') && startsWith(string(err.identifier), "copp:");
        end

        function obj = from_exception(err)
            %FROM_EXCEPTION Build a CoppError facade from a caught MException.
            if isa(err, 'copp.diag.CoppError')
                obj = err;
                return
            end
            if ~isa(err, 'MException')
                error("copp:InvalidArgument", ...
                    "from_exception expects an MException or copp.diag.CoppError.");
            end

            identifier = string(err.identifier);
            if startsWith(identifier, "copp:")
                parts = split(identifier, ":");
                category = parts(end);
            else
                category = "Unknown";
            end

            message = string(err.message);
            detail = "";
            tokens = split(message, ": ");
            if numel(tokens) >= 2
                detail = strjoin(tokens(2:end), ": ");
            end

            obj = copp.diag.CoppError(identifier, message, category, [], detail);
        end
    end
end
