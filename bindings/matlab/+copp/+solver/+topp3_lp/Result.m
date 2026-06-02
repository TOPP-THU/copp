classdef Result < copp.solver.copp3_socp.Result
    %RESULT TOPP3-LP expert result with Clarabel diagnostics.

    methods
        function obj = Result(varargin)
            obj@copp.solver.copp3_socp.Result(varargin{:});
        end
    end
end
