classdef Options < copp.solver.topp3_socp.ClarabelOptions
    %OPTIONS TOPP3-SOCP Clarabel backend options.

    methods
        function obj = Options(varargin)
            obj@copp.solver.topp3_socp.ClarabelOptions(varargin{:});
        end
    end
end
