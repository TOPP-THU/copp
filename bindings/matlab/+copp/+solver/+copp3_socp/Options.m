classdef Options < copp.solver.copp3_socp.ClarabelOptions
    %OPTIONS COPP3-SOCP Clarabel backend options.

    methods
        function obj = Options(varargin)
            obj@copp.solver.copp3_socp.ClarabelOptions(varargin{:});
        end
    end
end
