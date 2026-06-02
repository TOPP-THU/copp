classdef Options < copp.solver.topp3_lp.ClarabelOptions
    %OPTIONS TOPP3-LP Clarabel backend options.

    methods
        function obj = Options(varargin)
            obj@copp.solver.topp3_lp.ClarabelOptions(varargin{:});
        end
    end
end
