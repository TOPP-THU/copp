classdef Options < copp.solver.copp2_socp.ClarabelOptions
    %OPTIONS COPP2-SOCP Clarabel backend options.

    methods
        function obj = Options(varargin)
            obj@copp.solver.copp2_socp.ClarabelOptions(varargin{:});
        end
    end
end
