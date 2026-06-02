classdef ClarabelOptions < copp.clarabel.Options
    %CLARABELOPTIONS Solver-local alias for shared Clarabel options.

    methods
        function obj = ClarabelOptions(varargin)
            obj@copp.clarabel.Options(varargin{:});
        end
    end
end
