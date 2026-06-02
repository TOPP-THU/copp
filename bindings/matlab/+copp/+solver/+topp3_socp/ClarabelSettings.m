classdef ClarabelSettings < copp.clarabel.Settings
    %CLARABELSETTINGS Solver-local alias for shared Clarabel settings.

    methods
        function obj = ClarabelSettings(varargin)
            obj@copp.clarabel.Settings(varargin{:});
        end
    end
end
