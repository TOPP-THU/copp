function varargout = s_to_t(varargin)
%S_TO_T Solver-local alias for copp.interpolation.s_to_t_topp3.
if nargout == 0
    copp.interpolation.s_to_t_topp3(varargin{:});
else
    [varargout{1:nargout}] = copp.interpolation.s_to_t_topp3(varargin{:});
end
end
