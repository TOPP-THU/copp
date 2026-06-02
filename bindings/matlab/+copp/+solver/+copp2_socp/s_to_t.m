function varargout = s_to_t(varargin)
%S_TO_T Solver-local alias for copp.interpolation.s_to_t_topp2.
if nargout == 0
    copp.interpolation.s_to_t_topp2(varargin{:});
else
    [varargout{1:nargout}] = copp.interpolation.s_to_t_topp2(varargin{:});
end
end
