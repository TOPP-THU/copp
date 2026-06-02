function tf = is_copp_error(err)
%IS_COPP_ERROR Return true if ERR is a COPP MATLAB error.
tf = copp.diag.CoppError.is_copp_error(err);
end
