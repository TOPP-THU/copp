classdef DirectSolveMethod
    %DIRECTSOLVEMETHOD Direct KKT solver selection forwarded to Clarabel.

    enumeration
        auto
        qdldl
        faer
        mkl
        panua
    end
end
