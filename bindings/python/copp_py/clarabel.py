r"""Clarabel backend options and diagnostic result types."""

from ._native import (
    ClarabelDirectSolveMethod as DirectSolveMethod,
    ClarabelLinearSolverInfo as LinearSolverInfo,
    ClarabelOptions as Options,
    ClarabelSettings as Settings,
    ClarabelSolverStatus as SolverStatus,
    Copp2SocpResult as Copp2Result,
    Copp3ClarabelResult as Copp3Result,
)

ClarabelDirectSolveMethod = DirectSolveMethod
ClarabelLinearSolverInfo = LinearSolverInfo
ClarabelOptions = Options
ClarabelSettings = Settings
ClarabelSolverStatus = SolverStatus
Copp2SocpResult = Copp2Result
Copp3ClarabelResult = Copp3Result

__all__ = [
    "ClarabelDirectSolveMethod",
    "ClarabelLinearSolverInfo",
    "ClarabelOptions",
    "ClarabelSettings",
    "ClarabelSolverStatus",
    "Copp2Result",
    "Copp2SocpResult",
    "Copp3ClarabelResult",
    "Copp3Result",
    "DirectSolveMethod",
    "LinearSolverInfo",
    "Options",
    "Settings",
    "SolverStatus",
]
