"""Public namespace smoke tests for the Python package facade."""

import copp_py as copp


def test_public_modules_are_exported() -> None:
    """The package facade follows the Rust-like module layout."""
    for name in (
        "clarabel",
        "constraints",
        "core",
        "interpolation",
        "objective",
        "path",
        "robot",
        "solver",
    ):
        assert hasattr(copp, name)
        assert name in copp.__all__


def test_top_level_keeps_core_convenience_aliases() -> None:
    """Only common modeling types are re-exported at package root."""
    assert copp.Path is copp.path.Path
    assert copp.Robot is copp.robot.Robot
    assert copp.Constraints is copp.constraints.Constraints
    assert copp.Profile3rd is copp.interpolation.Profile3rd
    assert copp.SplineConfig is copp.path.SplineConfig
    assert copp.CoppError is copp.core.CoppError
    assert copp.PathError is copp.core.PathError
    assert copp.ConstraintError is copp.core.ConstraintError


def test_solver_modules_are_exported() -> None:
    """Solver algorithms live under copp.solver, one module per algorithm."""
    for name in (
        "reach_set2",
        "topp2_ra",
        "copp2_socp",
        "topp3_lp",
        "topp3_socp",
        "copp3_socp",
    ):
        assert hasattr(copp.solver, name)
        assert name in copp.solver.__all__


def test_solver_module_aliases_native_types() -> None:
    """Algorithm modules expose local Problem/Options/Result names."""
    assert copp.solver.topp2_ra.Problem is copp.solver.reach_set2.Problem
    assert copp.solver.topp2_ra.Options is copp.solver.reach_set2.Options
    assert copp.solver.copp2_socp.Options is copp.clarabel.Options
    assert copp.solver.topp3_lp.Problem is copp.solver.topp3_socp.Problem
    assert copp.solver.topp3_socp.Options is copp.clarabel.Options
    assert copp.solver.copp3_socp.Options is copp.clarabel.Options


def test_old_solver_shortcuts_are_not_exported() -> None:
    """Root-level solver shortcuts were removed before API stabilization."""
    for name in (
        "topp2_ra",
        "copp2_socp",
        "topp3_lp",
        "topp3_socp",
        "copp3_socp",
        "Topp2Problem",
        "Copp2Problem",
        "Topp3Problem",
        "Copp3Problem",
    ):
        assert not hasattr(copp, name)
