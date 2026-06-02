"""Sphinx configuration for the copp-py Python documentation."""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[4]
PYTHON_SOURCE = ROOT / "bindings" / "python"
sys.path.insert(0, str(PYTHON_SOURCE))


def _read_cargo_package_version() -> str:
    """Return the workspace crate version without importing the native module."""
    cargo_toml = ROOT / "Cargo.toml"
    in_package = False
    for raw_line in cargo_toml.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if line == "[package]":
            in_package = True
            continue
        if in_package and line.startswith("["):
            break
        if in_package and line.startswith("version"):
            _, value = line.split("=", 1)
            return value.strip().strip('"')
    raise RuntimeError(f"could not find [package].version in {cargo_toml}")

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = "copp-py"
copyright = "2026, Yunan Wang, Suqin He, Shize Lin, and Chuxiong Hu"
author = "Yunan Wang, Suqin He, Shize Lin, and Chuxiong Hu"
release = f"v{_read_cargo_package_version()}"

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.autosummary",
    "sphinx.ext.mathjax",
    "sphinx.ext.napoleon",
]

mathjax_path = "https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-chtml.js"
mathjax3_config = {
    "tex": {
        "inlineMath": [["\\(", "\\)"]],
        "displayMath": [["\\[", "\\]"]],
        "processEscapes": True,
    },
    "options": {
        "skipHtmlTags": ["script", "noscript", "style", "textarea", "pre", "code"],
    },
}

autosummary_generate = True
autodoc_member_order = "bysource"
autodoc_typehints = "description"
autodoc_default_options = {
    "show-inheritance": True,
}

napoleon_google_docstring = False
napoleon_numpy_docstring = True

# Keep strict cross-reference checking useful. PyO3 exposes implementation
# classes through copp_py._native, while user-facing docs document the public
# facade modules instead; generated signatures also contain external typing
# fragments that are not meaningful local documentation targets.
nitpicky = True
nitpick_ignore_regex = [
    ("py:class", r"copp_py\._native\..*"),
    ("py:class", r"ArrayLike"),
    ("py:class", r"numpy\..*"),
    ("py:class", r"default=.*"),
    ("py:class", r"[0-9]+(?:\.[0-9]+)?"),
    ("py:class", r"callable"),
    ("py:class", r"sequence"),
    ("py:class", r"casadi\..*"),
    ("py:class", r"sympy\..*"),
]

templates_path = ["_templates"]
exclude_patterns = []



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = "alabaster"
html_title = f"{project} Python {release}"
html_static_path = ["_static"]
