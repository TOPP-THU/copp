$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$cppRoot = (Resolve-Path (Join-Path $scriptDir "..")).Path
$repoRoot = (Resolve-Path (Join-Path (Join-Path $cppRoot "..") "..")).Path
$doxyfile = Join-Path $cppRoot "Doxyfile"
$docsHtml = Join-Path (Join-Path $cppRoot "docs") "html"
$docsIndex = Join-Path (Join-Path (Join-Path $cppRoot "docs") "html") "index.html"

if (-not (Get-Command doxygen -ErrorAction SilentlyContinue)) {
    throw "doxygen was not found in PATH. Install Doxygen first, then rerun this script."
}

if (Test-Path $docsHtml) {
    Remove-Item -LiteralPath $docsHtml -Recurse -Force
}

Push-Location $repoRoot
try {
    # Doxyfile reads PROJECT_NUMBER from this, so the docs carry the crate version.
    $versionLine = Get-Content (Join-Path $repoRoot "Cargo.toml") |
        Where-Object { $_ -match '^version = "(.+)"$' } |
        Select-Object -First 1
    if ($versionLine -match '^version = "(.+)"$') {
        $env:COPP_VERSION = $Matches[1]
    }

    & doxygen $doxyfile
    if ($LASTEXITCODE -ne 0) {
        throw "doxygen failed with exit code $LASTEXITCODE"
    }
} finally {
    Pop-Location
}

Write-Host "Generated COPP C++ API docs at $docsIndex"
