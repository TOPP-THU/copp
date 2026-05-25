$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$cRoot = (Resolve-Path (Join-Path $scriptDir "..")).Path
$repoRoot = (Resolve-Path (Join-Path (Join-Path $cRoot "..") "..")).Path
$doxyfile = Join-Path $cRoot "Doxyfile"
$docsIndex = Join-Path (Join-Path (Join-Path $cRoot "docs") "html") "index.html"

if (-not (Get-Command doxygen -ErrorAction SilentlyContinue)) {
    throw "doxygen was not found in PATH. Install Doxygen first, then rerun this script."
}

& (Join-Path $scriptDir "generate_headers.ps1")

Push-Location $repoRoot
try {
    & doxygen $doxyfile
    if ($LASTEXITCODE -ne 0) {
        throw "doxygen failed with exit code $LASTEXITCODE"
    }
} finally {
    Pop-Location
}

Write-Host "Generated COPP C API docs at $docsIndex"
