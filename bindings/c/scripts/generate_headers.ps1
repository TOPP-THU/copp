$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$cRoot = (Resolve-Path (Join-Path $scriptDir "..")).Path
$repoRoot = (Resolve-Path (Join-Path (Join-Path $cRoot "..") "..")).Path
$includeRoot = Join-Path (Join-Path $cRoot "include") "copp"
$buildRoot = Join-Path $cRoot "build"
$generatedHeader = Join-Path $buildRoot "copp_cbindgen_full.h"
$configPath = Join-Path $cRoot "cbindgen.toml"
$nl = "`n"
$utf8NoBom = [System.Text.UTF8Encoding]::new($false)

function Normalize-CoppNewlines {
    param([Parameter(Mandatory = $true)][string]$Text)

    return (($Text -replace "`r`n", "`n") -replace "`r", "`n")
}

New-Item -ItemType Directory -Force -Path $buildRoot | Out-Null
New-Item -ItemType Directory -Force -Path $includeRoot | Out-Null

$legacyHeaders = @(
    "status.h",
    "types.h",
    "version.h",
    (Join-Path "solver" "common.h"),
    (Join-Path "solver" "formulation.h"),
    (Join-Path "solver" "interpolation.h"),
    (Join-Path "solver" "reach_set2.h"),
    (Join-Path "solver" "topp2_ra.h"),
    (Join-Path "solver" "copp2_socp.h")
)
foreach ($legacyHeader in $legacyHeaders) {
    $legacyPath = Join-Path $includeRoot $legacyHeader
    if (Test-Path $legacyPath) {
        Remove-Item -LiteralPath $legacyPath -Force
    }
}
$legacySolverDir = Join-Path $includeRoot "solver"
if ((Test-Path $legacySolverDir) -and -not (Get-ChildItem -LiteralPath $legacySolverDir -Force)) {
    Remove-Item -LiteralPath $legacySolverDir -Force
}

Push-Location $repoRoot
try {
    & cbindgen --quiet --config $configPath --crate copp --output $generatedHeader
    if ($LASTEXITCODE -ne 0) {
        throw "cbindgen failed with exit code $LASTEXITCODE"
    }
} finally {
    Pop-Location
}

$fullText = [System.IO.File]::ReadAllText($generatedHeader, [System.Text.Encoding]::UTF8)

function Convert-CoppDocs {
    param([Parameter(Mandatory = $true)][string]$Text)

    $converted = $Text
    $converted = $converted -replace '\[`CoppStatus::Ok`\]', '`COPP_STATUS_OK`'
    $converted = $converted -replace '\[`([^`\]]+)`\]\([^)]*\)', '`$1`'
    $converted = $converted -replace '\[`([^`\]]+)`\]', '`$1`'
    $converted = $converted -replace '(?m)^ \* # Safety\s*$', ' * \warning Safety'
    $converted = $converted -replace '(?m)^ \* # Errors\s*$', ' * \par Errors'
    $converted = $converted -replace '(?m)^ \* # Parameters\s*$', ' * \par Parameters'
    $converted = $converted -replace '(?m)^ \* # Returns\s*$', ' * \par Returns'
    $converted = $converted -replace '(?m)^ \* # Notes\s*$', ' * \par Notes'
    $converted = $converted -replace 'Rust-owned', 'library-owned'
    $converted = $converted -replace 'Rust-allocated', 'COPP-allocated'
    $converted = $converted -replace 'Rust allocation capacity', 'Allocation capacity'
    $converted = $converted -replace 'Rust caught a panic at the C ABI boundary', 'COPP caught an internal panic at the C ABI boundary'
    $converted = $converted -replace 'Return the Rust crate version from `Cargo\.toml`', 'Return the COPP library version'
    $converted = $converted -replace 'Rust internals', 'COPP internals'
    $converted = $converted -replace 'before passing it to Rust internals', 'before passing it to COPP internals'
    $converted = $converted -replace 'before Rust uses it', 'before COPP uses it'
    $converted = $converted -replace 'Rust converts it', 'COPP converts it'
    $converted = $converted -replace 'Rust reduces', 'COPP reduces'
    $converted = $converted -replace 'C callers pass profile parts explicitly instead of constructing a\s+\*\s+`Topp3ProfileRef`:', 'C callers pass profile parts explicitly:'
    $converted = $converted -replace 'C callers pass profile parts explicitly instead of constructing a\s+\*\s+`Topp3ProfileRef`\.', 'C callers pass profile parts explicitly.'
    $converted = $converted -replace 'C callers pass profile parts explicitly instead of constructing a\s+\*\s+`Topp3ProfileMut`\.', 'C callers pass mutable profile parts explicitly.'
    $converted = $converted -replace 'C callers pass profile parts explicitly instead of constructing a\s+`Topp3ProfileRef`:', 'C callers pass profile parts explicitly:'
    $converted = $converted -replace 'C callers pass profile parts explicitly instead of constructing a\s+`Topp3ProfileRef`\.', 'C callers pass profile parts explicitly.'
    $converted = $converted -replace 'C callers pass profile parts explicitly instead of constructing a\s+`Topp3ProfileMut`\.', 'C callers pass mutable profile parts explicitly.'
    $converted = $converted -replace 'the Rust return flag', 'the operation result flag'
    $converted = $converted -replace 'This C ABI builds a Rust ([A-Z0-9]+) problem', 'This C ABI builds an internal $1 problem'
    $converted = $converted -replace 'The solver builds the Rust problem', 'The solver builds the internal problem'
    $converted = $converted -replace 'Building the Rust problem', 'Building the internal problem'
    $converted = $converted -replace 'Building the Rust TOPP3 problem', 'Building the internal TOPP3 problem'
    $converted = $converted -replace 'Building the Rust `Topp3Problem`', 'Building the internal `Topp3Problem`'
    $converted = $converted -replace 'This expert entry mirrors Rust `([^`]+)`', 'This expert entry mirrors `$1`'
    $converted = $converted -replace 'These fields mirror Rust `([^`]+)`', 'These fields match `$1`'
    $converted = $converted -replace 'This mirrors Rust `([^`]+)`', 'This matches `$1`'
    $converted = $converted -replace 'The tolerance fields mirror Rust `([^`]+)`', 'The tolerance fields match `$1`'
    $converted = $converted -replace 'The field order follows the Rust `ReachSet2`\s+ \* struct: `a_max` first, then `a_min`\.  Both vectors are library-owned', 'The field order is `a_max` first, then `a_min`. Both vectors are library-owned'
    $converted = $converted -replace 'This performs the same backward pass as Rust `reach_set2_backward`:', 'This performs the same backward pass as `reach_set2_backward`:'
    $converted = $converted -replace "This helper mirrors Rust's ``([^``]+)`` layout rule", 'This helper uses the public COPP profile layout'
    return $converted
}

function Get-CoppBlock {
    param(
        [Parameter(Mandatory = $true)][string]$Name,
        [Parameter(Mandatory = $true)][string]$Pattern
    )

    $match = [regex]::Match($fullText, "(?ms)$Pattern")
    if (-not $match.Success) {
        throw "Could not find generated C declaration for $Name"
    }

    $declStart = $match.Index
    $declEnd = $match.Index + $match.Length
    $docEnd = $fullText.LastIndexOf("*/", $declStart)
    if ($docEnd -ge 0) {
        $between = $fullText.Substring($docEnd + 2, $declStart - ($docEnd + 2))
        if ($between -match "^\s*$") {
            $docStart = $fullText.LastIndexOf("/**", $docEnd)
            if ($docStart -ge 0) {
                return (Convert-CoppDocs -Text ($fullText.Substring($docStart, $declEnd - $docStart))).Trim()
            }
        }
    }

    return (Convert-CoppDocs -Text $match.Value).Trim()
}

function Join-CoppBlocks {
    param([Parameter(Mandatory = $true)][string[]]$Blocks)
    return Normalize-CoppNewlines -Text (($Blocks | ForEach-Object { $_.Trim() }) -join ($nl + $nl))
}

function Write-CoppHeader {
    param(
        [Parameter(Mandatory = $true)][string]$RelativePath,
        [Parameter(Mandatory = $true)][string]$Guard,
        [string[]]$Includes = @(),
        [Parameter(Mandatory = $true)][string]$Body,
        [string]$Group = "",
        [string]$Brief = ""
    )

    $path = Join-Path $includeRoot $RelativePath
    $dir = Split-Path -Parent $path
    New-Item -ItemType Directory -Force -Path $dir | Out-Null

    $lines = [System.Collections.Generic.List[string]]::new()
    $lines.Add("#ifndef $Guard")
    $lines.Add("#define $Guard")
    $lines.Add("")
    $lines.Add("/*")
    $lines.Add(" * Generated by bindings/c/scripts/generate_headers.ps1.")
    $lines.Add(" * Do not edit by hand; update src/ffi/c and regenerate.")
    $lines.Add(" */")
    if ($Group) {
        $lines.Add("")
        $lines.Add("/**")
        $lines.Add(" * \file $RelativePath")
        if ($Brief) {
            $lines.Add(" * \brief $Brief")
        }
        $lines.Add(" * \ingroup $Group")
        $lines.Add(" */")
    }
    if ($Includes.Count -gt 0) {
        $lines.Add("")
        foreach ($include in $Includes) {
            $lines.Add("#include $include")
        }
    }
    $lines.Add("")
    $lines.Add("#ifdef __cplusplus")
    $lines.Add('extern "C" {')
    $lines.Add("#endif")
    $lines.Add("")
    if ($Group) {
        $lines.Add("/**")
        $lines.Add(" * \addtogroup $Group")
        $lines.Add(" * @{")
        $lines.Add(" */")
        $lines.Add("")
    }
    $lines.Add((Normalize-CoppNewlines -Text $Body.Trim()))
    if ($Group) {
        $lines.Add("")
        $lines.Add("/** @} */")
    }
    $lines.Add("")
    $lines.Add("#ifdef __cplusplus")
    $lines.Add("}")
    $lines.Add("#endif")
    $lines.Add("")
    $lines.Add("#endif /* $Guard */")

    [System.IO.File]::WriteAllText($path, (($lines -join $nl) + $nl), $utf8NoBom)
}

function Write-CoppUmbrellaHeader {
    $path = Join-Path $includeRoot "copp.h"
    $lines = @(
        "#ifndef COPP_H",
        "#define COPP_H",
        "",
        "/*",
        " * Generated by bindings/c/scripts/generate_headers.ps1.",
        " * Do not edit by hand; update src/ffi/c and regenerate.",
        " */",
        "",
        "/*",
        " * COPP C ABI umbrella header.",
        " *",
        " * Include this file for the full C ABI surface, or include a narrower header",
        " * such as `"copp/path.h`" or `"copp/topp2.h`" for smaller translation",
        " * units.",
        " */",
        "",
        "#include `"copp/core.h`"",
        "#include `"copp/path.h`"",
        "#include `"copp/robot.h`"",
        "#include `"copp/formulation.h`"",
        "#include `"copp/interpolation.h`"",
        "#include `"copp/topp2.h`"",
        "#include `"copp/copp2.h`"",
        "#include `"copp/topp3.h`"",
        "#include `"copp/copp3.h`"",
        "",
        "#endif /* COPP_H */"
    )
    [System.IO.File]::WriteAllText($path, (($lines -join $nl) + $nl), $utf8NoBom)
}

$blocks = @{
    CoppStatus = Get-CoppBlock "CoppStatus" "typedef enum CoppStatus\s*\{.*?\}\s*CoppStatus;"
    CoppPathOutOfRangeMode = Get-CoppBlock "CoppPathOutOfRangeMode" "typedef enum CoppPathOutOfRangeMode\s*\{.*?\}\s*CoppPathOutOfRangeMode;"
    CoppPathParametrization = Get-CoppBlock "CoppPathParametrization" "typedef enum CoppPathParametrization\s*\{.*?\}\s*CoppPathParametrization;"
    CoppVerbosity = Get-CoppBlock "CoppVerbosity" "typedef enum CoppVerbosity\s*\{.*?\}\s*CoppVerbosity;"
    CoppObjectiveKind = Get-CoppBlock "CoppObjectiveKind" "typedef enum CoppObjectiveKind\s*\{.*?\}\s*CoppObjectiveKind;"
    CoppClarabelDirectSolveMethod = Get-CoppBlock "CoppClarabelDirectSolveMethod" "typedef enum CoppClarabelDirectSolveMethod\s*\{.*?\}\s*CoppClarabelDirectSolveMethod;"
    CoppClarabelSolverStatus = Get-CoppBlock "CoppClarabelSolverStatus" "typedef enum CoppClarabelSolverStatus\s*\{.*?\}\s*CoppClarabelSolverStatus;"
    CoppPath = Get-CoppBlock "CoppPath" "typedef struct CoppPath\s+CoppPath;"
    CoppRobot = Get-CoppBlock "CoppRobot" "typedef struct CoppRobot\s+CoppRobot;"
    CoppMatrixLayout = Get-CoppBlock "CoppMatrixLayout" "typedef enum CoppMatrixLayout\s*\{.*?\}\s*CoppMatrixLayout;"
    CoppVecF64 = Get-CoppBlock "CoppVecF64" "typedef struct CoppVecF64\s*\{.*?\}\s*CoppVecF64;"
    CoppVecUsize = Get-CoppBlock "CoppVecUsize" "typedef struct CoppVecUsize\s*\{.*?\}\s*CoppVecUsize;"
    CoppMatrixF64 = Get-CoppBlock "CoppMatrixF64" "typedef struct CoppMatrixF64\s*\{.*?\}\s*CoppMatrixF64;"
    CoppMatrixViewF64 = Get-CoppBlock "CoppMatrixViewF64" "typedef struct CoppMatrixViewF64\s*\{.*?\}\s*CoppMatrixViewF64;"
    CoppSliceF64 = Get-CoppBlock "CoppSliceF64" "typedef struct CoppSliceF64\s*\{.*?\}\s*CoppSliceF64;"
    CoppSliceMutF64 = Get-CoppBlock "CoppSliceMutF64" "typedef struct CoppSliceMutF64\s*\{.*?\}\s*CoppSliceMutF64;"
    CoppPathOptions = Get-CoppBlock "CoppPathOptions" "typedef struct CoppPathOptions\s*\{.*?\}\s*CoppPathOptions;"
    CoppPathEvaluate2ndFn = Get-CoppBlock "CoppPathEvaluate2ndFn" "typedef enum CoppStatus\s+\(\*CoppPathEvaluate2ndFn\)\(.*?\);"
    CoppPathEvaluate3rdFn = Get-CoppBlock "CoppPathEvaluate3rdFn" "typedef enum CoppStatus\s+\(\*CoppPathEvaluate3rdFn\)\(.*?\);"
    CoppInverseDynamicsFn = Get-CoppBlock "CoppInverseDynamicsFn" "typedef enum CoppStatus\s+\(\*CoppInverseDynamicsFn\)\(.*?\);"
    CoppClarabelSettings = Get-CoppBlock "CoppClarabelSettings" "typedef struct CoppClarabelSettings\s*\{.*?\}\s*CoppClarabelSettings;"
    CoppClarabelOptions = Get-CoppBlock "CoppClarabelOptions" "typedef struct CoppClarabelOptions\s*\{.*?\}\s*CoppClarabelOptions;"
    CoppClarabelLinearSolverInfo = Get-CoppBlock "CoppClarabelLinearSolverInfo" "typedef struct CoppClarabelLinearSolverInfo\s*\{.*?\}\s*CoppClarabelLinearSolverInfo;"
    Copp2SocpResult = Get-CoppBlock "Copp2SocpResult" "typedef struct Copp2SocpResult\s*\{.*?\}\s*Copp2SocpResult;"
    Copp3SocpResult = Get-CoppBlock "Copp3SocpResult" "typedef struct Copp3SocpResult\s*\{.*?\}\s*Copp3SocpResult;"
    CoppReachSet2Result = Get-CoppBlock "CoppReachSet2Result" "typedef struct CoppReachSet2Result\s*\{.*?\}\s*CoppReachSet2Result;"
    CoppObjective = Get-CoppBlock "CoppObjective" "typedef struct CoppObjective\s*\{.*?\}\s*CoppObjective;"
    Copp2Problem = Get-CoppBlock "Copp2Problem" "typedef struct Copp2Problem\s*\{.*?\}\s*Copp2Problem;"
    CoppProfile3rd = Get-CoppBlock "CoppProfile3rd" "typedef struct CoppProfile3rd\s*\{.*?\}\s*CoppProfile3rd;"
    Topp3Problem = Get-CoppBlock "Topp3Problem" "typedef struct Topp3Problem\s*\{.*?\}\s*Topp3Problem;"
    Copp3Problem = Get-CoppBlock "Copp3Problem" "typedef struct Copp3Problem\s*\{.*?\}\s*Copp3Problem;"
    Topp2RaOptions = Get-CoppBlock "Topp2RaOptions" "typedef struct Topp2RaOptions\s*\{.*?\}\s*Topp2RaOptions;"
    Topp2Problem = Get-CoppBlock "Topp2Problem" "typedef struct Topp2Problem\s*\{.*?\}\s*Topp2Problem;"
}

function Get-CoppFunction {
    param([Parameter(Mandatory = $true)][string]$Name)
    return Get-CoppBlock $Name "(?:(?:enum CoppStatus|void|size_t)\s+|const char\s+\*\s*)$Name\s*\(.*?\);"
}

$functions = @{
    copp_vec_f64_free = Get-CoppFunction "copp_vec_f64_free"
    copp_vec_usize_free = Get-CoppFunction "copp_vec_usize_free"
    copp_matrix_f64_free = Get-CoppFunction "copp_matrix_f64_free"
    copp_profile_3rd_free = Get-CoppFunction "copp_profile_3rd_free"
    copp_version = Get-CoppFunction "copp_version"
    copp_status_message = Get-CoppFunction "copp_status_message"
    copp_last_error_code = Get-CoppFunction "copp_last_error_code"
    copp_last_error_message = Get-CoppFunction "copp_last_error_message"
    copp_last_error_message_len = Get-CoppFunction "copp_last_error_message_len"
    copp_last_error_message_copy = Get-CoppFunction "copp_last_error_message_copy"
    copp_clear_last_error = Get-CoppFunction "copp_clear_last_error"
    copp_set_last_error_message = Get-CoppFunction "copp_set_last_error_message"
    copp_set_last_error_message_n = Get-CoppFunction "copp_set_last_error_message_n"
    copp_path_default_options = Get-CoppFunction "copp_path_default_options"
    copp_path_from_waypoints = Get-CoppFunction "copp_path_from_waypoints"
    copp_path_from_evaluator_2nd = Get-CoppFunction "copp_path_from_evaluator_2nd"
    copp_path_from_evaluator_3rd = Get-CoppFunction "copp_path_from_evaluator_3rd"
    copp_path_dim = Get-CoppFunction "copp_path_dim"
    copp_path_s_range = Get-CoppFunction "copp_path_s_range"
    copp_path_evaluate_up_to_2nd = Get-CoppFunction "copp_path_evaluate_up_to_2nd"
    copp_path_evaluate_up_to_3rd = Get-CoppFunction "copp_path_evaluate_up_to_3rd"
    copp_path_free = Get-CoppFunction "copp_path_free"
    copp_robot_create = Get-CoppFunction "copp_robot_create"
    copp_robot_set_inverse_dynamics = Get-CoppFunction "copp_robot_set_inverse_dynamics"
    copp_robot_clear_inverse_dynamics = Get-CoppFunction "copp_robot_clear_inverse_dynamics"
    copp_robot_append_s = Get-CoppFunction "copp_robot_append_s"
    copp_robot_amax_substitute = Get-CoppFunction "copp_robot_amax_substitute"
    copp_robot_dim = Get-CoppFunction "copp_robot_dim"
    copp_robot_len = Get-CoppFunction "copp_robot_len"
    copp_robot_capacity = Get-CoppFunction "copp_robot_capacity"
    copp_robot_is_empty = Get-CoppFunction "copp_robot_is_empty"
    copp_robot_idx_s_start = Get-CoppFunction "copp_robot_idx_s_start"
    copp_robot_idx_s_end = Get-CoppFunction "copp_robot_idx_s_end"
    copp_robot_constraint_rows = Get-CoppFunction "copp_robot_constraint_rows"
    copp_robot_get_s = Get-CoppFunction "copp_robot_get_s"
    copp_robot_get_amax = Get-CoppFunction "copp_robot_get_amax"
    copp_robot_s_vec = Get-CoppFunction "copp_robot_s_vec"
    copp_robot_amax_vec = Get-CoppFunction "copp_robot_amax_vec"
    copp_robot_acc_constraints_at = Get-CoppFunction "copp_robot_acc_constraints_at"
    copp_robot_jerk_constraints_at = Get-CoppFunction "copp_robot_jerk_constraints_at"
    copp_robot_jerk_linear_constraints_at = Get-CoppFunction "copp_robot_jerk_linear_constraints_at"
    copp_robot_clear_constraints = Get-CoppFunction "copp_robot_clear_constraints"
    copp_robot_pop_front_n = Get-CoppFunction "copp_robot_pop_front_n"
    copp_robot_pop_back_n = Get-CoppFunction "copp_robot_pop_back_n"
    copp_robot_pop_front_until = Get-CoppFunction "copp_robot_pop_front_until"
    copp_robot_pop_back_until = Get-CoppFunction "copp_robot_pop_back_until"
    copp_robot_sample_path_2nd = Get-CoppFunction "copp_robot_sample_path_2nd"
    copp_robot_sample_path_3rd = Get-CoppFunction "copp_robot_sample_path_3rd"
    copp_robot_set_q_2nd = Get-CoppFunction "copp_robot_set_q_2nd"
    copp_robot_set_q_3rd = Get-CoppFunction "copp_robot_set_q_3rd"
    copp_add_raw_constraint_1st = Get-CoppFunction "copp_add_raw_constraint_1st"
    copp_add_raw_constraint_2nd = Get-CoppFunction "copp_add_raw_constraint_2nd"
    copp_add_raw_constraint_3rd = Get-CoppFunction "copp_add_raw_constraint_3rd"
    copp_add_axial_velocity_limits = Get-CoppFunction "copp_add_axial_velocity_limits"
    copp_add_axial_velocity_limits_matrix = Get-CoppFunction "copp_add_axial_velocity_limits_matrix"
    copp_add_axial_acceleration_limits = Get-CoppFunction "copp_add_axial_acceleration_limits"
    copp_add_axial_acceleration_limits_matrix = Get-CoppFunction "copp_add_axial_acceleration_limits_matrix"
    copp_add_axial_torque_limits = Get-CoppFunction "copp_add_axial_torque_limits"
    copp_add_axial_torque_limits_matrix = Get-CoppFunction "copp_add_axial_torque_limits_matrix"
    copp_add_axial_jerk_limits = Get-CoppFunction "copp_add_axial_jerk_limits"
    copp_add_axial_jerk_limits_matrix = Get-CoppFunction "copp_add_axial_jerk_limits_matrix"
    copp_robot_free = Get-CoppFunction "copp_robot_free"
    copp_clarabel_default_options = Get-CoppFunction "copp_clarabel_default_options"
    copp_clarabel_solution_to_profile_2nd = Get-CoppFunction "copp_clarabel_solution_to_profile_2nd"
    copp_clarabel_solution_to_profile_3rd = Get-CoppFunction "copp_clarabel_solution_to_profile_3rd"
    copp2_socp = Get-CoppFunction "copp2_socp"
    copp2_socp_expert = Get-CoppFunction "copp2_socp_expert"
    copp2_socp_result_free = Get-CoppFunction "copp2_socp_result_free"
    copp3_socp = Get-CoppFunction "copp3_socp"
    copp3_socp_expert = Get-CoppFunction "copp3_socp_expert"
    copp3_socp_result_free = Get-CoppFunction "copp3_socp_result_free"
    topp3_socp = Get-CoppFunction "topp3_socp"
    topp3_socp_expert = Get-CoppFunction "topp3_socp_expert"
    topp3_lp = Get-CoppFunction "topp3_lp"
    topp3_lp_expert = Get-CoppFunction "topp3_lp_expert"
    copp_a_to_b_2nd = Get-CoppFunction "copp_a_to_b_2nd"
    copp_s_to_t_2nd = Get-CoppFunction "copp_s_to_t_2nd"
    copp_t_to_s_uniform_2nd = Get-CoppFunction "copp_t_to_s_uniform_2nd"
    copp_t_to_s_non_uniform_2nd = Get-CoppFunction "copp_t_to_s_non_uniform_2nd"
    copp_s_to_t_3rd = Get-CoppFunction "copp_s_to_t_3rd"
    copp_t_to_s_uniform_3rd = Get-CoppFunction "copp_t_to_s_uniform_3rd"
    copp_t_to_s_non_uniform_3rd = Get-CoppFunction "copp_t_to_s_non_uniform_3rd"
    copp_force_positive_a_3rd = Get-CoppFunction "copp_force_positive_a_3rd"
    copp_reach_set2_backward = Get-CoppFunction "copp_reach_set2_backward"
    copp_reach_set2_bidirectional = Get-CoppFunction "copp_reach_set2_bidirectional"
    copp_reach_set2_result_free = Get-CoppFunction "copp_reach_set2_result_free"
    topp2_ra_default_options = Get-CoppFunction "topp2_ra_default_options"
    topp2_ra = Get-CoppFunction "topp2_ra"
}

Write-CoppUmbrellaHeader

Write-CoppHeader "core.h" "COPP_CORE_H" @(
    "<stddef.h>",
    "<stdbool.h>",
    "<stdint.h>"
) (Join-CoppBlocks @(
    $blocks.CoppStatus,
    $functions.copp_status_message,
    $functions.copp_last_error_code,
    $functions.copp_last_error_message,
    $functions.copp_last_error_message_len,
    $functions.copp_last_error_message_copy,
    $functions.copp_clear_last_error,
    $functions.copp_set_last_error_message,
    $functions.copp_set_last_error_message_n,
    $functions.copp_version,
    $blocks.CoppVerbosity,
    $blocks.CoppClarabelDirectSolveMethod,
    $blocks.CoppClarabelSolverStatus,
    $blocks.CoppClarabelLinearSolverInfo,
    $blocks.CoppClarabelSettings,
    $blocks.CoppClarabelOptions,
    $functions.copp_clarabel_default_options,
    $blocks.CoppMatrixLayout,
    $blocks.CoppMatrixViewF64,
    @"
/**
 * Build a contiguous column-major matrix view.
 *
 * This is the recommended zero-copy input layout for COPP C APIs.
 */
#ifdef __cplusplus
#define COPP_MATRIX_VIEW_F64_COLUMN_MAJOR(data_, rows_, cols_) \
    CoppMatrixViewF64{(data_), (rows_), (cols_), COPP_MATRIX_LAYOUT_COLUMN_MAJOR, (rows_)}
#else
#define COPP_MATRIX_VIEW_F64_COLUMN_MAJOR(data_, rows_, cols_) \
    ((struct CoppMatrixViewF64){(data_), (rows_), (cols_), COPP_MATRIX_LAYOUT_COLUMN_MAJOR, (rows_)})
#endif

/**
 * Build a contiguous row-major matrix view.
 *
 * COPP accepts this layout, but it converts the input to column-major temporary
 * storage before use.
 */
#ifdef __cplusplus
#define COPP_MATRIX_VIEW_F64_ROW_MAJOR(data_, rows_, cols_) \
    CoppMatrixViewF64{(data_), (rows_), (cols_), COPP_MATRIX_LAYOUT_ROW_MAJOR, (cols_)}
#else
#define COPP_MATRIX_VIEW_F64_ROW_MAJOR(data_, rows_, cols_) \
    ((struct CoppMatrixViewF64){(data_), (rows_), (cols_), COPP_MATRIX_LAYOUT_ROW_MAJOR, (cols_)})
#endif
"@,
    $blocks.CoppSliceF64,
    $blocks.CoppSliceMutF64,
    $blocks.CoppVecF64,
    $blocks.CoppVecUsize,
    $blocks.CoppMatrixF64,
    $functions.copp_vec_f64_free,
    $functions.copp_vec_usize_free,
    $functions.copp_matrix_f64_free,
    @"
/**
 * Forward declaration for third-order profiles returned by TOPP3/COPP3 solvers.
 *
 * Include `copp/formulation.h` for the full definition and
 * `copp_profile_3rd_free`.
 */
struct CoppProfile3rd;
"@,
    $functions.copp_clarabel_solution_to_profile_2nd,
    $functions.copp_clarabel_solution_to_profile_3rd
)) -Group "copp_core" -Brief "Core status, memory, matrix, and shared solver option types."

Write-CoppHeader "path.h" "COPP_PATH_H" @(
    "<stddef.h>",
    '"copp/core.h"'
) (Join-CoppBlocks @(
    $blocks.CoppPath,
    $blocks.CoppPathOutOfRangeMode,
    $blocks.CoppPathParametrization,
    $blocks.CoppPathOptions,
    $blocks.CoppPathEvaluate2ndFn,
    $blocks.CoppPathEvaluate3rdFn,
    $functions.copp_path_default_options,
    $functions.copp_path_from_waypoints,
    $functions.copp_path_from_evaluator_2nd,
    $functions.copp_path_from_evaluator_3rd,
    $functions.copp_path_dim,
    $functions.copp_path_s_range,
    $functions.copp_path_evaluate_up_to_2nd,
    $functions.copp_path_evaluate_up_to_3rd,
    $functions.copp_path_free
)) -Group "copp_path" -Brief "Path construction, metadata, evaluation, and callback path APIs."

Write-CoppHeader "robot.h" "COPP_ROBOT_H" @(
    "<stddef.h>",
    '"copp/core.h"',
    '"copp/path.h"'
) (Join-CoppBlocks @(
    $blocks.CoppRobot,
    $blocks.CoppInverseDynamicsFn,
    $functions.copp_robot_create,
    $functions.copp_robot_set_inverse_dynamics,
    $functions.copp_robot_clear_inverse_dynamics,
    $functions.copp_robot_append_s,
    $functions.copp_robot_amax_substitute,
    $functions.copp_robot_dim,
    $functions.copp_robot_len,
    $functions.copp_robot_capacity,
    $functions.copp_robot_is_empty,
    $functions.copp_robot_idx_s_start,
    $functions.copp_robot_idx_s_end,
    $functions.copp_robot_constraint_rows,
    $functions.copp_robot_get_s,
    $functions.copp_robot_get_amax,
    $functions.copp_robot_s_vec,
    $functions.copp_robot_amax_vec,
    $functions.copp_robot_acc_constraints_at,
    $functions.copp_robot_jerk_constraints_at,
    $functions.copp_robot_jerk_linear_constraints_at,
    $functions.copp_robot_clear_constraints,
    $functions.copp_robot_pop_front_n,
    $functions.copp_robot_pop_back_n,
    $functions.copp_robot_pop_front_until,
    $functions.copp_robot_pop_back_until,
    $functions.copp_robot_sample_path_2nd,
    $functions.copp_robot_sample_path_3rd,
    $functions.copp_robot_set_q_2nd,
    $functions.copp_robot_set_q_3rd,
    $functions.copp_add_raw_constraint_1st,
    $functions.copp_add_raw_constraint_2nd,
    $functions.copp_add_raw_constraint_3rd,
    $functions.copp_add_axial_velocity_limits,
    $functions.copp_add_axial_velocity_limits_matrix,
    $functions.copp_add_axial_acceleration_limits,
    $functions.copp_add_axial_acceleration_limits_matrix,
    $functions.copp_add_axial_torque_limits,
    $functions.copp_add_axial_torque_limits_matrix,
    $functions.copp_add_axial_jerk_limits,
    $functions.copp_add_axial_jerk_limits_matrix,
    $functions.copp_robot_free
)) -Group "copp_robot" -Brief "Robot handles, path sampling, derivative input, constraints, and limits."

Write-CoppHeader "formulation.h" "COPP_FORMULATION_H" @(
    "<stddef.h>",
    '"copp/core.h"',
    '"copp/robot.h"'
) (Join-CoppBlocks @(
    $blocks.CoppObjectiveKind,
    $blocks.CoppObjective,
    $blocks.Copp2Problem,
    $blocks.Topp2Problem,
    $blocks.CoppProfile3rd,
    $functions.copp_profile_3rd_free,
    $blocks.Topp3Problem,
    $blocks.Copp3Problem
)) -Group "copp_formulation" -Brief "TOPP/COPP problem descriptors, objectives, and profile ownership."

Write-CoppHeader "interpolation.h" "COPP_INTERPOLATION_H" @(
    "<stdbool.h>",
    '"copp/core.h"'
) (Join-CoppBlocks @(
    $functions.copp_a_to_b_2nd,
    $functions.copp_s_to_t_2nd,
    $functions.copp_t_to_s_uniform_2nd,
    $functions.copp_t_to_s_non_uniform_2nd,
    $functions.copp_s_to_t_3rd,
    $functions.copp_t_to_s_uniform_3rd,
    $functions.copp_t_to_s_non_uniform_3rd,
    $functions.copp_force_positive_a_3rd
)) -Group "copp_interpolation" -Brief "Second- and third-order interpolation utilities."

Write-CoppHeader "topp2.h" "COPP_TOPP2_H" @(
    "<stddef.h>",
    '"copp/core.h"',
    '"copp/formulation.h"'
) (Join-CoppBlocks @(
    $blocks.Topp2RaOptions,
    $functions.topp2_ra_default_options,
    $functions.topp2_ra,
    $blocks.CoppReachSet2Result,
    $functions.copp_reach_set2_backward,
    $functions.copp_reach_set2_bidirectional,
    $functions.copp_reach_set2_result_free
)) -Group "copp_topp2" -Brief "TOPP2-RA and second-order reachable-set APIs."

Write-CoppHeader "copp2.h" "COPP_COPP2_H" @(
    "<stddef.h>",
    "<stdbool.h>",
    "<stdint.h>",
    '"copp/core.h"',
    '"copp/formulation.h"'
) (Join-CoppBlocks @(
    $blocks.Copp2SocpResult,
    $functions.copp2_socp,
    $functions.copp2_socp_expert,
    $functions.copp2_socp_result_free
)) -Group "copp_copp2" -Brief "COPP2-SOCP solver APIs."

Write-CoppHeader "topp3.h" "COPP_TOPP3_H" @(
    "<stddef.h>",
    '"copp/core.h"',
    '"copp/formulation.h"'
) (Join-CoppBlocks @(
    @"
/**
 * Forward declaration for expert TOPP3-SOCP diagnostics.
 *
 * Include `copp/copp3.h` when the full `Copp3SocpResult` definition or
 * `copp3_socp_result_free` is needed.
 */
typedef struct Copp3SocpResult Copp3SocpResult;
"@,
    $functions.topp3_lp,
    $functions.topp3_lp_expert,
    $functions.topp3_socp,
    $functions.topp3_socp_expert
)) -Group "copp_topp3" -Brief "TOPP3-LP and TOPP3-SOCP APIs."

Write-CoppHeader "copp3.h" "COPP_COPP3_H" @(
    "<stddef.h>",
    "<stdbool.h>",
    "<stdint.h>",
    '"copp/core.h"',
    '"copp/formulation.h"',
    '"copp/topp3.h"'
) (Join-CoppBlocks @(
    $blocks.Copp3SocpResult,
    $functions.copp3_socp,
    $functions.copp3_socp_expert,
    $functions.copp3_socp_result_free
)) -Group "copp_copp3" -Brief "COPP3-SOCP solver APIs."

Write-Host "Generated split C headers under $includeRoot"
