function build(opts)
%BUILD Build the COPP MATLAB MEX gateway.
%
% build() compiles +copp/+internal/copp_mex and links it to the native
% copp library built by Cargo. The default configuration links the static
% native library so end users do not need to manage a separate runtime library.
%
% Name-value options:
%   Linkage:
%       "static" links against the native static library and adds platform
%       system libraries. "dynamic" links against the native shared library
%       and copies the runtime library into +copp/+internal. On Linux the
%       MEX is linked with rpath=$ORIGIN; on macOS it is linked with
%       rpath=@loader_path, so the copied runtime can be found next to the MEX.
%   Configuration:
%       "release" or "debug", selecting target/<configuration>.
%
% Build Rust first, for example:
%   cargo build --release --lib --features matlab
arguments
    opts.Linkage (1,1) string {mustBeMember(opts.Linkage, ["static", "dynamic"])} = "static"
    opts.Configuration (1,1) string {mustBeMember(opts.Configuration, ["debug", "release"])} = "release"
end

matlab_dir = fileparts(mfilename('fullpath'));
linkage = char(opts.Linkage);
configuration = char(opts.Configuration);
repo_root = fullfile(matlab_dir, '..', '..');
src = fullfile(matlab_dir, 'src', 'copp_mex.cpp');
include_dir = fullfile(repo_root, 'bindings', 'c', 'include');
lib_dir = fullfile(repo_root, 'target', configuration);
out_dir = fullfile(matlab_dir, '+copp', '+internal');

if ~isfolder(out_dir)
    mkdir(out_dir);
end

lib_file = nativeLibraryFile(lib_dir, linkage);
link_inputs = nativeLinkInputs(lib_dir, lib_file, linkage);

mex_args = [{'-R2018a'}, ...
    {['-I', include_dir]}, ...
    {src}, ...
    link_inputs(:)', ...
    {'-outdir'}, {out_dir}, ...
    {'-output'}, {'copp_mex'}];

% On Linux, statically link libstdc++ to avoid GLIBCXX version conflicts
% with MATLAB's bundled libstdc++.so
if isunix && ~ismac
    mex_args = [mex_args, {'CXXFLAGS="$CXXFLAGS -static-libstdc++\"'}];
end

mex(mex_args{:});

if strcmp(linkage, 'dynamic')
    runtime = nativeRuntimeFile(lib_dir);
    if isempty(runtime) || ~isfile(runtime)
        error("copp:BuildError", ...
            "Native runtime library not found: %s.", runtime);
    end
    copyfile(runtime, out_dir);
    fixMacDynamicInstallNames(lib_file, fullfile(out_dir, runtimeFileName()), ...
        fullfile(out_dir, ['copp_mex.', mexext]));
end
end

function inputs = nativeLinkInputs(lib_dir, lib_file, linkage)
if strcmp(linkage, 'dynamic') && ~ispc
    inputs = {['-L', lib_dir], '-lcopp', dynamicRuntimePathFlag()};
else
    inputs = {lib_file};
end
inputs = [inputs, nativeSystemLibraries(linkage)];
end

function lib_file = nativeLibraryFile(lib_dir, linkage)
if ispc
    if strcmp(linkage, 'dynamic')
        lib_file = fullfile(lib_dir, 'copp.dll.lib');
    else
        lib_file = fullfile(lib_dir, 'copp.lib');
    end
elseif ismac
    if strcmp(linkage, 'dynamic')
        lib_file = fullfile(lib_dir, 'libcopp.dylib');
    else
        lib_file = fullfile(lib_dir, 'libcopp.a');
    end
else
    if strcmp(linkage, 'dynamic')
        lib_file = fullfile(lib_dir, 'libcopp.so');
    else
        lib_file = fullfile(lib_dir, 'libcopp.a');
    end
end

if ~isfile(lib_file)
    error("copp:BuildError", ...
        "Native library not found: %s. Build Rust first, for example with cargo build --release --lib --features matlab.", ...
        lib_file);
end
end

function runtime = nativeRuntimeFile(lib_dir)
if ispc
    runtime = fullfile(lib_dir, 'copp.dll');
elseif ismac
    runtime = fullfile(lib_dir, 'libcopp.dylib');
else
    runtime = fullfile(lib_dir, 'libcopp.so');
end
end

function name = runtimeFileName()
if ispc
    name = 'copp.dll';
elseif ismac
    name = 'libcopp.dylib';
else
    name = 'libcopp.so';
end
end

function flag = dynamicRuntimePathFlag()
if ismac
    flag = 'LDFLAGS="$LDFLAGS -Wl,-rpath,@loader_path"';
elseif isunix
    flag = 'LDFLAGS="$LDFLAGS -Wl,-rpath,\$ORIGIN"';
else
    flag = '';
end
end

function fixMacDynamicInstallNames(source_runtime, copied_runtime, mex_file)
if ~ismac
    return
end

runInstallNameTool({'-id', '@rpath/libcopp.dylib', copied_runtime});
runInstallNameTool({'-change', source_runtime, '@rpath/libcopp.dylib', mex_file});
runInstallNameTool({'-change', 'libcopp.dylib', '@rpath/libcopp.dylib', mex_file});
end

function runInstallNameTool(args)
quoted_args = cellfun(@shellQuote, args, 'UniformOutput', false);
command = ['install_name_tool', sprintf(' %s', quoted_args{:})];
[status, output] = system(command);
if status ~= 0
    error("copp:BuildError", ...
        "install_name_tool failed while preparing the dynamic MEX runtime:%s%s", ...
        newline, output);
end
end

function quoted = shellQuote(value)
text = char(value);
quoted = ['"', strrep(text, '"', '\"'), '"'];
end

function libs = nativeSystemLibraries(linkage)
libs = {};
if ~strcmp(linkage, 'static')
    return
end

if ispc
    libs = {'ws2_32.lib', 'userenv.lib', 'ntdll.lib', 'bcrypt.lib', 'advapi32.lib'};
elseif ismac
    libs = {'-ldl', '-lpthread', '-lm'};
else
    libs = {'-ldl', '-lpthread', '-lm'};
end
end
