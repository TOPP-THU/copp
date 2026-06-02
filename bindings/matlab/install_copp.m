function info = install_copp(opts)
%INSTALL_COPP Download and install a prebuilt COPP MATLAB toolbox.
%
% install_copp() detects the current MATLAB platform, downloads the matching
% .mltbx asset from the latest COPP GitHub Release, installs it, and verifies
% that the installed MATLAB package can report its version.
%
% install_copp(Version="v0.1.0") installs a specific release tag.
% install_copp(DryRun=true) prints the candidate toolbox locations without
% downloading or installing anything.
arguments
    opts.Version (1,1) string = "latest"
    opts.Repository (1,1) string = "TOPP-THU/copp"
    opts.AssetRoot (1,1) string = ""
    opts.Package (1,1) string {mustBeMember(opts.Package, ["auto", "copp", "copp"])} = "auto"
    opts.AgreeToLicense (1,1) logical = true
    opts.KeepDownload (1,1) logical = false
    opts.DryRun (1,1) logical = false
    opts.Timeout (1,1) double {mustBePositive} = 120
end

platform = coppMatlabPlatform();
locations = candidateToolboxLocations(opts.Repository, opts.AssetRoot, opts.Version, platform);

fprintf("COPP MATLAB platform: %s\n", platform);
fprintf("COPP MATLAB release: %s\n", opts.Version);
fprintf("Candidate toolbox locations:\n");
for i = 1:numel(locations)
    fprintf("  %s\n", locations(i));
end

if opts.DryRun
    info = struct( ...
        "Name", "COPP MATLAB dry run", ...
        "Version", char(opts.Version), ...
        "Guid", "", ...
        "Platform", char(platform), ...
        "ToolboxLocations", locations);
    return
end

toolboxFile = string(tempname) + ".mltbx";
cleanup = onCleanup(@() deleteIfRequested(toolboxFile, opts.KeepDownload));

sourceLocation = "";
lastError = [];
httpOptions = weboptions(Timeout=opts.Timeout);
for i = 1:numel(locations)
    try
        if isfile(locations(i))
            fprintf("Copying %s\n", locations(i));
            copyfile(locations(i), toolboxFile);
        elseif isRemoteLocation(locations(i))
            fprintf("Downloading %s\n", locations(i));
            websave(toolboxFile, locations(i), httpOptions);
        else
            error("copp:InstallError", "Candidate toolbox file does not exist: %s", locations(i));
        end
        sourceLocation = locations(i);
        break
    catch err
        lastError = err;
    end
end

if strlength(sourceLocation) == 0
    if isempty(lastError)
        error("copp:InstallError", "No COPP MATLAB toolbox locations were generated.");
    end
    error("copp:InstallError", ...
        "Could not find or download a COPP MATLAB toolbox for platform %s. Last error: %s", ...
        platform, lastError.message);
end

fprintf("Installing %s\n", toolboxFile);
info = matlab.addons.toolbox.installToolbox(toolboxFile, opts.AgreeToLicense);
rehash toolboxcache

versionText = verifyInstalledPackage(opts.Package);
fprintf("Installed COPP MATLAB package version: %s\n", versionText);
end

function platform = coppMatlabPlatform()
arch = lower(string(computer("arch")));
switch arch
    case {"win64", "pcwin64"}
        platform = "windows-x86_64";
    case {"glnxa64"}
        platform = "linux-x86_64";
    case {"maca64"}
        platform = "macos-aarch64";
    case {"maci64"}
        platform = "macos-x86_64";
    otherwise
        error("copp:InstallError", ...
            "Unsupported MATLAB platform architecture: %s.", arch);
end
end

function locations = candidateToolboxLocations(repository, assetRoot, version, platform)
assetStable = "copp-matlab-" + platform + ".mltbx";
locations = strings(0, 1);

if strlength(assetRoot) > 0
    if isfolder(assetRoot)
        locations(end + 1, 1) = fullfile(assetRoot, assetStable);
        if version ~= "latest"
            tag = normalizeReleaseTag(version);
            locations(end + 1, 1) = fullfile(assetRoot, "copp-matlab-" + tag + "-" + platform + ".mltbx");
        end
        platformAssets = dir(fullfile(assetRoot, "copp-matlab-*" + platform + ".mltbx"));
        for i = 1:numel(platformAssets)
            locations(end + 1, 1) = fullfile(platformAssets(i).folder, platformAssets(i).name);
        end
        locations = unique(locations, "stable");
    else
        locations(end + 1, 1) = joinUrl(assetRoot, assetStable);
        if version ~= "latest"
            tag = normalizeReleaseTag(version);
            locations(end + 1, 1) = joinUrl(assetRoot, "copp-matlab-" + tag + "-" + platform + ".mltbx");
        end
    end
    return
end

repoBase = "https://github.com/" + repository;
if version == "latest"
    locations(end + 1, 1) = repoBase + "/releases/latest/download/" + assetStable;
    tag = latestReleaseTag(repository);
    if strlength(tag) > 0
        locations(end + 1, 1) = repoBase + "/releases/download/" + tag + "/" + assetStable;
        locations(end + 1, 1) = repoBase + "/releases/download/" + tag + "/copp-matlab-" + tag + "-" + platform + ".mltbx";
    end
else
    tag = normalizeReleaseTag(version);
    locations(end + 1, 1) = repoBase + "/releases/download/" + tag + "/" + assetStable;
    locations(end + 1, 1) = repoBase + "/releases/download/" + tag + "/copp-matlab-" + tag + "-" + platform + ".mltbx";
end
end

function tag = latestReleaseTag(repository)
tag = "";
try
    httpOptions = weboptions(Timeout=30);
    data = webread("https://api.github.com/repos/" + repository + "/releases/latest", httpOptions);
    if isstruct(data) && isfield(data, "tag_name")
        tag = string(data.tag_name);
    end
catch
    % The stable latest/download URL can still work without GitHub API access.
end
end

function tag = normalizeReleaseTag(version)
tag = string(version);
if ~startsWith(tag, "v") && ~isempty(regexp(char(tag), "^[0-9]+(\.[0-9]+){1,3}([+-].*)?$", "once"))
    tag = "v" + tag;
end
end

function url = joinUrl(root, asset)
root = string(regexprep(char(root), "/+$", ""));
url = root + "/" + asset;
end

function result = isRemoteLocation(location)
location = lower(string(location));
result = startsWith(location, "https://") || startsWith(location, "http://");
end

function versionText = verifyInstalledPackage(packageName)
if packageName == "auto"
    packages = ["copp", "copp"];
else
    packages = packageName;
end

lastError = [];
for package = packages
    try
        versionText = string(eval(package + ".version()"));
        return
    catch err
        lastError = err;
    end
end

if isempty(lastError)
    error("copp:InstallError", "Could not verify the installed COPP MATLAB package.");
end
error("copp:InstallError", ...
    "Installed toolbox was not found on the MATLAB path. Last error: %s", ...
    lastError.message);
end

function deleteIfRequested(fileName, keepDownload)
if keepDownload || ~isfile(fileName)
    return
end
delete(fileName);
end
