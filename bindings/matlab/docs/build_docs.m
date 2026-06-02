function html_files = build_docs(opts)
%BUILD_DOCS Generate MATLAB documentation HTML pages.
%
% html_files = build_docs() publishes the MATLAB publish-style source pages
% in docs/source and then post-processes the generated HTML into a compact
% Help Center-style reference layout. The authored .m files stay diffable,
% runnable, and compatible with MATLAB publish in R2024b.
%
% Examples
%   build_docs
%   build_docs(OutputDir="docs/html", BuildSearchDatabase=true)

arguments
    opts.OutputDir (1,1) string = string(fullfile(fileparts(mfilename('fullpath')), 'html'))
    opts.BuildSearchDatabase (1,1) logical = false
    opts.ShowCode (1,1) logical = true
    opts.EvalCode (1,1) logical = true
    opts.MathRenderer (1,1) string = "mathjax-cdn"
end

docs_dir = fileparts(mfilename('fullpath'));
source_dir = fullfile(docs_dir, 'source');
if ~isfolder(source_dir)
    % Convenient for small local tests where build_docs.m and the source .m
    % files are kept in the same folder. In the toolbox tree, docs/source is
    % still the canonical source directory.
    source_dir = docs_dir;
end

matlab_dir = fileparts(docs_dir);
output_dir = char(opts.OutputDir);
validate_math_renderer(opts.MathRenderer);

if ~isfolder(output_dir)
    mkdir(output_dir);
end

pages = doc_pages();
api_entries = api_reference_entries(matlab_dir);
module_entries = api_module_entries(api_entries);
method_entries = api_method_link_entries(api_entries);
link_entries = [api_entries; module_entries; method_entries];
html_files = strings(numel(pages), 1);

addpath(matlab_dir);

for page_index = 1:numel(pages)
    page = pages(page_index);
    source_file = fullfile(source_dir, page.name + ".m");
    if ~isfile(source_file)
        error("copp:DocsSourceNotFound", ...
            "Documentation source file not found: %s", source_file);
    end

    matlab_dir_literal = matlab_string_literal(matlab_dir);
    source_file_literal = matlab_string_literal(source_file);
    show_code = opts.ShowCode && page.name ~= "api_reference";

    options = struct( ...
        'format', 'html', ...
        'outputDir', output_dir, ...
        'showCode', show_code, ...
        'evalCode', opts.EvalCode, ...
        'catchError', false, ...
        'codeToEvaluate', sprintf( ...
        'addpath(%s); rehash; run(%s);', ...
        matlab_dir_literal, source_file_literal), ...
        'imageFormat', 'png');

    html_files(page_index) = string(publish(source_file, options));
    apply_reference_theme(html_files(page_index), page, opts.MathRenderer, link_entries);
end

ref_files = write_api_help_pages(output_dir, api_entries, module_entries, link_entries, opts.MathRenderer);
html_files = [html_files; ref_files];

if opts.MathRenderer ~= "none"
    delete_equation_images(output_dir);
end

write_index_page(output_dir, pages, module_entries, link_entries, opts.MathRenderer);
write_helptoc(output_dir, pages, api_entries, module_entries);

if opts.BuildSearchDatabase
    builddocsearchdb(output_dir);
end

disp("Generated MATLAB documentation:");
disp(html_files);
fprintf("Documentation entry point: %s\n", fullfile(output_dir, 'index.html'));
end

function pages = doc_pages()
pages = struct( ...
    'name', { ...
    "overview", ...
    "path", ...
    "robot_constraints", ...
    "objectives_clarabel", ...
    "solvers_topp2_copp2", ...
    "solvers_topp3_copp3", ...
    "interpolation", ...
    "errors_diagnostics", ...
    "api_reference", ...
    "s_to_t_topp2"}, ...
    'title', { ...
    "COPP for MATLAB", ...
    "Path Module", ...
    "Robot and Constraints", ...
    "Objectives and Clarabel", ...
    "TOPP2 and COPP2 Solvers", ...
    "TOPP3 and COPP3 Solvers", ...
    "Interpolation", ...
    "Errors and Diagnostics", ...
    "API Reference", ...
    "s_to_t_topp2"}, ...
    'breadcrumb', { ...
    "Documentation Home / COPP / MATLAB", ...
    "Documentation Home / COPP / MATLAB / Path", ...
    "Documentation Home / COPP / MATLAB / Robot", ...
    "Documentation Home / COPP / MATLAB / Objectives", ...
    "Documentation Home / COPP / MATLAB / Solvers / Second Order", ...
    "Documentation Home / COPP / MATLAB / Solvers / Third Order", ...
    "Documentation Home / COPP / MATLAB / Interpolation", ...
    "Documentation Home / COPP / MATLAB / Diagnostics", ...
    "Documentation Home / COPP / MATLAB / Reference", ...
    "Documentation Home / COPP / MATLAB / Interpolation"}, ...
    'subtitle', { ...
    "Mathematical orientation, package layout, and a complete first workflow.", ...
    "Build geometric paths q(s) from waypoints, callbacks, symbolic formulas, CasADi, or Jet3 formulas.", ...
    "Store station grids, sampled derivatives, physical limits, raw inequalities, and inverse dynamics.", ...
    "Choose COPP objective terms and configure Clarabel-backed solvers.", ...
    "Use reachability and SOCP solvers for second-order path-parameterization problems.", ...
    "Use LP and SOCP solvers for third-order models.", ...
    "Convert path-domain profiles into time grids and sample s(t).", ...
    "Understand MATLAB exceptions, CoppError, solver statuses, verbosity, and last-error snapshots.", ...
    "Public classes, functions, solver namespaces, options, results, and profile helpers.", ...
    "Convert a TOPP2 path-speed profile into cumulative arrival times."});
end

function literal = matlab_string_literal(value)
escaped = strrep(char(value), '''', '''''');
literal = ['''', escaped, ''''];
end

function apply_reference_theme(html_file, page, math_renderer, link_entries)
html_file = char(html_file);
html = fileread(html_file);

html = remove_publish_source_listing(html);
if math_renderer ~= "none"
    [html, has_math] = replace_equation_images_with_mathjax(html);
    has_math = has_math || has_inline_mathjax(html);
    if has_math
        html = inject_mathjax(html, math_renderer);
    end
end
html = add_viewport_meta(html);
html = add_reference_theme_css(html);
html = rewrite_title(html, page);
html = rewrite_body_header(html);
html = rewrite_page_heading(html, page);
html = rewrite_contents_nav(html);
html = enhance_reference_sections(html);
html = enhance_code_blocks(html);
html = enhance_argument_blocks(html);
html = enhance_see_also(html);
html = enhance_footer(html);
html = link_api_references(html, link_entries, "root");

write_text_file(html_file, html);
end

function html = remove_publish_source_listing(html)
% MATLAB publish can append a full source listing in an HTML comment. Keep the
% published page focused on reference content and avoid huge diffs in HTML.
html = regexprep(html, ...
    '(?s)\s*<!--\s*##### SOURCE BEGIN #####.*?##### SOURCE END #####\s*-->\s*', ...
    newline);
end

function html = add_viewport_meta(html)
if isempty(regexp(html, '<meta\s+name="viewport"', 'once'))
    html = regexprep(html, '<head>', ...
        ['<head>', newline, ...
        '<meta name="viewport" content="width=device-width, initial-scale=1.0">'], ...
        'once');
end
end

function html = add_reference_theme_css(html)
css = reference_theme_css();
html = regexprep(html, '</style>', [css, newline, '</style>'], 'once');
end

function html = rewrite_title(html, page)
html = regexprep(html, ...
    '<title>.*?</title>', ...
    ['<title>', html_escape(page.title), ' - COPP MATLAB Reference</title>'], ...
    'once');
end

function html = rewrite_body_header(html)
header_html = [ ...
    '<body class="mw-doc-body">', newline, ...
    '<a class="skip-link" href="#mw-main-content">Skip to content</a>', newline, ...
    '<header class="mw-site-header" role="banner">', newline, ...
    '  <div class="mw-site-header-inner">', newline, ...
    '    <div class="mw-brand-block">', newline, ...
    '      <div class="mw-brand-title">COPP Help Center</div>', newline, ...
    '      <div class="mw-brand-subtitle">MATLAB Reference</div>', newline, ...
    '    </div>', newline, ...
    '    <div class="mw-search-box" aria-hidden="true">Search Help Center</div>', newline, ...
    '  </div>', newline, ...
    '</header>', newline];
html = regexprep(html, '<body[^>]*>', header_html, 'once');
end

function html = rewrite_page_heading(html, page)
heading_html = [ ...
    '<div class="mw-page-head" id="mw-main-content" tabindex="-1">', newline, ...
    breadcrumb_html(page.breadcrumb), newline, ...
    '<h1>', html_escape(page.title), '</h1>', newline, ...
    '<div class="function-label">', html_escape(page.subtitle), '</div>', newline, ...
    '</div>'];
html = regexprep(html, '<h1>.*?</h1>', heading_html, 'once');
end

function html = breadcrumb_html(breadcrumb)
parts = string(strsplit(char(breadcrumb), ' / '));
items = strings(1, 0);
for k = 1:numel(parts)
    if k > 1
        items(end+1) = "<span class=""breadcrumb-separator"">/</span>"; %#ok<AGROW>
    end
    if k == numel(parts)
        items(end+1) = "<span class=""breadcrumb-current"">" + html_escape(parts(k)) + "</span>"; %#ok<AGROW>
    else
        items(end+1) = "<span>" + html_escape(parts(k)) + "</span>"; %#ok<AGROW>
    end
end
html = char("<nav class=""breadcrumb"" aria-label=""Breadcrumb"">" + strjoin(items, "") + "</nav>");
end

function html = rewrite_contents_nav(html)
% MATLAB publish emits a Contents section before the first h2. Convert it to
% the Help Center-like "On this page" rail without changing source comments.
if ~isempty(regexp(html, '<h2>Contents</h2>\s*<div>', 'once'))
    html = regexprep(html, ...
        '<h2>Contents</h2>\s*<div>', ...
        ['<nav class="on-page" aria-labelledby="on-page-title">', newline, ...
        '<div class="on-page-title" id="on-page-title">On this page</div>', newline, ...
        '<div class="on-page-list">'], ...
        'once');
    html = regexprep(html, ...
        '</ul>\s*</div>\s*(<h2\s+id="[^"]+">)', ...
        ['</ul>', newline, '</div>', newline, '</nav>', newline, '$1'], ...
        'once');
else
    html = insert_generated_on_page_nav(html);
end
end

function html = insert_generated_on_page_nav(html)
[tokens, ~] = regexp(html, '<h2\s+id="([^"]+)">\s*(.*?)\s*</h2>', 'tokens', 'match');
if isempty(tokens)
    return
end

items = strings(1, 0);
for k = 1:numel(tokens)
    target = html_escape(tokens{k}{1});
    label = strip_html(tokens{k}{2});
    items(end+1) = "<li><a href=""#" + target + """>" + html_escape(label) + "</a></li>"; %#ok<AGROW>
end

nav_html = char(strjoin([ ...
    "<nav class=""on-page"" aria-labelledby=""on-page-title"">", ...
    "<div class=""on-page-title"" id=""on-page-title"">On this page</div>", ...
    "<div class=""on-page-list""><ul>", ...
    strjoin(items, ""), ...
    "</ul></div></nav>"], newline));

html = regexprep(html, '</div>\s*(<h2\s+id="[^"]+">)', ['</div>', newline, nav_html, newline, '$1'], 'once');
end

function html = enhance_reference_sections(html)
% Add stable classes to the canonical sections generated from publish-style
% headings. CSS then controls section spacing and Help Center-like rhythm.
section_names = [ ...
    "Syntax", "Description", "Examples", "Input Arguments", ...
    "Name-Value Arguments", "Output Arguments", "More About", ...
    "Tips", "Extended Capabilities", "Version History", "See Also"];
for k = 1:numel(section_names)
    name = section_names(k);
    slug = lower(regexprep(char(name), '[^a-zA-Z0-9]+', '-'));
    slug = regexprep(slug, '(^-|-$)', '');
    pattern = ['<h2\s+id="([^"]+)">\s*', regexptranslate('escape', char(name)), '\s*</h2>'];
    replacement = ['<h2 id="$1" class="ref-section ref-section-', slug, '">', char(name), '</h2>'];
    html = regexprep(html, pattern, replacement);
end
end

function html = enhance_code_blocks(html)
% Syntax blocks from comments use class="language-matlab". Example code uses
% class="codeinput" and captured output uses class="codeoutput".
html = enhance_syntax_blocks(html);

html = regexprep(html, ...
    '(?s)<pre class="codeinput">(.*?)</pre>', ...
    '<div class="code-example"><div class="code-toolbar"><span>MATLAB</span></div><pre class="codeinput">$1</pre></div>');

html = regexprep(html, ...
    '(?s)<pre class="codeoutput">(.*?)</pre>', ...
    '<div class="code-output"><div class="code-toolbar code-output-toolbar"><span>Output</span></div><pre class="codeoutput">$1</pre></div>');
end

function html = enhance_syntax_blocks(html)
[matches, tokens] = regexp(html, ...
    '(?s)(<h2\s+id="[^"]+"\s+class="ref-section ref-section-syntax">\s*Syntax\s*</h2>\s*)<pre class="language-matlab">(.*?)</pre>', ...
    'match', 'tokens');
for k = 1:numel(matches)
    heading = tokens{k}{1};
    code_html = regexprep(char(tokens{k}{2}), '<br\s*/?>', newline, 'ignorecase');
    code = html_unescape(regexprep(code_html, '<[^>]+>', ''));
    syntax_lines = strip(splitlines(string(code)));
    syntax_lines(syntax_lines == "") = [];
    if isempty(syntax_lines)
        replacement = [heading, '<div class="ref-syntax"><pre class="language-matlab">', tokens{k}{2}, '</pre></div>'];
    else
        items = strings(numel(syntax_lines), 1);
        blocks = strings(numel(syntax_lines), 1);
        for line_index = 1:numel(syntax_lines)
            anchor = "syntax-" + line_index;
            items(line_index) = "<li><a href=""#" + anchor + """><code>" + ...
                string(html_escape(syntax_lines(line_index))) + "</code></a></li>";
            blocks(line_index) = "<pre id=""" + anchor + """ class=""language-matlab"">" + ...
                string(html_escape(syntax_lines(line_index))) + "</pre>";
        end
        replacement = char(strjoin([
            string(heading)
            "<div class=""ref-syntax syntax-list""><ul>"
            items
            "</ul></div>"
            blocks], newline));
    end
    html = strrep(html, matches{k}, replacement);
end
end

function html = enhance_argument_blocks(html)
% MATLAB publish turns *|arg| - summary* into <p><b><tt>arg</tt> - summary</b></p>.
% Convert that compact authoring idiom into Reference-page-style argument
% headings. The source .m files remain simple, diffable publish documents.
html = regexprep(html, ...
    '<p>\s*<b>\s*<tt>(.*?)</tt>\s*[-–—]\s*(.*?)\s*</b>\s*</p>', ...
    '<h3 class="argument-name"><tt>$1</tt><span class="argument-dash"> — </span><span class="argument-summary">$2</span></h3>');

html = regexprep(html, ...
    '<p>\s*(Data Types:\s*.*?)\s*</p>', ...
    '<p class="argument-meta">$1</p>');

html = regexprep(html, ...
    '<p>\s*(Default:\s*.*?)\s*</p>', ...
    '<p class="argument-meta">$1</p>');

html = regexprep(html, ...
    '<p>\s*(Example:\s*.*?)\s*</p>', ...
    '<p class="argument-example">$1</p>');
end

function html = enhance_see_also(html)
html = regexprep(html, ...
    '(?s)(<h2\s+id="[^"]+"\s+class="ref-section ref-section-see-also">\s*See Also\s*</h2>\s*)<p>(.*?)</p>', ...
    '$1<div class="see-also-list"><p>$2</p></div>', ...
    'once');
end

function html = enhance_footer(html)
html = regexprep(html, '<p class="footer">', '<footer class="footer"><p>', 'once');
html = regexprep(html, '</p>\s*</div>\s*</body>', '</p></footer></div></body>', 'once');
end

function css = reference_theme_css()
lines = [
    ""
    "/* COPP MATLAB reference theme. Designed for MATLAB publish HTML from R2024b. */"
    ":root { --mw-bg:#ffffff; --mw-page-bg:#f7f7f7; --mw-text:#333333; --mw-muted:#666666; --mw-border:#d6d6d6; --mw-border-light:#e5e5e5; --mw-link:#0076a8; --mw-link-hover:#005f87; --mw-code-bg:#f5f5f5; --mw-code-border:#dcdcdc; --mw-table-head:#f3f3f3; --mw-accent:#c45400; }"
    "html { min-height:100%; background:var(--mw-page-bg); }"
    "html body.mw-doc-body, html body { margin:0; background:var(--mw-page-bg); color:var(--mw-text); font-family:Arial, Helvetica, sans-serif; font-size:14px; line-height:1.55; -webkit-font-smoothing:antialiased; text-rendering:optimizeLegibility; }"
    "body, body * { box-sizing:border-box; }"
    ".skip-link { position:absolute; left:-9999px; top:auto; width:1px; height:1px; overflow:hidden; }"
    ".skip-link:focus { left:16px; top:12px; width:auto; height:auto; z-index:1000; padding:7px 10px; background:#fff; border:1px solid var(--mw-border); color:var(--mw-link); }"
    ".mw-site-header { background:#f2f2f2; border-top:4px solid var(--mw-accent); border-bottom:1px solid #cfcfcf; color:#333; }"
    ".mw-site-header-inner { max-width:1180px; min-height:54px; margin:0 auto; padding:0 32px; display:flex; align-items:center; justify-content:space-between; gap:24px; }"
    ".mw-brand-block { display:flex; align-items:baseline; gap:12px; min-width:0; }"
    ".mw-brand-title { color:#222; font-size:18px; font-weight:400; white-space:nowrap; }"
    ".mw-brand-subtitle { color:#666; font-size:13px; white-space:nowrap; }"
    ".mw-search-box { width:268px; min-height:31px; padding:6px 34px 6px 11px; border:1px solid #bdbdbd; border-radius:2px; background:#fff; color:#777; font-size:13px; line-height:17px; text-align:left; }"
    ".content { max-width:1180px; min-height:calc(100vh - 59px); margin:0 auto; padding:28px 34px 52px; background:var(--mw-bg); border-left:1px solid #eeeeee; border-right:1px solid #eeeeee; }"
    ".mw-page-head { max-width:860px; outline:none; }"
    ".breadcrumb { display:flex; flex-wrap:wrap; align-items:center; gap:0; margin:0 0 19px; color:#6f6f6f; font-size:12px; line-height:1.35; }"
    ".breadcrumb span { display:inline-block; }"
    ".breadcrumb-separator { margin:0 7px; color:#a0a0a0; }"
    ".breadcrumb-current { color:#555; }"
    "h1 { margin:0 0 4px; padding:0; color:#333; font-size:32px; font-weight:400; line-height:1.15; letter-spacing:-0.01em; }"
    ".function-label { max-width:850px; margin:0 0 23px; color:#555; font-size:15px; line-height:1.45; }"
    "h2 { clear:left; max-width:850px; margin:31px 0 14px; padding:23px 0 7px; border-bottom:1px solid var(--mw-border); color:#333; font-size:24px; font-weight:400; line-height:1.25; }"
    "h2.ref-section-syntax { margin-top:24px; }"
    "h3 { max-width:850px; margin:22px 0 8px; padding:0; color:#333; font-size:17px; font-weight:600; line-height:1.35; }"
    "h4 { max-width:850px; margin:18px 0 7px; padding:0; color:#333; font-size:14px; font-weight:600; line-height:1.35; }"
    "p { max-width:850px; margin:0 0 14px; color:#333; font-size:14px; line-height:1.55; }"
    "a { color:var(--mw-link); text-decoration:none; }"
    "a:hover, a:focus { color:var(--mw-link-hover); text-decoration:underline; }"
    "a:visited { color:#005f87; }"
    "ul, ol { max-width:850px; margin:0 0 18px 23px; padding:0; }"
    "ul { list-style:square; }"
    "ol { list-style:decimal; }"
    "li { margin:4px 0 6px; padding:0; line-height:1.5; }"
    "li ul, li ol { margin-top:6px; margin-bottom:7px; }"
    "tt, code, pre { font-family:Consolas, 'Courier New', Courier, monospace; }"
    "tt, code { color:#222; font-size:.95em; background:var(--mw-code-bg); border:1px solid var(--mw-code-border); border-radius:3px; padding:1px 4px; }"
    "pre code, pre tt, .ref-syntax code, .syntax-list code { background:transparent; border:0; border-radius:0; padding:0; }"
    ".ref-syntax { max-width:850px; margin:0 0 19px; }"
    ".ref-syntax pre.language-matlab { margin:0; padding:0; border:0; background:#fff; color:#111; font-size:14px; line-height:1.8; white-space:pre-wrap; overflow:visible; }"
    "pre.language-matlab { max-width:850px; margin:8px 0 18px; padding:10px 12px; border:1px solid var(--mw-code-border); background:var(--mw-code-bg); color:#222; font-size:13px; line-height:1.45; overflow:auto; }"
    ".code-example, .code-output { max-width:850px; margin:10px 0 18px; border:1px solid var(--mw-code-border); background:var(--mw-code-bg); }"
    ".code-output { margin-top:-10px; background:#fff; }"
    ".code-toolbar { min-height:28px; padding:5px 10px 4px; border-bottom:1px solid var(--mw-code-border); background:#eeeeee; color:#666; font-size:11px; line-height:18px; text-transform:uppercase; letter-spacing:.03em; }"
    ".code-output-toolbar { background:#fafafa; color:#777; }"
    "pre.codeinput, pre.codeoutput { max-width:none; margin:0; border:0; border-radius:0; font-size:13px; line-height:1.48; overflow:auto; }"
    "pre.codeinput { padding:11px 12px 12px; background:var(--mw-code-bg); color:#222; }"
    "pre.codeoutput { padding:10px 12px 12px; background:#fff; color:#4c4c4c; }"
    "pre.error { max-width:850px; padding:10px 12px; border:1px solid #d9534f; background:#fff4f4; color:#b00000; }"
    "span.keyword { color:#0000ff; }"
    "span.comment { color:#228b22; }"
    "span.string { color:#a020f0; }"
    "span.untermstring { color:#b20000; }"
    "span.syscmd { color:#b28c00; }"
    "span.typesection { color:#a0522d; }"
    ".argument-name { max-width:850px; margin:22px 0 5px; padding:0; border:0; color:#333; font-size:17px; font-weight:600; line-height:1.35; }"
    ".argument-name tt { font-size:1em; font-weight:600; color:#111; }"
    ".argument-dash { color:#777; font-weight:400; }"
    ".argument-summary { color:#333; font-weight:400; }"
    ".argument-meta, .argument-example { max-width:850px; margin:-2px 0 13px; color:#666; font-size:13px; line-height:1.45; }"
    ".argument-meta tt, .argument-example tt { color:#333; }"
    ".syntax-list { max-width:850px; margin:0 0 12px; }"
    ".syntax-list ul { margin:0 0 12px 19px; }"
    ".syntax-list code { color:var(--mw-link); }"
    ".api-grid { max-width:920px; display:grid; grid-template-columns:repeat(auto-fit, minmax(250px, 1fr)); gap:10px; margin:8px 0 22px; }"
    ".api-card { min-width:0; padding:11px 13px 12px; border:1px solid var(--mw-border); background:#fff; }"
    ".api-card h3 { margin:0 0 5px; font-size:15px; font-weight:600; line-height:1.35; }"
    ".api-card p { margin:0; color:#555; font-size:13px; line-height:1.45; }"
    ".api-kind { margin:0 0 5px; color:#777; font-size:11px; line-height:1.2; text-transform:uppercase; letter-spacing:.03em; }"
    ".api-link code, a.api-link { color:var(--mw-link); }"
    ".api-related { max-width:850px; columns:2; column-gap:28px; }"
    ".api-description { max-width:850px; margin:0 0 16px; }"
    ".api-description p { margin:0 0 12px; }"
    ".api-detail { max-width:920px; margin:0 0 24px; padding:0 0 6px; border-bottom:1px solid var(--mw-border-light); }"
    ".api-detail h3 { margin-top:18px; }"
    ".argument-list { max-width:920px; display:grid; grid-template-columns:repeat(auto-fit, minmax(260px, 1fr)); gap:10px; margin:8px 0 22px; }"
    ".argument-item { min-width:0; padding:10px 12px 11px; border-left:3px solid var(--mw-border); background:#fafafa; }"
    ".argument-item h3 { margin:0 0 5px; font-size:15px; }"
    ".argument-item p { margin:0; color:#555; font-size:13px; line-height:1.45; }"
    ".argument-default { display:block; margin-top:5px; color:#666; }"
    ".on-page { float:right; position:sticky; top:16px; width:270px; margin:0 0 24px 42px; padding:13px 15px 14px; border:1px solid var(--mw-border); background:#f7f7f7; color:#333; }"
    ".on-page-title { margin:0 0 8px; color:#333; font-size:13px; font-weight:600; line-height:1.35; }"
    ".on-page-list ul { margin:0; padding:0 0 0 17px; list-style:square; }"
    ".on-page-list li { margin:4px 0; color:#777; font-size:13px; line-height:1.35; }"
    ".on-page-list a { color:var(--mw-link); }"
    "table { max-width:850px; width:auto; margin:10px 0 22px; border-collapse:collapse; border-spacing:0; font-size:13px; line-height:1.45; }"
    "table th, table td { padding:7px 9px; border:1px solid var(--mw-border); text-align:left; vertical-align:top; }"
    "table th { background:var(--mw-table-head); color:#333; font-weight:600; }"
    "table tr:nth-child(even) td { background:#fbfbfb; }"
    "img { max-width:100%; height:auto; border:0; }"
    "p img { display:inline-block; margin:2px 0 5px; vertical-align:middle; }"
    ".mathjax-display { display:block; max-width:850px; margin:2px 0 16px; overflow-x:auto; overflow-y:hidden; }"
    ".MathJax { font-size:100% !important; }"
    ".see-also-list { max-width:850px; margin:3px 0 20px; }"
    ".see-also-list p { margin:0; line-height:2.05; }"
    ".see-also-list tt { display:inline-block; margin:0 4px 7px 0; padding:2px 7px; border:1px solid #d7e7ef; border-radius:2px; background:#f2f8fb; color:var(--mw-link); font-size:13px; line-height:1.5; }"
    ".footer { clear:both; max-width:850px; margin:36px 0 0; padding:14px 0 0; border-top:1px solid var(--mw-border); color:#777; font-size:12px; font-style:normal; line-height:1.45; }"
    ".footer p { margin:0; color:#777; font-size:12px; }"
    ".footer a, .footer a:visited { color:#777; }"
    "@media (max-width: 1080px) { .on-page { float:none; position:static; width:auto; max-width:850px; margin:0 0 24px; } .mw-page-head, h2, h3, p, ul, ol, table, .ref-syntax, .code-example, .code-output, .see-also-list, .footer { max-width:100%; } }"
    "@media (max-width: 760px) { .mw-site-header-inner { min-height:auto; padding:10px 18px; align-items:flex-start; flex-direction:column; gap:8px; } .mw-brand-block { display:block; } .mw-brand-title { font-size:17px; } .mw-brand-subtitle { margin-top:1px; } .mw-search-box { display:none; } .content { padding:22px 18px 38px; border-left:0; border-right:0; } h1 { font-size:28px; } h2 { font-size:21px; margin-top:27px; } h3, .argument-name { font-size:16px; } pre.codeinput, pre.codeoutput, pre.language-matlab { font-size:12px; } }"
    "@media print { html body.mw-doc-body, html body { background:#fff; } .mw-site-header, .on-page, .skip-link { display:none; } .content { max-width:none; min-height:0; padding:0; border:0; } pre.codeinput, pre.codeoutput, pre.language-matlab { white-space:pre-wrap; word-wrap:break-word; overflow:visible; } a { color:#000; text-decoration:none; } }"
    ];
css = char(strjoin(lines, newline));
end

function text = strip_html(html)
text = regexprep(char(html), '<[^>]+>', '');
text = regexprep(text, '\s+', ' ');
text = strtrim(text);
end

function text = html_escape(value)
text = char(value);
text = strrep(text, '&', '&amp;');
text = strrep(text, '<', '&lt;');
text = strrep(text, '>', '&gt;');
text = strrep(text, '"', '&quot;');
text = strrep(text, '''', '&#39;');
end

function text = xml_escape(value)
text = html_escape(value);
end

function write_text_file(path, text)
[fid, message] = fopen(path, 'w', 'n', 'UTF-8');
if fid < 0
    error("copp:DocsWriteError", "Could not write %s: %s", path, message);
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '%s', text);
end

function validate_math_renderer(math_renderer)
if ~ismember(math_renderer, ["mathjax-cdn", "none"])
    error("copp:DocsError", ...
        "Unsupported MathRenderer value '%s'. Use 'mathjax-cdn' or 'none'.", ...
        math_renderer);
end
end

function [html, has_math] = replace_equation_images_with_mathjax(html)
has_math = false;
pattern = '(?:<p>\s*)?<img\s+[^>]*?alt="(\$\$[^"]*?\$\$)"[^>]*?>(?:\s*</p>)?';
[matches, tokens] = regexp(html, pattern, 'match', 'tokens');

for k = 1:numel(matches)
    latex = strtrim(char(html_unescape(tokens{k}{1})));
    if numel(latex) < 4 || ~startsWith(string(latex), "$$") || ~endsWith(string(latex), "$$")
        continue
    end

    math_body = strtrim(latex(3:end-2));
    replacement = ['<div class="mathjax-display">\[', html_escape(math_body), '\]</div>'];
    html = strrep(html, matches{k}, replacement);
    has_math = true;
end
end

function has_math = has_inline_mathjax(html)
has_math = any(contains(string(html), "\(") | contains(string(html), "\)"));
end

function script = mathjax_script_block(math_renderer)
validate_math_renderer(math_renderer);
if math_renderer == "none"
    script = "";
    return
end

bs = string(char(92));
mathjax_config = "window.MathJax = { tex: { displayMath: [['" + ...
    bs + bs + "[','" + bs + bs + "]']], inlineMath: [['" + ...
    bs + bs + "(','" + bs + bs + ")']], processEscapes: true }, chtml: { matchFontHeight: false } };";
mathjax_lines = [
    "<script>"
    mathjax_config
    "</script>"
    "<script defer src=""https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-chtml.js""></script>"
    ];
script = strjoin(mathjax_lines, newline);
end

function html = inject_mathjax(html, math_renderer)
script = mathjax_script_block(math_renderer);
if script == ""
    return
end

mathjax_lines = string(script);
insert_html = [char(strjoin(mathjax_lines, newline)), newline];
head_idx = regexp(html, '</head>', 'once');
if isempty(head_idx)
    html = [insert_html, html];
else
    html = [html(1:head_idx-1), insert_html, html(head_idx:end)];
end
end

function text = html_unescape(text)
text = string(text);
text = replace(text, "&#10;", newline);
text = replace(text, "&#13;", newline);
text = replace(text, "&quot;", string(char(34)));
text = replace(text, "&apos;", string(char(39)));
text = replace(text, "&lt;", "<");
text = replace(text, "&gt;", ">");
text = replace(text, "&amp;", "&");
end

function delete_equation_images(output_dir)
files = dir(fullfile(output_dir, '*_eq*.png'));
for k = 1:numel(files)
    delete(fullfile(files(k).folder, files(k).name));
end
end

function entries = api_reference_entries(matlab_dir)
package_root = fullfile(matlab_dir, '+copp');
files = dir(fullfile(package_root, '**', '*.m'));
entry_template = struct( ...
    'name', "", ...
    'target', "", ...
    'category', "", ...
    'source', "", ...
    'module', "", ...
    'kind', "", ...
    'summary', "");
entries = repmat(entry_template, numel(files), 1);
entry_count = 0;

for k = 1:numel(files)
    file_name = string(files(k).name);
    if file_name == "Contents.m"
        continue
    end

    folder_parts = string(strsplit(files(k).folder, filesep));
    if any(folder_parts == "+internal")
        continue
    end

    full_path = string(fullfile(files(k).folder, files(k).name));
    rel_path = extractAfter(full_path, string(package_root) + filesep);
    parts = split(rel_path, filesep);

    api_parts = "copp";
    for part_index = 1:numel(parts)
        part = parts(part_index);
        if startsWith(part, "+")
            api_parts(end+1) = extractAfter(part, 1); %#ok<AGROW>
        else
            [~, base_name] = fileparts(part);
            api_parts(end+1) = string(base_name); %#ok<AGROW>
        end
    end

    api_name = strjoin(api_parts, ".");
    entry_count = entry_count + 1;
    kind = api_kind(full_path);
    entries(entry_count) = struct( ...
        'name', api_name, ...
        'target', api_target(api_name), ...
        'category', api_category(api_name), ...
        'source', full_path, ...
        'module', api_module(api_name), ...
        'kind', kind, ...
        'summary', "");
end

entries = entries(1:entry_count);
if ~isempty(entries)
    [~, order] = sort(lower(string({entries.name})));
    entries = entries(order);
end
end

function module = api_module(api_name)
parts = split(string(api_name), ".");
if numel(parts) <= 1
    module = "";
else
    module = strjoin(parts(1:end-1), ".");
end
end

function kind = api_kind(source_file)
text = fileread(char(source_file));
if ~isempty(regexp(text, '(?m)^\s*classdef(\s|\(|$)', 'once'))
    kind = "class";
else
    kind = "function";
end
end

function target = api_target(api_name)
safe_name = regexprep(char(api_name), '[^A-Za-z0-9_.-]', '_');
target = string("ref/" + safe_name + ".html");
end

function category = api_category(api_name)
name = string(api_name);
if contains(name, ".interpolation.")
    category = "Interpolation";
elseif contains(name, ".objective.")
    category = "Objective Descriptors";
elseif contains(name, ".solver.topp2_") || contains(name, ".solver.reach_set2") || ...
        contains(name, ".solver.copp2_")
    category = "Second-Order Solvers";
elseif contains(name, ".solver.")
    category = "Third-Order Solvers";
elseif contains(name, ".clarabel.") || contains(name, ".diag.") || ...
        contains(name, ".Clarabel") || contains(name, ".Verbosity") || ...
        contains(name, ".CoppError") || contains(name, ".is_copp_error") || ...
        contains(name, ".last_error")
    category = "Options and Diagnostics";
else
    category = "Core Package";
end
end

function summary = module_summary(module_name)
name = string(module_name);
switch name
    case "copp"
        summary = "Core MATLAB package objects and utilities.";
    case "copp.clarabel"
        summary = "Clarabel solver settings and option adapters.";
    case "copp.diag"
        summary = "Diagnostics, error objects, and verbosity helpers.";
    case "copp.interpolation"
        summary = "Convert solver profiles between station and time domains.";
    case "copp.objective"
        summary = "Objective descriptors used by COPP solvers.";
    case "copp.solver"
        summary = "Second- and third-order solver namespaces.";
    case "copp.solver.topp2_ra"
        summary = "TOPP2 reachability-analysis solver and interpolation wrappers.";
    case "copp.solver.copp2_socp"
        summary = "Second-order convex optimization solver backed by Clarabel.";
    case "copp.solver.reach_set2"
        summary = "Second-order reach-set construction tools.";
    case "copp.solver.topp3"
        summary = "Shared third-order TOPP problem helpers.";
    case "copp.solver.topp3_lp"
        summary = "Third-order linear-programming solver backed by Clarabel.";
    case "copp.solver.topp3_socp"
        summary = "Third-order SOCP solver backed by Clarabel.";
    case "copp.solver.copp3_socp"
        summary = "Third-order COPP SOCP solver backed by Clarabel.";
    otherwise
        summary = "MATLAB API namespace.";
end
end

function modules = api_module_entries(api_entries)
if isempty(api_entries)
    modules = struct('name', {}, 'target', {}, 'category', {}, 'source', {}, ...
        'module', {}, 'kind', {}, 'summary', {});
    return
end

raw_module_names = unique(string({api_entries.module}));
raw_module_names(raw_module_names == "") = [];
module_names = strings(0, 1);
for raw_index = 1:numel(raw_module_names)
    parts = split(raw_module_names(raw_index), ".");
    for part_index = 1:numel(parts)
        module_names(end+1, 1) = strjoin(parts(1:part_index), "."); %#ok<AGROW>
    end
end
module_names = unique(module_names);
module_names(module_names == "") = [];
modules = repmat(struct( ...
    'name', "", ...
    'target', "", ...
    'category', "", ...
    'source', "", ...
    'module', "", ...
    'kind', "module", ...
    'summary', ""), numel(module_names), 1);

for k = 1:numel(module_names)
    name = module_names(k);
    modules(k).name = name;
    modules(k).target = api_target(name);
    modules(k).category = api_category(name);
    modules(k).source = "";
    modules(k).module = api_module(name);
    modules(k).kind = "module";
    modules(k).summary = module_summary(name);
end

if ~isempty(modules)
    [~, order] = sort(lower(string({modules.name})));
    modules = modules(order);
end
end

function methods = api_method_link_entries(api_entries)
method_template = struct( ...
    'name', "", ...
    'target', "", ...
    'category', "", ...
    'source', "", ...
    'module', "", ...
    'kind', "method", ...
    'summary', "");
methods = repmat(method_template, 0, 1);

class_entries = api_entries(string({api_entries.kind}) == "class");
for entry_index = 1:numel(class_entries)
    entry = class_entries(entry_index);
    source_file = string(entry.source);
    if strlength(source_file) == 0 || ~isfile(source_file)
        continue
    end

    source_text = fileread(char(source_file));
    class_methods = parse_class_methods(source_text, entry.name);
    class_short_name = last_api_part(entry.name);
    for method_index = 1:numel(class_methods)
        method = class_methods(method_index);
        if string(method.name) == class_short_name
            continue
        end

        anchor = method_anchor(method.name);
        methods(end+1, 1) = struct( ...
            'name', string(entry.name) + "." + string(method.name), ...
            'target', string(entry.target) + "#" + anchor, ...
            'category', string(entry.category), ...
            'source', source_file, ...
            'module', string(entry.name), ...
            'kind', "method", ...
            'summary', string(method.summary)); %#ok<AGROW>
    end
end
end

function ref_files = write_api_help_pages(output_dir, api_entries, module_entries, link_entries, math_renderer)
ref_dir = fullfile(output_dir, 'ref');
if isfolder(ref_dir)
    old_files = dir(fullfile(ref_dir, '*.html'));
    for k = 1:numel(old_files)
        delete(fullfile(old_files(k).folder, old_files(k).name));
    end
else
    mkdir(ref_dir);
end

ref_files = strings(numel(api_entries) + numel(module_entries), 1);
help_texts = strings(numel(api_entries), 1);
for k = 1:numel(api_entries)
    help_text = clean_help_text(help(char(api_entries(k).name)), api_entries(k).name);
    if strlength(string(strtrim(help_text))) == 0
        help_text = sprintf('No MATLAB help text is available yet for %s.', api_entries(k).name);
    end
    api_entries(k).summary = help_summary(help_text, api_entries(k).name);
    help_texts(k) = string(help_text);
end

for k = 1:numel(api_entries)
    entry = api_entries(k);
    help_text = char(help_texts(k));

    target_file = fullfile(output_dir, strrep(char(entry.target), '/', filesep));
    write_text_file(target_file, api_help_html(entry, help_text, api_entries, module_entries, link_entries, math_renderer));
    ref_files(k) = string(target_file);
end

offset = numel(api_entries);
for k = 1:numel(module_entries)
    entry = module_entries(k);
    target_file = fullfile(output_dir, strrep(char(entry.target), '/', filesep));
    write_text_file(target_file, api_module_html(entry, api_entries, module_entries, link_entries));
    ref_files(offset + k) = string(target_file);
end
end

function help_text = clean_help_text(help_text, api_name)
lines = splitlines(string(help_text));
stripped = strip(lines);
doc_command = "doc " + string(api_name);
doc_index = find(stripped == doc_command, 1, 'last');

if ~isempty(doc_index)
    start_index = doc_index;
    if doc_index > 1 && (contains(stripped(doc_index - 1), string(api_name)) || ...
            contains(lower(stripped(doc_index - 1)), "documentation"))
        start_index = doc_index - 1;
    end
    lines(start_index:doc_index) = [];
end

while ~isempty(lines) && strip(lines(end)) == ""
    lines(end) = [];
end

help_text = char(strjoin(lines, newline));
end

function html = api_help_html(entry, help_text, api_entries, module_entries, link_entries, math_renderer)
title = char(entry.name);
subtitle = char(help_summary(help_text, entry.name));
css = [reference_theme_css(), newline, ...
    '.helptext { max-width:920px; margin:0 0 22px; padding:0; border:0; background:#fff; color:#222; font-family:Consolas, ''Courier New'', monospace; font-size:13px; line-height:1.5; white-space:pre-wrap; overflow:auto; }', newline, ...
    '.api-links { max-width:850px; margin:0 0 22px; color:#555; font-size:13px; }'];
detail = api_source_detail(entry, help_text);
syntax_html = api_syntax_html(detail.syntax);
members_html = api_members_html(detail, link_entries);
method_details_html = api_method_details_html(detail, link_entries);
description_html = api_description_html(entry, help_text, link_entries);
arguments_html = api_arguments_html(entry, detail, link_entries);
examples_html = api_examples_html(detail, link_entries);
related_html = api_related_html(entry, api_entries, module_entries);
mathjax_script = "";
math_html = [description_html, arguments_html, members_html, method_details_html, examples_html];
if math_renderer ~= "none" && has_inline_mathjax(math_html)
    mathjax_script = mathjax_script_block(math_renderer);
end

lines = [
    "<!doctype html>"
    "<html lang=""en"">"
    "<head>"
    "<meta charset=""UTF-8"">"
    "<meta name=""viewport"" content=""width=device-width, initial-scale=1.0"">"
    "<title>" + string(html_escape(title)) + " - COPP MATLAB Reference</title>"
    "<style>"
    string(css)
    "</style>"
    string(mathjax_script)
    "</head>"
    "<body class=""mw-doc-body"">"
    "<a class=""skip-link"" href=""#mw-main-content"">Skip to content</a>"
    "<header class=""mw-site-header"" role=""banner""><div class=""mw-site-header-inner""><div class=""mw-brand-block""><div class=""mw-brand-title"">COPP Help Center</div><div class=""mw-brand-subtitle"">MATLAB Reference</div></div><div class=""mw-search-box"" aria-hidden=""true"">Search Help Center</div></div></header>"
    "<main class=""content"">"
    "<div class=""mw-page-head"" id=""mw-main-content"" tabindex=""-1"">"
    "<nav class=""breadcrumb"" aria-label=""Breadcrumb""><span>Documentation Home</span><span class=""breadcrumb-separator"">/</span><span>COPP</span><span class=""breadcrumb-separator"">/</span><span>MATLAB</span><span class=""breadcrumb-separator"">/</span><span class=""breadcrumb-current"">API Reference</span></nav>"
    "<h1>" + string(html_escape(title)) + "</h1>"
    "<div class=""function-label"">" + string(html_escape(subtitle)) + "</div>"
    "</div>"
    "<div class=""api-links""><a href=""../api_reference.html"">API Reference</a> | <a href=""" + api_ref_href(entry.module, module_entries, "ref") + """>" + string(html_escape(entry.module)) + "</a> | <a href=""../index.html"">User Guide</a></div>"
    string(syntax_html)
    string(description_html)
    string(arguments_html)
    string(members_html)
    string(method_details_html)
    string(examples_html)
    string(related_html)
    "<footer class=""footer""><p>Generated from MATLAB help comments.</p></footer>"
    "</main>"
    "</body>"
    "</html>"
    ];
html = char(strjoin(lines, newline));
end

function html = api_module_html(module_entry, api_entries, module_entries, link_entries)
module_name = string(module_entry.name);
direct_entries = api_entries(string({api_entries.module}) == module_name);
child_modules = module_entries(string({module_entries.module}) == module_name);

css = reference_theme_css();
direct_html = api_entry_grid_html(direct_entries, "ref", module_name, link_entries);
children_html = api_entry_grid_html(child_modules, "ref", module_name, link_entries);
if strlength(direct_html) == 0
    direct_html = '<p>No direct public classes or functions are documented in this module.</p>';
end
if strlength(children_html) == 0
    children_html = '<p>No child modules.</p>';
end

lines = [
    "<!doctype html>"
    "<html lang=""en"">"
    "<head>"
    "<meta charset=""UTF-8"">"
    "<meta name=""viewport"" content=""width=device-width, initial-scale=1.0"">"
    "<title>" + module_name + " - COPP MATLAB Module</title>"
    "<style>"
    string(css)
    "</style>"
    "</head>"
    "<body class=""mw-doc-body"">"
    "<a class=""skip-link"" href=""#mw-main-content"">Skip to content</a>"
    "<header class=""mw-site-header"" role=""banner""><div class=""mw-site-header-inner""><div class=""mw-brand-block""><div class=""mw-brand-title"">COPP Help Center</div><div class=""mw-brand-subtitle"">MATLAB Reference</div></div><div class=""mw-search-box"" aria-hidden=""true"">Search Help Center</div></div></header>"
    "<main class=""content"">"
    "<div class=""mw-page-head"" id=""mw-main-content"" tabindex=""-1"">"
    "<nav class=""breadcrumb"" aria-label=""Breadcrumb""><span>Documentation Home</span><span class=""breadcrumb-separator"">/</span><span>COPP</span><span class=""breadcrumb-separator"">/</span><span>MATLAB</span><span class=""breadcrumb-separator"">/</span><span class=""breadcrumb-current"">Module</span></nav>"
    "<h1>" + module_name + "</h1>"
    "<div class=""function-label"">Classes, functions, and child namespaces in this MATLAB module.</div>"
    "</div>"
    "<div class=""api-links""><a href=""../api_reference.html"">API Reference</a> | <a href=""../index.html"">User Guide</a></div>"
    "<h2 id=""child-modules"">Child Modules</h2>"
    string(children_html)
    "<h2 id=""module-members"">Classes and Functions</h2>"
    string(direct_html)
    "<footer class=""footer""><p>Generated from the MATLAB package tree.</p></footer>"
    "</main>"
    "</body>"
    "</html>"
    ];
html = char(strjoin(lines, newline));
end

function detail = api_source_detail(entry, help_text)
detail = struct( ...
    'syntax', strings(0, 1), ...
    'properties', [], ...
    'methods', [], ...
    'inputs', [], ...
    'nameValues', [], ...
    'outputs', [], ...
    'examples', strings(0, 1));
source_file = string(entry.source);
if strlength(source_file) == 0 || ~isfile(source_file)
    detail.syntax = syntax_from_help(help_text);
    detail.outputs = parse_outputs_from_syntax(detail.syntax);
    return
end

source_text = fileread(char(source_file));
if entry.kind == "class"
    detail.syntax = syntax_from_help(help_text);
    detail.properties = parse_class_properties(source_text);
    detail.methods = parse_class_methods(source_text, entry.name);
    constructor_index = find(string({detail.methods.name}) == last_api_part(entry.name), 1);
    if ~isempty(constructor_index)
        constructor = detail.methods(constructor_index);
        if ~isempty(constructor.syntax)
            detail.syntax = constructor.syntax;
        end
        detail.inputs = constructor.inputs;
        detail.nameValues = constructor.nameValues;
        detail.syntax = expand_name_value_syntax(detail.syntax, detail.nameValues);
    end
    help_examples = extract_help_examples(help_text);
    default_examples = default_examples_for_api(entry.name);
    detail.examples = [help_examples; default_examples];
else
    detail.syntax = syntax_from_function_source(source_text, entry.name);
    [detail.inputs, detail.nameValues] = parse_function_arguments(source_text);
    detail.syntax = expand_name_value_syntax(detail.syntax, detail.nameValues);
    if isempty(detail.syntax)
        detail.syntax = syntax_from_help(help_text);
    end
    detail.outputs = parse_outputs_from_syntax(detail.syntax);
    detail.examples = extract_help_examples(help_text);
    default_examples = default_examples_for_api(entry.name);
    if ~isempty(default_examples)
        detail.examples = [detail.examples; default_examples];
    end
end
end

function syntax = syntax_from_help(help_text)
lines = splitlines(string(help_text));
syntax = strings(0, 1);
for k = 1:numel(lines)
    line = strip(lines(k));
    if line == ""
        continue
    end
    if contains(line, "(") && (contains(line, "=") || startsWith(line, upper(extractBefore(line + " ", " "))))
        if startsWith(line, "%")
            line = strip(extractAfter(line, 1));
        end
        syntax(end+1, 1) = line; %#ok<AGROW>
    end
    if numel(syntax) >= 5
        break
    end
end
end

function syntax = syntax_from_function_source(source_text, api_name)
syntax = strings(0, 1);
matches = regexp(source_text, '^\s*function\s+([^\r\n]+)', 'tokens', 'lineanchors');
if isempty(matches)
    return
end

line = string(strtrim(matches{1}{1}));
line = erase(line, " %#ok<STOUT>");
open_idx = strfind(char(line), '(');
close_idx = find(char(line) == ')', 1, 'last');
if isempty(open_idx) || isempty(close_idx) || close_idx <= open_idx(1)
    syntax = string(api_name);
    return
end

prefix = extractBefore(line, open_idx(1));
args = extractBetween(line, open_idx(1) + 1, close_idx - 1);
args = args(1);
eq_idx = strfind(char(prefix), '=');
if isempty(eq_idx)
    lhs = "";
else
    lhs = strip(extractBefore(prefix, eq_idx(end)));
end

if lhs == ""
    syntax = string(api_name) + "(" + args + ")";
else
    syntax = lhs + " = " + string(api_name) + "(" + args + ")";
end
end

function [inputs, name_values] = parse_function_arguments(source_text)
template = struct('name', "", 'summary', "", 'default', "");
inputs = repmat(template, 0, 1);
name_values = repmat(template, 0, 1);

lines = splitlines(string(source_text));
inside = false;
for k = 1:numel(lines)
    line = strip(lines(k));
    if startsWith(line, "arguments")
        inside = true;
        continue
    end
    if inside && line == "end"
        break
    end
    if ~inside || line == "" || startsWith(line, "%")
        continue
    end

    [arg, is_name_value] = parse_argument_declaration(line);
    if arg.name == ""
        continue
    end
    if is_name_value
        name_values(end+1) = arg; %#ok<AGROW>
    elseif string(arg.name) ~= "opts"
        inputs(end+1) = arg; %#ok<AGROW>
    end
end
end

function [arg, is_name_value] = parse_argument_declaration(line)
arg = struct('name', "", 'summary', "", 'default', "");
is_name_value = false;

default_value = "";
eq_index = strfind(char(line), '=');
if ~isempty(eq_index)
    default_value = string(strtrim(extractAfter(line, eq_index(1))));
end

dotted_tokens = regexp(char(line), '^([A-Za-z]\w*)\.([A-Za-z]\w*)', 'tokens', 'once');
if ~isempty(dotted_tokens)
    arg.name = string(dotted_tokens{2});
    is_name_value = true;
else
    tokens = regexp(char(line), '^([A-Za-z]\w*)', 'tokens', 'once');
    if isempty(tokens)
        return
    end
    arg.name = string(tokens{1});
end
arg.default = default_value;
end

function outputs = parse_outputs_from_syntax(syntax)
template = struct('name', "", 'summary', "", 'default', "");
outputs = repmat(template, 0, 1);
if isempty(syntax)
    return
end

line = string(syntax(1));
eq_index = strfind(char(line), '=');
if isempty(eq_index)
    return
end

lhs = strip(extractBefore(line, eq_index(1)));
lhs = erase(lhs, "[");
lhs = erase(lhs, "]");
parts = strip(split(lhs, ","));
parts(parts == "") = [];
for k = 1:numel(parts)
    outputs(end+1) = struct('name', parts(k), 'summary', "", 'default', ""); %#ok<AGROW>
end
end

function syntax = expand_name_value_syntax(syntax, name_values)
if isempty(syntax) || isempty(name_values)
    return
end

expanded = strings(0, 1);
for k = 1:numel(syntax)
    line = string(syntax(k));
    if contains(line, ", opts)")
        expanded(end+1, 1) = replace(line, ", opts)", ")"); %#ok<AGROW>
        placeholders = strings(numel(name_values), 1);
        for nv_index = 1:numel(name_values)
            name = string(name_values(nv_index).name);
            placeholders(nv_index) = name + "=" + name;
        end
        expanded(end+1, 1) = replace(line, "opts)", strjoin(placeholders, ", ") + ")"); %#ok<AGROW>
    elseif contains(line, "(opts)")
        expanded(end+1, 1) = replace(line, "(opts)", "()"); %#ok<AGROW>
        placeholders = strings(numel(name_values), 1);
        for nv_index = 1:numel(name_values)
            name = string(name_values(nv_index).name);
            placeholders(nv_index) = name + "=" + name;
        end
        if ~isempty(placeholders)
            expanded(end+1, 1) = replace(line, "opts)", strjoin(placeholders, ", ") + ")"); %#ok<AGROW>
        end
    elseif contains(line, "opts)")
        expanded(end+1, 1) = replace(line, "opts)", ")"); %#ok<AGROW>
    else
        expanded(end+1, 1) = line; %#ok<AGROW>
    end
end
syntax = unique(expanded, 'stable');
end

function html = api_syntax_html(syntax)
if isempty(syntax)
    html = "";
    return
end

items = strings(numel(syntax), 1);
for k = 1:numel(syntax)
    code = html_escape(syntax(k));
    items(k) = "<li><a href=""#syntax-" + k + """><code>" + string(code) + "</code></a></li>";
end

blocks = strings(numel(syntax), 1);
for k = 1:numel(syntax)
    blocks(k) = "<pre id=""syntax-" + k + """ class=""language-matlab"">" + ...
        string(html_escape(syntax(k))) + "</pre>";
end

html = char(strjoin([
    "<h2 id=""syntax"">Syntax</h2>"
    "<div class=""syntax-list""><ul>"
    items
    "</ul></div>"
    blocks], newline));
end

function html = api_syntax_html_with_heading(syntax, heading, prefix)
if isempty(syntax)
    html = "";
    return
end

items = strings(numel(syntax), 1);
blocks = strings(numel(syntax), 1);
for k = 1:numel(syntax)
    anchor = prefix + "-" + k;
    items(k) = "<li><a href=""#" + anchor + """><code>" + ...
        string(html_escape(syntax(k))) + "</code></a></li>";
    blocks(k) = "<pre id=""" + anchor + """ class=""language-matlab"">" + ...
        string(html_escape(syntax(k))) + "</pre>";
end

html = char(strjoin([
    "<h4>" + string(html_escape(heading)) + "</h4>"
    "<div class=""syntax-list""><ul>"
    items
    "</ul></div>"
    blocks], newline));
end

function html = api_members_html(detail, link_entries)
sections = strings(0, 1);
if ~isempty(detail.properties)
    sections(end+1) = "<h2 id=""properties"">Properties</h2>";
    sections(end+1) = member_cards_html(detail.properties, "property", link_entries);
end
if ~isempty(detail.methods)
    sections(end+1) = "<h2 id=""methods"">Methods</h2>";
    sections(end+1) = member_cards_html(detail.methods, "method", link_entries);
end
html = char(strjoin(sections, newline));
end

function html = api_method_details_html(detail, link_entries)
if isempty(detail.methods)
    html = "";
    return
end

blocks = strings(0, 1);
for k = 1:numel(detail.methods)
    method = detail.methods(k);
    if isempty(method.syntax) && strlength(method.description) == 0 && isempty(method.examples)
        continue
    end

    anchor = "method-" + lower(regexprep(char(method.name), '[^A-Za-z0-9]+', '-'));
    blocks(end+1, 1) = "<section class=""api-detail"" id=""" + anchor + """>"; %#ok<AGROW>
    blocks(end+1, 1) = "<h3><code>" + string(html_escape(method.name)) + "</code></h3>"; %#ok<AGROW>
    if ~isempty(method.syntax)
        blocks(end+1, 1) = api_syntax_html_with_heading(method.syntax, "Syntax", anchor + "-syntax"); %#ok<AGROW>
    end
    if strlength(method.description) > 0
        desc = link_api_references(char(method.description), link_entries, "ref");
        blocks(end+1, 1) = "<div class=""api-description"">" + string(desc) + "</div>"; %#ok<AGROW>
    end
    if ~isempty(method.examples)
        blocks(end+1, 1) = examples_blocks_html(method.examples); %#ok<AGROW>
    end
    blocks(end+1, 1) = "</section>"; %#ok<AGROW>
end

if isempty(blocks)
    html = "";
else
    html = char(strjoin(["<h2 id=""method-details"">Method Details</h2>"; blocks], newline));
end
end

function html = api_description_html(entry, help_text, link_entries)
paragraphs = prose_paragraphs_from_help(help_text, entry.name);
if isempty(paragraphs)
    paragraphs = default_description_for_api(entry);
end
if isempty(paragraphs)
    html = "";
    return
end

items = strings(numel(paragraphs), 1);
for k = 1:numel(paragraphs)
    paragraph = format_inline_doc_html(paragraphs(k), link_entries, "ref");
    items(k) = "<p>" + string(paragraph) + "</p>";
end
html = char(strjoin(["<h2 id=""description"">Description</h2>"; items], newline));
end

function html = api_arguments_html(entry, detail, link_entries)
sections = strings(0, 1);
if ~isempty(detail.inputs)
    sections(end+1, 1) = "<h2 id=""input-arguments"">Input Arguments</h2>";
    sections(end+1, 1) = argument_list_html(detail.inputs, "input", entry, link_entries);
end
if ~isempty(detail.nameValues)
    sections(end+1, 1) = "<h2 id=""name-value-arguments"">Name-Value Arguments</h2>";
    sections(end+1, 1) = argument_list_html(detail.nameValues, "name-value", entry, link_entries);
end
if ~isempty(detail.outputs)
    sections(end+1, 1) = "<h2 id=""output-arguments"">Output Arguments</h2>";
    sections(end+1, 1) = argument_list_html(detail.outputs, "output", entry, link_entries);
end
html = char(strjoin(sections, newline));
end

function html = argument_list_html(args, role, entry, link_entries)
rows = strings(numel(args), 1);
for k = 1:numel(args)
    name = string(args(k).name);
    summary = api_variable_summary(name, role, entry.name);
    default_text = "";
    if role ~= "output" && strlength(string(args(k).default)) > 0
        default_text = " <span class=""argument-default"">Default: <code>" + ...
            string(html_escape(args(k).default)) + "</code></span>";
    end
    rows(k) = "<article class=""argument-item""><h3><code>" + ...
        string(html_escape(name)) + "</code></h3><p>" + ...
        string(format_inline_doc_html(summary, link_entries, "ref")) + default_text + "</p></article>";
end
html = char("<div class=""argument-list"">" + newline + strjoin(rows, newline) + newline + "</div>");
end

function html = api_examples_html(detail, link_entries)
examples = detail.examples;
if isempty(examples)
    html = "";
    return
end

heading = "<h2 id=""examples"">Examples</h2>";
blocks = examples_blocks_html(examples);
html = char(strjoin([heading; string(link_api_references(blocks, link_entries, "ref"))], newline));
end

function html = examples_blocks_html(examples)
blocks = strings(numel(examples), 1);
for k = 1:numel(examples)
    blocks(k) = "<div class=""code-example""><div class=""code-toolbar""><span>MATLAB</span></div><pre class=""codeinput"">" + ...
        string(html_escape(examples(k))) + "</pre></div>";
end
html = char(strjoin(blocks, newline));
end

function html = member_cards_html(members, kind, link_entries)
cards = strings(numel(members), 1);
for k = 1:numel(members)
    name = string(html_escape(members(k).name));
    summary = string(format_inline_doc_html(members(k).summary, link_entries, "ref"));
    member_id = "member-" + lower(regexprep(char(members(k).name), '[^A-Za-z0-9]+', '-'));
    if string(kind) == "method"
        href = "#" + method_anchor(members(k).name);
    else
        href = "#" + member_id;
    end
    cards(k) = "<article class=""api-card"" id=""" + member_id + """><h3><a href=""" + ...
        href + """><code>" + name + "</code></a></h3><p>" + summary + "</p></article>";
end
html = char("<div class=""api-grid"">" + newline + strjoin(cards, newline) + newline + "</div>");
end

function members = parse_class_properties(source_text)
lines = splitlines(string(source_text));
template = struct('name', "", 'summary', "");
members = repmat(template, 0, 1);
inside = false;
skip_block = false;
comment_buffer = strings(0, 1);

for k = 1:numel(lines)
    raw = lines(k);
    line = strip(raw);
    if startsWith(line, "properties")
        inside = true;
        skip_block = is_private_or_hidden_block(line);
        comment_buffer = strings(0, 1);
        continue
    end
    if inside && line == "end"
        inside = false;
        skip_block = false;
        comment_buffer = strings(0, 1);
        continue
    end
    if ~inside || skip_block || line == ""
        continue
    end
    if startsWith(line, "%")
        comment_buffer(end+1) = strip(extractAfter(line, 1)); %#ok<AGROW>
        continue
    end

    tokens = regexp(char(line), '^([A-Za-z]\w*)($|\s|=|\(|\{)', 'tokens', 'once');
    if isempty(tokens)
        comment_buffer = strings(0, 1);
        continue
    end

    name = string(tokens{1});
    members(end+1) = struct('name', name, 'summary', member_summary(comment_buffer, name)); %#ok<AGROW>
    comment_buffer = strings(0, 1);
end
end

function members = parse_class_methods(source_text, class_api_name)
lines = splitlines(string(source_text));
template = struct('name', "", 'summary', "", ...
    'syntax', strings(0, 1), 'description', "", 'examples', strings(0, 1), ...
    'inputs', [], 'nameValues', []);
members = repmat(template, 0, 1);
inside = false;
skip_block = false;

for k = 1:numel(lines)
    line = strip(lines(k));
    if startsWith(line, "properties") || startsWith(line, "events") || ...
            startsWith(line, "enumeration")
        inside = false;
        skip_block = false;
        continue
    end
    if startsWith(line, "methods")
        inside = true;
        skip_block = is_private_or_hidden_block(line);
        continue
    end
    if ~inside || skip_block
        continue
    end

    tokens = regexp(char(line), '^\s*function\s+(?:\[[^\]]+\]\s*=\s*|[A-Za-z]\w*\s*=\s*)?([A-Za-z]\w*(?:\.[A-Za-z]\w*)?)\s*\(', 'tokens', 'once');
    if isempty(tokens)
        continue
    end
    name = string(tokens{1});
    if startsWith(name, "get.") || startsWith(name, "set.")
        continue
    end
    comment_lines = following_comment_lines(lines, k + 1);
    summary = member_summary(comment_lines, name);
    if summary == ""
        summary = "Method " + name + ".";
    end
    [inputs, name_values] = parse_arguments_after_line(lines, k + 1);
    syntax = syntax_from_comment_lines(comment_lines);
    signature_syntax = syntax_from_method_signature(line, class_api_name, name);
    if isempty(syntax)
        syntax = signature_syntax;
    end
    syntax = expand_name_value_syntax(syntax, name_values);
    members(end+1) = struct( ...
        'name', name, ...
        'summary', summary, ...
        'syntax', syntax, ...
        'description', description_from_comment_lines(comment_lines, name), ...
        'examples', examples_from_comment_lines(comment_lines), ...
        'inputs', inputs, ...
        'nameValues', name_values); %#ok<AGROW>
end
end

function [inputs, name_values] = parse_arguments_after_line(lines, start_index)
template = struct('name', "", 'summary', "", 'default', "");
inputs = repmat(template, 0, 1);
name_values = repmat(template, 0, 1);
inside = false;

for k = start_index:numel(lines)
    line = strip(lines(k));
    if ~inside && startsWith(line, "function")
        return
    end
    if ~inside && (startsWith(line, "methods") || startsWith(line, "properties") || ...
            startsWith(line, "events") || startsWith(line, "enumeration"))
        return
    end
    if startsWith(line, "arguments")
        inside = true;
        continue
    end
    if inside && line == "end"
        return
    end
    if ~inside || line == "" || startsWith(line, "%")
        continue
    end

    [arg, is_name_value] = parse_argument_declaration(line);
    if arg.name == ""
        continue
    end
    if is_name_value
        name_values(end+1) = arg; %#ok<AGROW>
    elseif string(arg.name) ~= "opts"
        inputs(end+1) = arg; %#ok<AGROW>
    end
end
end

function syntax = syntax_from_method_signature(line, class_api_name, method_name)
syntax = strings(0, 1);
line = string(strtrim(line));
line = erase(line, " %#ok<STOUT>");
line = regexprep(line, '^\s*function\s+', '');
open_idx = strfind(char(line), '(');
close_idx = find(char(line) == ')', 1, 'last');
if isempty(open_idx) || isempty(close_idx) || close_idx <= open_idx(1)
    return
end

prefix = strip(extractBefore(line, open_idx(1)));
args = extractBetween(line, open_idx(1) + 1, close_idx - 1);
args = args(1);
eq_idx = strfind(char(prefix), '=');
if isempty(eq_idx)
    lhs = "";
else
    lhs = strip(extractBefore(prefix, eq_idx(end)));
end

class_short_name = last_api_part(class_api_name);
if string(method_name) == class_short_name
    call_name = string(class_api_name);
else
    call_name = string(method_name);
end

if lhs == ""
    syntax = call_name + "(" + args + ")";
else
    syntax = lhs + " = " + call_name + "(" + args + ")";
end
end

function comment_lines = following_comment_lines(lines, start_index)
comment_lines = strings(0, 1);
for k = start_index:numel(lines)
    line = strip(lines(k));
    if startsWith(line, "%")
        comment_lines(end+1, 1) = strip(extractAfter(line, 1)); %#ok<AGROW>
    elseif line == ""
        if ~isempty(comment_lines)
            comment_lines(end+1, 1) = ""; %#ok<AGROW>
        end
    else
        break
    end
end
end

function syntax = syntax_from_comment_lines(comment_lines)
syntax = strings(0, 1);
for k = 1:numel(comment_lines)
    line = strip(comment_lines(k));
    if line == "" || startsWith(line, "Example:")
        continue
    end
    if contains(line, "(") && (contains(line, "=") || startsWith(line, "["))
        tokens = regexp(char(line), '^\s*((?:\[[^\]]+\]|[A-Za-z]\w*)\s*=\s*(?:\.\.\.)?(?:[A-Za-z]\w*\.)*[A-Za-z]\w*\([^)]*\))', 'tokens', 'once');
        if isempty(tokens)
            tokens = regexp(char(line), '^\s*((?:[A-Za-z]\w*\.)*[A-Za-z]\w*\([^)]*\))', 'tokens', 'once');
        end
        if isempty(tokens)
            syntax(end+1, 1) = line; %#ok<AGROW>
        else
            syntax(end+1, 1) = string(tokens{1}); %#ok<AGROW>
        end
    end
    if numel(syntax) >= 6
        break
    end
end
end

function description = description_from_comment_lines(comment_lines, member_name)
paragraphs = paragraphs_from_comment_lines(comment_lines);
if isempty(paragraphs)
    description = "";
    return
end

summary = member_summary(comment_lines, member_name);
text_blocks = strings(0, 1);
for k = 1:numel(paragraphs)
    paragraph = clean_api_prose_paragraph(paragraphs(k));
    macro = upper(regexprep(char(member_name), '[^A-Za-z0-9]', '_'));
    paragraph = string(regexprep(char(paragraph), ['^', regexptranslate('escape', macro), '\s+'], ''));
    if paragraph == "" || startsWith(paragraph, "Example:") || skip_internal_doc_paragraph(paragraph)
        continue
    end
    if paragraph == summary
        continue
    end
    text_blocks(end+1, 1) = "<p>" + string(inline_math_help_html(paragraph)) + "</p>"; %#ok<AGROW>
end
description = strjoin(text_blocks, newline);
end

function examples = examples_from_comment_lines(comment_lines)
examples = strings(0, 1);
inside = false;
buffer = strings(0, 1);
for k = 1:numel(comment_lines)
    line = comment_lines(k);
    stripped = strip(line);
    if startsWith(stripped, "Example:")
        if inside && ~isempty(buffer)
            examples(end+1, 1) = strjoin(buffer, newline); %#ok<AGROW>
        end
        inside = true;
        buffer = strings(0, 1);
        after = strip(extractAfter(stripped, strlength("Example:")));
        if after ~= ""
            buffer(end+1, 1) = after; %#ok<AGROW>
        end
        continue
    end
    if inside
        if stripped == "" && isempty(buffer)
            continue
        end
        if stripped == "" && ~isempty(buffer)
            examples(end+1, 1) = strjoin(buffer, newline); %#ok<AGROW>
            inside = false;
            buffer = strings(0, 1);
        else
            buffer(end+1, 1) = line; %#ok<AGROW>
        end
    end
end
if inside && ~isempty(buffer)
    examples(end+1, 1) = strjoin(buffer, newline);
end
end

function paragraphs = paragraphs_from_comment_lines(comment_lines)
paragraphs = strings(0, 1);
buffer = strings(0, 1);
for k = 1:numel(comment_lines)
    line = strip(comment_lines(k));
    if line == ""
        if ~isempty(buffer)
            paragraphs(end+1, 1) = strjoin(buffer, " "); %#ok<AGROW>
            buffer = strings(0, 1);
        end
    else
        buffer(end+1, 1) = line; %#ok<AGROW>
    end
end
if ~isempty(buffer)
    paragraphs(end+1, 1) = strjoin(buffer, " ");
end
end

function skip = is_private_or_hidden_block(line)
normalized = lower(regexprep(char(line), '\s+', ''));
private_access = (contains(normalized, "access=private") || contains(normalized, "access=protected")) && ...
    ~contains(normalized, "setaccess=") && ~contains(normalized, "getaccess=");
private_get_access = contains(normalized, "getaccess=private") || contains(normalized, "getaccess=protected");
hidden_block = contains(normalized, "hidden") && ~contains(normalized, "hidden=false");
skip = private_access || private_get_access || hidden_block;
end

function summary = member_summary(comment_lines, name)
if isempty(comment_lines)
    summary = "Member " + string(name) + ".";
    return
end

line = strip(comment_lines(1));
upper_name = upper(regexprep(char(name), '^get\.', ''));
pattern = "^" + regexptranslate('escape', upper_name) + "\s+";
summary = regexprep(char(line), char(pattern), '');
summary = string(strtrim(summary));
if summary == ""
    summary = line;
end
end

function html = api_related_html(entry, api_entries, module_entries)
same_module = api_entries(string({api_entries.module}) == string(entry.module) & ...
    string({api_entries.name}) ~= string(entry.name));
same_module = same_module(1:min(numel(same_module), 12));
module_link = api_ref_href(entry.module, module_entries, "ref");
if isempty(same_module)
    html = "<h2 id=""see-also"">See Also</h2><p><a href=""" + module_link + """>" + ...
        string(html_escape(entry.module)) + "</a></p>";
    html = char(html);
    return
end

items = strings(numel(same_module), 1);
for k = 1:numel(same_module)
    items(k) = "<li><a href=""" + api_ref_href(same_module(k).name, same_module, "ref") + """><code>" + ...
        string(html_escape(api_display_name(same_module(k).name, entry.module))) + "</code></a></li>";
end
html = char(strjoin([
    "<h2 id=""see-also"">See Also</h2>"
    "<p><a href=""" + module_link + """>" + string(html_escape(entry.module)) + "</a></p>"
    "<ul class=""api-related"">"
    items
    "</ul>"], newline));
end

function anchor = method_anchor(method_name)
anchor = "method-" + lower(regexprep(char(method_name), '[^A-Za-z0-9]+', '-'));
end

function html = format_inline_doc_html(text, link_entries, context)
html = link_api_references(inline_math_help_html(text), link_entries, context);
end

function html = api_entry_grid_html(entries, context, current_module, link_entries)
if isempty(entries)
    html = "";
    return
end

cards = strings(numel(entries), 1);
for k = 1:numel(entries)
    entry = entries(k);
    summary = string(entry.summary);
    if summary == ""
        summary = fallback_entry_summary(entry);
    end
    label = api_display_name(entry.name, current_module);
    summary_html = format_inline_doc_html(summary, link_entries, context);
    cards(k) = "<article class=""api-card""><div class=""api-kind"">" + ...
        string(html_escape(entry.kind)) + "</div><h3><a href=""" + ...
        api_ref_href(entry.name, entries, context) + """><code>" + ...
        string(html_escape(label)) + "</code></a></h3><p>" + ...
        string(summary_html) + "</p></article>";
end
html = char("<div class=""api-grid"">" + newline + strjoin(cards, newline) + newline + "</div>");
end

function label = api_display_name(api_name, current_module)
api_name = string(api_name);
current_module = string(current_module);
prefix = current_module + ".";
if current_module ~= "" && startsWith(api_name, prefix)
    label = extractAfter(api_name, strlength(prefix));
    if contains(label, ".")
        parts = split(label, ".");
        label = parts(1);
    end
else
    parts = split(api_name, ".");
    label = parts(end);
end
end

function summary = fallback_entry_summary(entry)
switch string(entry.kind)
    case "module"
        summary = module_summary(entry.name);
    case "class"
        summary = "Class in " + string(entry.module) + ".";
    otherwise
        summary = "Function in " + string(entry.module) + ".";
end
end

function href = api_ref_href(api_name, entries, context)
href = "../api_reference.html";
api_name = string(api_name);
if api_name == ""
    return
end
for k = 1:numel(entries)
    if string(entries(k).name) == api_name
        target = string(entries(k).target);
        if context == "ref"
            target = erase(target, "ref/");
        end
        href = string(html_escape(target));
        return
    end
end
end

function html = link_api_references(html, entries, context)
if isempty(entries)
    return
end

html = char(html);
[protected_matches, starts, ends] = regexp(html, '(?s)<pre(?=[\s>]).*?</pre>|<a(?=[\s>]).*?</a>|\\\(.*?\\\)|\\\[.*?\\\]', 'match', 'start', 'end');
for k = numel(protected_matches):-1:1
    placeholder = sprintf('__COPP_ESCAPED_HTML_%04d__', k);
    html = [html(1:starts(k)-1), placeholder, html(ends(k)+1:end)];
end

aliases = api_link_aliases(entries, context);
labels = string({aliases.label});
[~, order] = sort(strlength(labels), 'descend');
for idx = reshape(order, 1, [])
    label = labels(idx);
    if strlength(label) == 0
        continue
    end
    replacement = "<a class=""api-link"" href=""" + string(aliases(idx).href) + """>" + ...
        string(html_escape(label)) + "</a>";
    pattern = ['(?<![A-Za-z0-9_.])', regexptranslate('escape', char(label)), '(?![A-Za-z0-9_.])'];
    html = regexprep(html, pattern, char(replacement));
end

for k = 1:numel(protected_matches)
    placeholder = sprintf('__COPP_ESCAPED_HTML_%04d__', k);
    html = strrep(html, placeholder, protected_matches{k});
end
end

function aliases = api_link_aliases(entries, context)
template = struct('label', "", 'href', "");
aliases = repmat(template, 0, 1);
if isempty(entries)
    return
end

names = string({entries.name});
kinds = string({entries.kind});
last_parts = strings(numel(entries), 1);
for k = 1:numel(entries)
    parts = split(names(k), ".");
    last_parts(k) = parts(end);
end

ambiguous_short_names = [
    "Problem"
    "Options"
    "Result"
    "Settings"
    "solve"
    "solve_expert"
    "version"
    "time"
    "linear"
    "plus"
    "minus"
    "times"
    "rdivide"
    "ldivide"
    "mrdivide"
    "mldivide"
    "mtimes"
    "power"
    "mpower"
    "sin"
    "cos"
    "sqrt"
    "exp"
    "log"
    "abs"
    "solver"
    "objective"
    "interpolation"
    "clarabel"
    "diag"
    ];

for k = 1:numel(entries)
    name = names(k);
    href = api_ref_href(name, entries, context);
    aliases(end+1, 1) = struct('label', name, 'href', href); %#ok<AGROW>

    if startsWith(name, "copp.")
        relative_name = extractAfter(name, strlength("copp."));
        if relative_name ~= "" && (contains(relative_name, ".") || kinds(k) ~= "module")
            aliases(end+1, 1) = struct('label', relative_name, 'href', href); %#ok<AGROW>
        end
    end

    short_name = last_parts(k);
    is_unique = sum(last_parts == short_name) == 1;
    is_specific_kind = kinds(k) ~= "module" || startsWith(name, "copp.");
    if is_unique && strlength(short_name) >= 4 && ~any(short_name == ambiguous_short_names) && is_specific_kind
        aliases(end+1, 1) = struct('label', short_name, 'href', href); %#ok<AGROW>
    end
end

if isempty(aliases)
    return
end

labels = string({aliases.label});
[~, first_index] = unique(labels, 'stable');
aliases = aliases(sort(first_index));
end

function html = inline_math_help_html(help_text)
html = string(html_escape(help_text));
html = wrap_inline_code_identifiers(html);

replacements = [
    "a_min(k) &lt;= a(k) &lt;= a_max(k)", "\(a_{\min}(k) \le a(k) \le a_{\max}(k)\)"
    "a(k) = (ds/dt)^2", "\(a(k) = (ds/dt)^2\)"
    "a(k)=(ds/dt)^2", "\(a(k) = (ds/dt)^2\)"
    "b(k) = d2s/dt2", "\(b(k) = d^2s/dt^2\)"
    "b(k)=d2s/dt2", "\(b(k) = d^2s/dt^2\)"
    "a = (ds/dt)^2", "\(a = (ds/dt)^2\)"
    "a=(ds/dt)^2", "\(a = (ds/dt)^2\)"
    "(ds/dt)^2", "\((ds/dt)^2\)"
    "d2s/dt2", "\(d^2s/dt^2\)"
    "tau = ddq", "\(\tau = \ddot{q}\)"
    "q(s)", "\(q(s)\)"
    "s(t)", "\(s(t)\)"
    "dim-by-N", "\(\mathrm{dim} \times N\)"
    "dim-by-1", "\(\mathrm{dim} \times 1\)"
    "N-by-1", "\(N \times 1\)"
    "1-by-N", "\(1 \times N\)"
    "1-by-2", "\(1 \times 2\)"
    "s_len-by-1", "\(s_\mathrm{len} \times 1\)"
    ];

for k = 1:size(replacements, 1)
    placeholder = "__COPP_INLINE_MATH_" + k + "__";
    html = replace(html, replacements(k, 1), placeholder);
end

for k = 1:size(replacements, 1)
    placeholder = "__COPP_INLINE_MATH_" + k + "__";
    html = replace(html, placeholder, replacements(k, 2));
end

html = char(html);
end

function html = wrap_inline_code_identifiers(html)
html = string(html);
html = regexprep(html, ...
    '(?<![A-Za-z0-9_])([A-Z][A-Za-z0-9_]*(?:\.[A-Za-z][A-Za-z0-9_]*)+)(?![A-Za-z0-9_])', ...
    '<code>$1</code>');
html = regexprep(html, ...
    '(?<![A-Za-z0-9_.])([A-Z][A-Z0-9]*_[A-Z0-9_]+)(?![A-Za-z0-9_.])', ...
    '<code>$1</code>');
html = regexprep(html, ...
    '(?<![A-Za-z0-9_.])([a-z][a-z0-9]*_[A-Za-z0-9_]+)(?![A-Za-z0-9_.])', ...
    '<code>$1</code>');
end

function paragraphs = prose_paragraphs_from_help(help_text, api_name)
lines = splitlines(string(help_text));
paragraphs = strings(0, 1);
buffer = strings(0, 1);
for k = 1:numel(lines)
    line = strip(lines(k));
    if line == ""
        if ~isempty(buffer)
            paragraphs(end+1, 1) = strjoin(buffer, " "); %#ok<AGROW>
            buffer = strings(0, 1);
        end
    else
        buffer(end+1, 1) = line; %#ok<AGROW>
    end
end
if ~isempty(buffer)
    paragraphs(end+1, 1) = strjoin(buffer, " ");
end

if ~isempty(paragraphs)
    first = paragraphs(1);
    macro = upper(regexprep(last_api_part(api_name), '[^A-Za-z0-9]', '_'));
    if startsWith(first, macro + " ")
        paragraphs(1) = [];
    end
end

cleaned = strings(0, 1);
for k = 1:numel(paragraphs)
    paragraph = clean_api_prose_paragraph(paragraphs(k));
    if paragraph == "" || startsWith(paragraph, "Example:") || skip_internal_doc_paragraph(paragraph)
        continue
    end
    cleaned(end+1, 1) = paragraph; %#ok<AGROW>
end
paragraphs = cleaned;
end

function skip = skip_internal_doc_paragraph(paragraph)
text = lower(string(paragraph));
patterns = [
    "native handle"
    "native_id"
    "mex gateway"
    "c abi"
    "zero-copy"
    "raw native"
    "registry id"
    "coppPath"
    ];
skip = any(contains(text, lower(patterns)));
end

function paragraphs = default_description_for_api(entry)
name = string(entry.name);
kind = string(entry.kind);
paragraphs = strings(0, 1);

if contains(name, ".solver.") && endsWith(name, ".Problem")
    namespace = extractBefore(name, strlength(name) - strlength(".Problem") + 1);
    if contains(name, "copp2_")
        paragraphs(end+1, 1) = "Use this Problem object to bind a copp.Robot, COPP objective descriptors, a closed station interval, and boundary values for a = (ds/dt)^2 before calling " + namespace + ".solve.";
    elseif contains(name, "topp2_") || contains(name, "reach_set2")
        paragraphs(end+1, 1) = "Use this Problem object to bind a copp.Robot, a closed station interval, and boundary values for a = (ds/dt)^2 before calling " + namespace + ".solve or a reach-set helper in the same namespace.";
    elseif contains(name, "copp3_")
        paragraphs(end+1, 1) = "Use this Problem object to bind a third-order copp.Robot, COPP objective descriptors, an a_linearization seed, boundary values, and stationary-node options before calling " + namespace + ".solve.";
    else
        paragraphs(end+1, 1) = "Use this Problem object to bind a third-order copp.Robot, an a_linearization seed, boundary values, and stationary-node options before calling " + namespace + ".solve.";
    end
    paragraphs(end+1, 1) = "Problem objects are lightweight MATLAB descriptors. They keep references to user-owned objects and validate shape/index contracts at construction time.";
elseif contains(name, ".solver.") && endsWith(name, ".Options")
    namespace = extractBefore(name, strlength(name) - strlength(".Options") + 1);
    paragraphs(end+1, 1) = "Use this Options object to override numerical tolerances, sampling counts, verbosity, or backend policy for " + namespace + " calls.";
    paragraphs(end+1, 1) = "Default construction is usually a good first choice; set name-value arguments only when a solve needs different precision, diagnostics, or refinement behavior.";
elseif contains(name, ".solver.") && (endsWith(name, ".solve") || endsWith(name, ".solve_expert") || endsWith(name, ".solve_with_reach_set"))
    namespace = extractBefore(name, strlength(name) - strlength("." + last_api_part(name)) + 1);
    paragraphs(end+1, 1) = "Call this function after constructing the matching " + namespace + ".Problem and options object. Normal solve functions return accepted profiles directly; expert variants return diagnostic Result objects.";
elseif kind == "class"
    paragraphs(end+1, 1) = "This MATLAB class is part of the public COPP API. Use its constructor, properties, and documented methods from the same package namespace.";
elseif kind == "function"
    paragraphs(end+1, 1) = "This MATLAB function is part of the public COPP API. See Syntax and Arguments for accepted inputs and returned values.";
end
end

function paragraph = clean_api_prose_paragraph(paragraph)
paragraph = strip(string(paragraph));
paragraph = regexprep(paragraph, '^[A-Z]+_[A-Z0-9_]*\s+', '');
paragraph = regexprep(paragraph, '^\s*(?:\[[^\]]+\]|[A-Za-z]\w*)\s*=\s*(?:\.\.\.)?(?:[A-Za-z]\w*\.)*[A-Za-z]\w*\([^)]*\)\s*', '');
paragraph = regexprep(paragraph, '^\s*(?:[A-Za-z]\w*\.)*[A-Za-z]\w*\([^)]*\)\s*', '');
paragraph = strip(paragraph);
if paragraph == ""
    return
end

first_word = extractBefore(paragraph + " ", " ");
switch lower(first_word)
    case "returns"
        paragraph = "Returns" + extractAfter(paragraph, strlength(first_word));
    case "converts"
        paragraph = "Converts" + extractAfter(paragraph, strlength(first_word));
    case "starts"
        paragraph = "Starts" + extractAfter(paragraph, strlength(first_word));
    case "wraps"
        paragraph = "Wraps" + extractAfter(paragraph, strlength(first_word));
    case "builds"
        paragraph = "Builds" + extractAfter(paragraph, strlength(first_word));
end
end

function examples = extract_help_examples(help_text)
lines = splitlines(string(help_text));
comment_lines = strip(lines);
examples = examples_from_comment_lines(comment_lines);
end

function examples = default_examples_for_api(api_name)
name = string(api_name);
examples = strings(0, 1);
switch name
    case "copp.Path"
        examples(end+1, 1) = strjoin([
            "waypoints = [0 1 2; 0 0.5 0];"
            "path = copp.Path.from_waypoints(waypoints, s_range=[0, 1]);"
            "s = linspace(0, 1, 5);"
            "q = path.evaluate_q(s);"
            "disp(q)"], newline);
    case "copp.interpolation.a_to_b_topp2"
        examples(end+1, 1) = strjoin([
            "s = linspace(0, 1, 5).';"
            "a = [0; 0.5; 1.0; 0.5; 0];"
            "b = copp.interpolation.a_to_b_topp2(s, a);"
            "disp(b)"], newline);
    case "copp.interpolation.s_to_t_topp2"
        examples(end+1, 1) = strjoin([
            "s = linspace(0, 1, 5).';"
            "a = 0.2 + sin(pi*s).^2;"
            "[t_final, t_s] = copp.interpolation.s_to_t_topp2(s, a);"
            "disp([t_final, t_s(end)])"], newline);
    otherwise
        examples = default_solver_examples(name);
end
end

function examples = default_solver_examples(api_name)
examples = strings(0, 1);
name = string(api_name);
if ~contains(name, ".solver.")
    return
end

parts = split(name, ".");
if numel(parts) < 4
    return
end
solver_name = parts(end - 1);
member_name = parts(end);

switch member_name
    case "Problem"
        examples = solver_problem_example(solver_name);
    case "Options"
        examples = solver_options_example(solver_name);
    case {"solve", "solve_expert", "solve_with_reach_set", "backward", "bidirectional"}
        examples = solver_call_example(solver_name, member_name);
end
end

function examples = solver_problem_example(solver_name)
examples = strings(0, 1);
solver_name = string(solver_name);
switch solver_name
    case {"topp2_ra", "reach_set2"}
        examples(end+1, 1) = strjoin([
            "ExampleCommon.setup_path();"
            "ctx = ExampleCommon.second_order_context();"
            "problem = copp.solver." + solver_name + ".Problem( ..."
            "    ctx.robot, ..."
            "    idx_s_interval=ctx.idx_s_interval, ..."
            "    a_boundary=ctx.a_boundary);"], newline);
    case "copp2_socp"
        examples(end+1, 1) = strjoin([
            "ExampleCommon.setup_path();"
            "ctx = ExampleCommon.second_order_context();"
            "objectives = ExampleCommon.convex_objectives(ctx.dim);"
            "problem = copp.solver." + solver_name + ".Problem( ..."
            "    ctx.robot, ..."
            "    objectives, ..."
            "    idx_s_interval=ctx.idx_s_interval, ..."
            "    a_boundary=ctx.a_boundary);"], newline);
    case {"topp3", "topp3_lp", "topp3_socp"}
        examples(end+1, 1) = strjoin([
            "ExampleCommon.setup_path();"
            "ctx = ExampleCommon.third_order_context();"
            "a_seed = ExampleCommon.solve_topp2_seed(ctx.robot, ctx.n);"
            "problem = copp.solver." + solver_name + ".Problem( ..."
            "    ctx.robot, ..."
            "    a_seed, ..."
            "    idx_s_start=1, ..."
            "    a_boundary=ctx.a_boundary, ..."
            "    b_boundary=ctx.b_boundary, ..."
            "    num_stationary_max=ctx.num_stationary_max);"], newline);
    case "copp3_socp"
        examples(end+1, 1) = strjoin([
            "ExampleCommon.setup_path();"
            "ctx = ExampleCommon.third_order_context();"
            "a_seed = ExampleCommon.solve_topp2_seed(ctx.robot, ctx.n);"
            "objectives = ExampleCommon.convex_objectives(ctx.dim);"
            "problem = copp.solver." + solver_name + ".Problem( ..."
            "    ctx.robot, ..."
            "    objectives, ..."
            "    a_seed, ..."
            "    idx_s_start=1, ..."
            "    a_boundary=ctx.a_boundary, ..."
            "    b_boundary=ctx.b_boundary, ..."
            "    num_stationary_max=ctx.num_stationary_max);"], newline);
end
end

function examples = solver_options_example(solver_name)
examples = strings(0, 1);
solver_name = string(solver_name);
switch solver_name
    case {"topp2_ra", "reach_set2"}
        examples(end+1, 1) = "options = copp.solver." + solver_name + ".Options(verbosity=""summary"");";
    case {"copp2_socp", "topp3_lp", "topp3_socp", "copp3_socp"}
        examples(end+1, 1) = "options = copp.solver." + solver_name + ".Options();";
end
end

function examples = solver_call_example(solver_name, member_name)
examples = strings(0, 1);
solver_name = string(solver_name);
member_name = string(member_name);
switch solver_name
    case {"topp2_ra", "copp2_socp"}
        if member_name == "solve_expert"
            call_line = "result = copp.solver." + solver_name + ".solve_expert(problem, options);";
            post_line = "a_profile = result.a;";
        else
            call_line = "a_profile = copp.solver." + solver_name + ".solve(problem, options);";
            post_line = "[t_final, ~, s_t] = ExampleCommon.postprocess_topp2(ctx, a_profile);";
        end
        examples(end+1, 1) = strjoin([
            "ExampleCommon.setup_path();"
            "ctx = ExampleCommon.second_order_context();"
            solver_objective_setup_line(solver_name)
            "problem = " + solver_problem_call_line(solver_name) + ";"
            "options = copp.solver." + solver_name + ".Options();"
            call_line
            post_line], newline);
    case "reach_set2"
        examples(end+1, 1) = strjoin([
            "ExampleCommon.setup_path();"
            "ctx = ExampleCommon.second_order_context();"
            "problem = copp.solver.reach_set2.Problem(ctx.robot);"
            "options = copp.solver.reach_set2.Options();"
            "reach_set = copp.solver.reach_set2." + member_name + "(problem, options);"], newline);
    case {"topp3_lp", "topp3_socp"}
        output_line = "profile = copp.solver." + solver_name + ".solve(problem, options);";
        options_line = "options = copp.solver." + solver_name + ".Options();";
        if member_name == "solve_expert"
            output_line = "result = copp.solver." + solver_name + ".solve_expert(problem, options);";
        end
        examples(end+1, 1) = strjoin([
            "ExampleCommon.setup_path();"
            "ctx = ExampleCommon.third_order_context();"
            "a_seed = ExampleCommon.solve_topp2_seed(ctx.robot, ctx.n);"
            "problem = " + solver_problem_call_line(solver_name) + ";"
            options_line
            output_line], newline);
    case "copp3_socp"
        if member_name == "solve_expert"
            output_line = "result = copp.solver.copp3_socp.solve_expert(problem, options);";
        else
            output_line = "profile = copp.solver.copp3_socp.solve(problem, options);";
        end
        examples(end+1, 1) = strjoin([
            "ExampleCommon.setup_path();"
            "ctx = ExampleCommon.third_order_context();"
            "a_seed = ExampleCommon.solve_topp2_seed(ctx.robot, ctx.n);"
            "objectives = ExampleCommon.convex_objectives(ctx.dim);"
            "problem = " + solver_problem_call_line(solver_name) + ";"
            "options = copp.solver.copp3_socp.Options();"
            output_line], newline);
end
end

function line = solver_objective_setup_line(solver_name)
if contains(string(solver_name), "copp2")
    line = "objectives = ExampleCommon.convex_objectives(ctx.dim);";
else
    line = "";
end
end

function line = solver_problem_call_line(solver_name)
solver_name = string(solver_name);
switch solver_name
    case {"topp2_ra", "reach_set2"}
        line = "copp.solver." + solver_name + ".Problem(ctx.robot, idx_s_interval=ctx.idx_s_interval, a_boundary=ctx.a_boundary)";
    case "copp2_socp"
        line = "copp.solver." + solver_name + ".Problem(ctx.robot, objectives, idx_s_interval=ctx.idx_s_interval, a_boundary=ctx.a_boundary)";
    case {"topp3", "topp3_lp", "topp3_socp"}
        line = "copp.solver." + solver_name + ".Problem(ctx.robot, a_seed, idx_s_start=1, a_boundary=ctx.a_boundary, b_boundary=ctx.b_boundary, num_stationary_max=ctx.num_stationary_max)";
    otherwise
        line = "copp.solver." + solver_name + ".Problem(ctx.robot, objectives, a_seed, idx_s_start=1, a_boundary=ctx.a_boundary, b_boundary=ctx.b_boundary, num_stationary_max=ctx.num_stationary_max)";
end
end

function summary = api_variable_summary(name, role, api_name)
name = string(name);
role = string(role);
api_name = string(api_name);
switch lower(name)
    case "s"
        summary = "Station grid or path-parameter samples. Use a real finite vector; station-grid inputs are usually strictly increasing.";
    case "a"
        summary = "Second-order node profile. Element a(k) stores the squared path speed (ds/dt)^2 at station s(k).";
    case "b"
        summary = "Interval or edge acceleration profile. For TOPP2 interpolation this is an (N-1)-by-1 column vector.";
    case "t_s"
        summary = "Cumulative arrival time at each station, returned as an N-by-1 double column vector.";
    case "t_final"
        summary = "Final traversal time. This is the last element of the cumulative time grid.";
    case "t0"
        summary = "Initial time offset added to the cumulative arrival-time grid.";
    case "t"
        summary = "Time value or vector of time samples.";
    case "t_sample"
        summary = "User-provided time samples where \(s(t)\) should be evaluated.";
    case "dt"
        summary = "Uniform time step used to create sampling times from zero to the final traversal time.";
    case "s_t"
        summary = "Sampled path parameter values s(t).";
    case "path"
        summary = "A copp.Path object used to sample geometric path data.";
    case "robot"
        summary = "A copp.Robot object containing sampled path data, limits, and constraints.";
    case "problem"
        summary = "Problem struct or object for the selected solver namespace.";
    case "objectives"
        summary = "Objective descriptor or cell array of descriptors created by copp.objective functions.";
    case "options"
        summary = "Solver options struct or object for the selected solver namespace.";
    case "settings"
        summary = "Clarabel or solver settings object.";
    case "result"
        summary = "Structured solver result containing the optimized profile and solver metadata.";
    case "profile"
        summary = "Third-order profile object containing a, b, and stationary-node metadata.";
    case "waypoints"
        summary = "Waypoint matrix whose columns are path samples. MATLAB APIs use dim-by-N layout.";
    case "q"
        summary = "Path position samples, returned as a dim-by-N matrix.";
    case "dq"
        summary = "First derivative of the path with respect to the path parameter.";
    case "ddq"
        summary = "Second derivative of the path with respect to the path parameter.";
    case "dddq"
        summary = "Third derivative of the path with respect to the path parameter.";
    case "q_expr"
        summary = "Symbolic or CasADi expression defining the geometric path.";
    case "symbol"
        summary = "Scalar symbolic or CasADi variable used as the path parameter.";
    case "s_range"
        summary = "Two-element vector [s_min, s_max] defining the valid path range.";
    case "idx_s_interval"
        summary = "Closed 1-based station interval [idx_s_start, idx_s_final] covered by the problem.";
    case "idx_s_start"
        summary = "1-based start station for third-order problems. The covered interval runs from this station through the stored profile length.";
    case "a_boundary"
        summary = "Endpoint squared-speed values [a_start, a_final] for the selected station interval.";
    case "b_boundary"
        summary = "Endpoint path-acceleration values [b_start, b_final] for third-order profiles.";
    case "a_linearization"
        summary = "Seed a(s) profile used to linearize third-order constraints.";
    case "a_linearization_floor"
        summary = "Positive floor applied to a_linearization when forming third-order linearization data.";
    case "num_stationary_max"
        summary = "Maximum number of stationary nodes allowed by the third-order profile descriptor.";
    case "num_edge_max"
        summary = "Maximum edge or contour capacity used by reachable-set options.";
    case "include_final"
        summary = "Whether sampling helpers should include the final time sample exactly.";
    case "verbosity"
        summary = "Diagnostic verbosity. Common values are silent, summary, debug, and trace.";
    case "has_a"
        summary = "Whether a second-order expert result contains an accepted a profile.";
    case "has_profile"
        summary = "Whether a third-order expert result contains an accepted Profile3rd.";
    case "solver_status"
        summary = "Backend solver status normalized to a lower-snake-case MATLAB string.";
    case "objective_value"
        summary = "Weighted COPP objective value reported by the solver when available.";
    case "objective_terms"
        summary = "Per-objective unweighted values reported by expert solver results.";
    case "dim"
        summary = "Positive integer path or robot dimension.";
    otherwise
        if role == "output"
            summary = "Returned value from " + api_name + ". See Description for its shape and meaning.";
        elseif role == "name-value"
            summary = "Name-value option for " + api_name + ".";
        else
            summary = "Input value for " + api_name + ". See Description for accepted shape and constraints.";
        end
end
end

function part = last_api_part(api_name)
parts = split(string(api_name), ".");
part = parts(end);
end

function summary = help_summary(help_text, api_name)
lines = splitlines(string(help_text));
lines = strip(lines);
lines(lines == "") = [];
if isempty(lines)
    summary = "MATLAB help for " + string(api_name) + ".";
    return
end

first_line = char(lines(1));
tokens = regexp(first_line, '^[A-Z0-9_]+\s+(.*)$', 'tokens', 'once');
if isempty(tokens)
    summary = string(first_line);
else
    summary = string(tokens{1});
end
end

function write_index_page(output_dir, pages, module_entries, link_entries, math_renderer)
overview_file = fullfile(output_dir, 'overview.html');
index_file = fullfile(output_dir, 'index.html');
if ~isfile(overview_file)
    return
end

html = fileread(overview_file);
home_html = home_reference_html(pages, module_entries, link_entries);
if ~isempty(regexp(html, '</div>\s*<nav class="on-page"', 'once'))
    html = regexprep(html, ...
        '</div>\s*(<nav class="on-page")', ...
        ['</div>', newline, home_html, newline, '$1'], ...
        'once');
else
    html = regexprep(html, '(<h2\s+id="[^"]+">)', [home_html, newline, '$1'], 'once');
end

html = regexprep(html, '<title>.*?</title>', ...
    '<title>COPP for MATLAB - COPP MATLAB Reference</title>', 'once');
html = link_api_references(html, link_entries, "root");
if math_renderer ~= "none" && has_inline_mathjax(html) && isempty(regexp(html, 'tex-chtml\.js', 'once'))
    html = inject_mathjax(html, math_renderer);
end
write_text_file(index_file, html);
end

function html = home_reference_html(pages, module_entries, link_entries)
guide_names = [
    "path"
    "robot_constraints"
    "solvers_topp2_copp2"
    "solvers_topp3_copp3"
    "interpolation"
    "errors_diagnostics"
    "api_reference"
    ];
page_names = string({pages.name});
page_titles = string({pages.title});
page_subtitles = string({pages.subtitle});

guide_cards = strings(0, 1);
for k = 1:numel(guide_names)
    idx = find(page_names == guide_names(k), 1);
    if isempty(idx)
        continue
    end
    guide_cards(end+1, 1) = "<article class=""api-card""><div class=""api-kind"">guide</div><h3><a href=""" + ...
        page_names(idx) + ".html""><code>" + string(html_escape(page_titles(idx))) + "</code></a></h3><p>" + ...
        string(format_inline_doc_html(page_subtitles(idx), link_entries, "root")) + "</p></article>"; %#ok<AGROW>
end

namespace_entries = module_entries(string({module_entries.module}) == "copp");
namespace_html = api_entry_grid_html(namespace_entries, "root", "copp", link_entries);
if strlength(namespace_html) == 0
    namespace_html = "<p>No generated API namespaces are available.</p>";
end

html = char(strjoin([
    "<section class=""home-reference"">"
    "<h2 id=""start-here"">Start Here</h2>"
    "<div class=""api-grid"">"
    guide_cards
    "</div>"
    "<h2 id=""api-namespaces"">API Namespaces</h2>"
    string(namespace_html)
    "</section>"], newline));
end

function write_helptoc(output_dir, pages, api_entries, module_entries)
page_names = string({pages.name});
page_titles = string({pages.title});

lines = strings(0, 1);
lines(end+1) = "<?xml version=""1.0"" encoding=""utf-8""?>";
lines(end+1) = "<toc version=""2.0"">";
lines(end+1) = "  <tocitem target=""index.html"">COPP MATLAB";
lines(end+1) = "    <tocitem target=""index.html"" image=""HelpIcon.GETTING_STARTED"">Getting Started</tocitem>";
lines(end+1) = "    <tocitem target=""path.html"" image=""HelpIcon.USER_GUIDE"">User Guide";

for k = 1:numel(page_names)
    if page_names(k) == "overview" || page_names(k) == "api_reference"
        continue
    end
    lines(end+1) = "      <tocitem target=""" + xml_escape(page_names(k) + ".html") + """>" + ...
        xml_escape(page_titles(k)) + "</tocitem>"; %#ok<AGROW>
end

lines(end+1) = "    </tocitem>";
lines(end+1) = "    <tocitem target=""api_reference.html"" image=""HelpIcon.FUNCTION"">API Reference";

category_items = [
    "Core Package", "api_reference.html#1"
    "Options and Diagnostics", "api_reference.html#5"
    "Objective Descriptors", "api_reference.html#4"
    "Second-Order Solvers", "api_reference.html#6"
    "Third-Order Solvers", "api_reference.html#7"
    "Interpolation", "api_reference.html#8"
    ];

for category_index = 1:size(category_items, 1)
    category = category_items(category_index, 1);
    category_line = "      <tocitem target=""" + xml_escape(category_items(category_index, 2)) + """>" + ...
        xml_escape(category);
    lines(end+1) = category_line; %#ok<AGROW>

    category_modules = module_entries(string({module_entries.category}) == category);
    for module_index = 1:numel(category_modules)
        module_entry = category_modules(module_index);
        lines(end+1) = "        <tocitem target=""" + xml_escape(module_entry.target) + """>" + ...
            xml_escape(module_entry.name); %#ok<AGROW>
        module_api = api_entries(string({api_entries.module}) == string(module_entry.name));
        for module_api_index = 1:numel(module_api)
            entry = module_api(module_api_index);
            lines(end+1) = "          <tocitem target=""" + xml_escape(entry.target) + """>" + ...
                xml_escape(entry.name) + "</tocitem>"; %#ok<AGROW>
        end
        lines(end+1) = "        </tocitem>"; %#ok<AGROW>
    end

    for entry_index = 1:numel(api_entries)
        entry = api_entries(entry_index);
        if entry.category ~= category
            continue
        end
        if any(string({module_entries.name}) == string(entry.module))
            continue
        end

        lines(end+1) = "        <tocitem target=""" + xml_escape(entry.target) + """>" + ...
            xml_escape(entry.name) + "</tocitem>"; %#ok<AGROW>
    end

    lines(end+1) = "      </tocitem>"; %#ok<AGROW>
end

lines(end+1) = "    </tocitem>";
lines(end+1) = "  </tocitem>";
lines(end+1) = "</toc>";
write_text_file(fullfile(output_dir, 'helptoc.xml'), char(strjoin(lines, newline)));
end
