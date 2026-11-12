function export_report_figures()
%EXPORT_REPORT_FIGURES  Saves everything the project report needs, in one go.
%   Run it from this folder (type  export_report_figures  and press Enter).
%   It takes about 15-30 minutes and writes to  ..\report_figures\ :
%     simulink\   a PNG of the top-level model and of every subsystem
%     results\    every result figure (whole figure + each panel separately)
%     *.csv/.mat  the result tables
%     console_log.txt   everything printed in the Command Window
%   Nothing outside D:\Major Project is touched.

here = fileparts(mfilename('fullpath'));
cd(here);
outdir = fullfile(fileparts(here), 'report_figures');
sdir = fullfile(outdir, 'simulink');
rdir = fullfile(outdir, 'results');
for d = {outdir, sdir, rdir}
    if ~exist(d{1}, 'dir'), mkdir(d{1}); end
end
logf = fullfile(outdir, 'console_log.txt');
if exist(logf, 'file'), delete(logf); end
diary(logf);
c1 = onCleanup(@() diary('off'));
v = ver('MATLAB');
fprintf('MATLAB %s %s | %s\n', v.Version, v.Release, datestr(now));
modelName = 'EV_BMS_SOC_Estimation_System';
close all force;

%% 1. Build (if needed) and print the Simulink diagrams ----------------------
evalin('base', 'run(''ET_Battery_Params.m'');');
ensure_model_built(modelName);
open_system(fullfile(here, [modelName '.slx']));
idx = {};
print_sys(modelName, fullfile(sdir, '00_TOP_LEVEL.png'));
idx(end+1, :) = {'simulink/00_TOP_LEVEL.png', modelName};
subs = find_system(modelName, 'LookUnderMasks', 'none', 'FollowLinks', 'off', ...
    'BlockType', 'SubSystem');
for k = 1:numel(subs)
    s = subs{k};
    if is_skippable(s), continue; end
    depth = numel(strfind(s, '/'));
    nm = regexprep(get_param(s, 'Name'), '[^\w]', '_');
    f = fullfile(sdir, sprintf('%02d_L%d_%s.png', k, depth, nm));
    try
        open_system(s);
        print_sys(s, f);
        close_system(s);
        idx(end+1, :) = {['simulink/' file_name(f)], s}; %#ok<AGROW>
    catch ME
        fprintf(2, 'Could not print %s: %s\n', s, ME.message);
    end
end
fprintf('Simulink diagrams saved: %d\n', size(idx, 1));

%% 2. Run the four demos and save every figure ------------------------------
runs = {'run_Validation_LGHG2', 'run_SOC_Estimation_Test', ...
        'run_Extreme_Heat_Scenarios', 'run_Indian_Climate_Scenarios'};
sizes = {[1400 1900], [1500 1700], [1700 3000], [1500 1000]};
for r = 1:numel(runs)
    close all force;
    fprintf('\n================ %s ================\n', runs{r});
    t0 = tic;
    try
        evalin('base', [runs{r} ';']);
    catch ME
        fprintf(2, '%s FAILED:\n%s\n', runs{r}, ME.getReport('extended', 'hyperlinks', 'off'));
        continue;
    end
    fprintf('%s finished in %.1f min\n', runs{r}, toc(t0)/60);
    figs = findall(0, 'Type', 'figure');
    [~, order] = sort(arrayfun(@(h) h.Number, figs));
    figs = figs(order);
    for j = 1:numel(figs)
        fg = figs(j);
        base = sprintf('%d%d_%s', r, j, regexprep(fg.Name, '[^\w]', '_'));
        base = base(1:min(end, 60));
        sz = sizes{r};
        if j > 1 && r ~= 3, sz = [1500 1000]; end
        set(fg, 'Visible', 'off', 'Units', 'pixels', 'Position', [20 20 sz]);
        drawnow;
        f = fullfile(rdir, [base '.png']);
        save_fig(fg, f, 200);
        idx(end+1, :) = {['results/' file_name(f)], fg.Name}; %#ok<AGROW>
        try, savefig(fg, fullfile(rdir, [base '.fig'])); catch, end
        % every panel on its own, for the report pages
        ax = findall(fg, 'Type', 'axes');
        ax = flipud(ax);                                   % creation order
        for a = 1:numel(ax)
            try
                nf = figure('Visible', 'off', 'Color', 'w', 'Units', 'pixels', ...
                    'Position', [20 20 1100 500]);
                lg = ax(a).Legend;
                if ~isempty(lg)
                    obj = copyobj([lg, ax(a)], nf);
                    na = obj(2);
                else
                    na = copyobj(ax(a), nf);
                end
                set(na, 'Units', 'normalized', 'Position', [0.08 0.14 0.88 0.76], 'FontSize', 12);
                ttl = strtrim(strjoin(cellstr(na.Title.String), ' '));
                f = fullfile(rdir, sprintf('%s_p%02d.png', base, a));
                save_fig(nf, f, 200);
                idx(end+1, :) = {['results/' file_name(f)], [fg.Name ' | ' ttl]}; %#ok<AGROW>
                close(nf);
            catch ME
                try                                   % fall back: export the axes as drawn
                    f = fullfile(rdir, sprintf('%s_p%02d.png', base, a));
                    exportgraphics(ax(a), f, 'Resolution', 300, 'BackgroundColor', 'white');
                    ttl = strtrim(strjoin(cellstr(ax(a).Title.String), ' '));
                    idx(end+1, :) = {['results/' file_name(f)], [fg.Name ' | ' ttl]}; %#ok<AGROW>
                catch ME2
                    fprintf(2, 'panel %d of %s: %s / %s\n', a, fg.Name, ME.message, ME2.message);
                end
            end
        end
    end
end

%% 3. Result tables -------------------------------------------------------
for fn = {'soc_test_results.csv', 'bms_warnings_test.csv', 'validation_results.csv', ...
          'extreme_heat_results.mat'}
    if exist(fullfile(here, fn{1}), 'file')
        copyfile(fullfile(here, fn{1}), fullfile(outdir, fn{1}));
    end
end
T = cell2table(idx, 'VariableNames', {'file', 'what'});
writetable(T, fullfile(outdir, 'figure_index.csv'));
fprintf('\nDONE. %d images saved in %s\n', size(idx, 1), outdir);
end

% -------------------------------------------------------------------------
function tf = is_skippable(s)
% MATLAB Function blocks, Stateflow charts, library blocks and masked blocks
tf = false;
try
    if ~strcmp(get_param(s, 'SFBlockType'), 'NONE'), tf = true; return; end
catch
end
try
    if ~strcmp(get_param(s, 'LinkStatus'), 'none'), tf = true; return; end
catch
end
try
    if strcmp(get_param(s, 'Mask'), 'on'), tf = true; return; end
catch
end
end

function print_sys(s, f)
print(['-s' s], '-dpng', '-r250', f);
end

function save_fig(fg, f, res)
if exist('exportgraphics', 'file')
    exportgraphics(fg, f, 'Resolution', res, 'BackgroundColor', 'white');
else
    set(fg, 'PaperPositionMode', 'auto', 'InvertHardcopy', 'off');
    print(fg, f, '-dpng', sprintf('-r%d', res));
end
end

function n = file_name(f)
[~, a, b] = fileparts(f);
n = [a b];
end
