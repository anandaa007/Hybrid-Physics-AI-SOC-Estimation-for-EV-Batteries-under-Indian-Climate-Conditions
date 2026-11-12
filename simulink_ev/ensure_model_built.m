function ensure_model_built(modelName)
%ENSURE_MODEL_BUILT  (Re)build the model when it is missing or older than
%   its sources (build script, estimator code, estimator parameters), and
%   clear Simulink's compiled cache so the new code and constants are used.
here = fileparts(mfilename('fullpath'));
slx = fullfile(here, [modelName '.slx']);
srcs = {'build_EV_BMS_SOC_Estimation_System.m', 'estimator_code.m', ...
        'SOC_Estimator_Params.mat', 'ET_Battery_Params.m'};
need = ~exist(slx, 'file');
if ~need
    d0 = dir(slx); t_slx = d0.datenum;
    for k = 1:numel(srcs)
        d = dir(fullfile(here, srcs{k}));
        if ~isempty(d) && d.datenum > t_slx, need = true; end
    end
end
if need
    fprintf('Building %s (model missing or sources updated) ...\n', modelName);
    if bdIsLoaded(modelName), close_system(modelName, 0); end
    if exist(slx, 'file')          % keep the previous model, in case it was edited by hand
        bk = fullfile(here, '_previous_models');
        if ~exist(bk, 'dir'), mkdir(bk); end
        movefile(slx, fullfile(bk, sprintf('%s_%s.slx.bak', modelName, datestr(now, 'yyyymmdd_HHMMSS'))));
    end
    if exist(fullfile(here, 'slprj'), 'dir'), rmdir(fullfile(here, 'slprj'), 's'); end
    if exist(fullfile(here, [modelName '.slxc']), 'file'), delete(fullfile(here, [modelName '.slxc'])); end
    evalin('base', 'build_EV_BMS_SOC_Estimation_System;');
end
end
