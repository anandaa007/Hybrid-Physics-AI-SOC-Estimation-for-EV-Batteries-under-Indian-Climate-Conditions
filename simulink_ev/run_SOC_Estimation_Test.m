%% =========================================================================
%% run_SOC_Estimation_Test.m   --  THE PROJECT DEMO (Simulink)
%% Runs EV_BMS_SOC_Estimation_System on held-out MEASURED LG 18650HG2 data:
%%   * the dataset's four test files at -10, 0, 10, 25 C (UDDS + LA92 + US06
%%     discharges with recharges in between), and
%%   * the extreme-temperature recordings of the same cycles at 40 C (UDDS,
%%     LA92, US06) and -20 C (UDDS).
%% None of these cycles was used to train the CNN or identify the models.
%% The estimator (electro-thermal EKF + 1-D CNN + adaptive fusion) runs at
%% 1 Hz exactly as on the BMS. Figure 1: SOC tracking. Figure 2: cell
%% temperatures and the BMS warnings raised during each test.
%% Press F5 in this folder.
%% =========================================================================
clear; clc;
here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
cd(here);
modelName = 'EV_BMS_SOC_Estimation_System';
rmsf = @(x) sqrt(mean(x(:).^2, 'omitnan'));
run(fullfile(here, 'ET_Battery_Params.m'));
D = load(fullfile(here, 'LGHG2_SOC_test_recordings.mat'));
c = D.case1;
REC = struct('I', [c.t c.I], 'V', [c.t c.V], 'T', [c.t c.T], 'SOC', [c.t c.SOC_ref]);
[VEH, PACK, BMS, SCN] = ev_scenario('4W', 25, 1, struct('mode', 2, 'Icell', REC.I, ...
    'est_source', 2, 'bms_enable', 0));
ensure_model_built(modelName);
load_system(fullfile(here, [modelName '.slx']));
restoreStop = onCleanup(@() set_param(modelName, 'StopTime', 'SCN.t_end'));   % other scripts use the scenario length

names = {'CC', 'EKF', 'CNN', 'Fixed', 'Adaptive'};
col   = [7 8 9 3 2];                        % columns of log_soc
n = double(D.n_cases);
RM = zeros(n, 5); MA = zeros(n, 5); Tn = zeros(n, 1); W = zeros(n, 7); AL = zeros(n, 1);
f1 = figure('Name', 'SOC estimation on held-out measured data', 'Color', 'w');
t1 = tiledlayout(f1, ceil(n/2), 2, 'TileSpacing', 'compact', 'Padding', 'compact');
f2 = figure('Name', 'Cell temperature and BMS warnings', 'Color', 'w');
t2 = tiledlayout(f2, ceil(n/2), 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for k = 1:n
    c = D.(sprintf('case%d', k));
    REC = struct('I', [c.t c.I], 'V', [c.t c.V], 'T', [c.t c.T], 'SOC', [c.t c.SOC_ref]);
    [VEH, PACK, BMS, SCN] = ev_scenario('4W', [c.t c.T], 1, struct('mode', 2, 'Icell', REC.I, ...
        'est_source', 2, 'bms_enable', 0, 'soc0', 0.95));
    PACK.Ns = 1; PACK.Np = 1; BMS.vec(13) = 1;
    SCN.T0 = c.T(1);
    set_param(modelName, 'StopTime', num2str(c.t(end)));
    out = sim(modelName);
    y = out.get('log_soc');  b = out.get('log_bms');
    Y = interp1(y.Time, tsmat(y), c.t, 'previous', 'extrap');
    B = interp1(b.Time, tsmat(b), c.t, 'previous', 'extrap');
    ref = Y(:,1);
    early = c.t < 239;                      % no full CNN window yet: CNN falls back to EKF
    Y(early, 9) = Y(early, 8);
    for m = 1:5
        e = 100*(Y(:,col(m)) - ref);
        RM(k,m) = rmsf(e); MA(k,m) = mean(abs(e), 'omitnan');
    end
    Tn(k) = c.T_nominal;
    AL(k) = mean(Y(~early, 4));
    % BMS warnings (computed inside the model from the cell temperatures)
    chg = c.I < 0;
    W(k,:) = 100*[mean(B(:,3) > 0), mean(B(:,4) > 0), mean(B(:,5) > 0), ...
                  mean(B(:,6) > 0 & chg), mean(B(:,7) > 0), mean(B(:,8) > 0), mean(B(:,9) > 0)];
    out_range = Tn(k) < -10 || Tn(k) > 25;
    th = c.t/3600;
    % ---- Figure 1: SOC
    ax = nexttile(t1); hold(ax, 'on');
    plot(ax, th, 100*ref, 'k', 'LineWidth', 1.8);
    plot(ax, th, 100*Y(:,8), '--', 'Color', [0.92 0.41 0.20], 'LineWidth', 1.0);
    plot(ax, th, 100*Y(:,9), ':', 'Color', [0.93 0.63 0.00], 'LineWidth', 1.0);
    plot(ax, th, 100*Y(:,2), '-', 'Color', [0.38 0.31 0.84], 'LineWidth', 1.3);
    xlim(ax, [0 th(end)]); ylim(ax, [0 105]); grid(ax, 'on'); ylabel(ax, 'SOC (%)');
    ttl = sprintf('%+g C  %s:  fusion RMSE %.2f %%', Tn(k), shortname(c.name), RM(k,5));
    if out_range, ttl = [ttl '  (outside CNN training range)']; end
    title(ax, ttl, 'Interpreter', 'none', 'FontSize', 9);
    if k == 1, legend(ax, 'reference', 'EKF', '1D CNN', 'adaptive fusion', 'Location', 'southwest'); end
    % ---- Figure 2: temperatures + warnings
    ax = nexttile(t2); hold(ax, 'on');
    shade(ax, th, B(:,3) > 0, [1.0 0.85 0.4]);              % over-temperature warning
    shade(ax, th, B(:,4) > 0, [1.0 0.6 0.3]);               % derating
    shade(ax, th, B(:,6) > 0 & chg, [0.6 0.8 1.0]);         % charging below 0 C
    plot(ax, th, c.T, 'k', 'LineWidth', 1.2);
    plot(ax, th, Y(:,10), 'r', 'LineWidth', 1.0);
    yline(ax, BMS.T_warn, ':', 'warn 45 C', 'Color', [0.8 0.5 0]);
    yline(ax, BMS.T_derate, '--', 'derate 50 C', 'Color', [0.9 0.3 0]);
    yline(ax, BMS.T_chg_min, '--', 'no charging < 0 C', 'Color', [0.2 0.4 0.9]);
    xlim(ax, [0 th(end)]); grid(ax, 'on'); ylabel(ax, 'T (C)');
    title(ax, sprintf('%+g C: surface (measured) and core (EKF estimate)', Tn(k)), 'FontSize', 9);
    if k == 1, legend(ax, 'surface (DS18B20)', 'core (EKF)', 'Location', 'best'); end
end
xlabel(t1, 'time (h)'); xlabel(t2, 'time (h)');
title(t1, 'Held-out measured data: reference SOC vs EKF, 1D CNN and adaptive fusion');
title(t2, 'Cell temperature and BMS warnings (yellow: >45 C warning, orange: derating, blue: regen/charge blocked below 0 C)');

T = array2table([Tn RM AL], 'VariableNames', [{'Temp_C'}, strcat('RMSE_', names), {'mean_alpha_EKF'}]);
fprintf('\nSOC RMSE (%%) on held-out measured data:\n'); disp(T);
in = Tn >= -10 & Tn <= 25;
fprintf('Mean RMSE, -10..25 C test files : CC %.2f | EKF %.2f | CNN %.2f | fixed %.2f | adaptive %.2f\n', mean(RM(in,:), 1));
fprintf('Mean RMSE, 40 C recordings      : CC %.2f | EKF %.2f | CNN %.2f | fixed %.2f | adaptive %.2f\n', mean(RM(Tn == 40,:), 1));
fprintf('Mean RMSE, -20 C recordings     : CC %.2f | EKF %.2f | CNN %.2f | fixed %.2f | adaptive %.2f\n', mean(RM(Tn == -20,:), 1));
Wt = array2table([Tn W], 'VariableNames', {'Temp_C', 'OT_warn_pct', 'OT_derate_pct', 'OT_shutdown_pct', ...
    'charge_below_0C_pct', 'over_voltage_pct', 'under_voltage_pct', 'low_SOC_pct'});
fprintf('\nBMS warnings (percent of test time):\n'); disp(Wt);
writetable(T, fullfile(here, 'soc_test_results.csv'));
writetable(Wt, fullfile(here, 'bms_warnings_test.csv'));

function s = shortname(nm)
s = regexprep(strtrim(nm), '^.*?\d{3}_', '');
s = regexprep(s, '_LGHG2.*$|\.mat$|_Norm.*$', '');
end

function shade(ax, t, on, colr)
% shaded vertical bands where 'on' is true
on = on(:)'; d = diff([0 on 0]);
s = find(d == 1); e = find(d == -1) - 1;
yl = [-30 70];
for k = 1:numel(s)
    patch(ax, [t(s(k)) t(e(k)) t(e(k)) t(s(k))], [yl(1) yl(1) yl(2) yl(2)], colr, ...
        'EdgeColor', 'none', 'FaceAlpha', 0.35, 'HandleVisibility', 'off');
end
end
