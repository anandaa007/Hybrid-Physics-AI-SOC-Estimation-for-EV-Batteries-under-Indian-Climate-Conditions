%% =========================================================================
%% run_Extreme_Heat_Scenarios.m
%% Rajasthan-type extreme heat: what the battery, the SOC estimator and the
%% BMS do at 40-60 C ambient. Simulation with EV_BMS_SOC_Estimation_System
%% (live sensors -> EKF + CNN + adaptive fusion -> BMS protection).
%%
%% NOTE  The cell model is fitted to measured data up to 40 C; above that its
%%       resistances follow the fitted Arrhenius law (extrapolation), OCV and
%%       hysteresis are held at the 40 C values. The CNN was trained at
%%       -10..25 C, so in this heat it is out of distribution and the adaptive
%%       fusion is expected to shift its weight to the EKF.
%% Press F5 in this folder.
%% =========================================================================
clear; clc;
here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
cd(here);
modelName = 'EV_BMS_SOC_Estimation_System';
run(fullfile(here, 'ET_Battery_Params.m'));
ET0 = ET;

% name, ambient C, initial pack temperature C (NaN = ambient), vehicle,
% MIDC cycles, grade %, Ru factor, constant charge current per cell (0 = drive)
S = {
 'Jaipur summer drive, 40 C',                 40, NaN, '4W', 4, 0, 1,  0
 'Delhi heat wave drive, 45 C',               45, NaN, '4W', 4, 0, 1,  0
 'Rajasthan heat wave drive, 50 C',           50, NaN, '4W', 4, 0, 1,  0
 'Thar desert, 55 C',                         55, NaN, '4W', 4, 0, 1,  0
 'Extreme 60 C (BMS must stop)',              60, NaN, '4W', 4, 0, 1,  0
 'Parked in sun: pack 58 C, then drive at 45 C', 45,  58, '4W', 4, 0, 1,  0
 'Scooter, sealed pack, 6% climb, 48 C',      48, NaN, '2W', 4, 6, 3,  0
 'Fast charge 1.3C at 45 C ambient',          45, NaN, '4W', 0, 0, 1, -4
 'Fast charge 1.3C at 50 C ambient',          50, NaN, '4W', 0, 0, 1, -4
};
ensure_model_built(modelName);
load_system(fullfile(here, [modelName '.slx']));
set_param(modelName, 'StopTime', 'SCN.t_end');   % each scenario runs for its own length

n = size(S, 1);
R = struct([]);
fig = figure('Name', 'Extreme heat: battery, SOC estimator and BMS', 'Color', 'w');
tl = tiledlayout(fig, n, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
for k = 1:n
    ET = apply_cooling(ET0, S{k,7});
    if S{k,8} < 0
        tt = (0:1700)';      % 28 min CC charge from 20 % SOC (ends before the 4.2 V limit)
        [VEH, PACK, BMS, SCN] = ev_scenario(S{k,4}, S{k,2}, 1, ...
            struct('mode', 2, 'Icell', [tt, S{k,8}*ones(size(tt))], 'soc0', 0.20));
    else
        [VEH, PACK, BMS, SCN] = ev_scenario(S{k,4}, S{k,2}, S{k,5}, ...
            struct('grade_pct', S{k,6}, 'soc0', 0.95));
    end
    if ~isnan(S{k,3}), SCN.T0 = S{k,3}; end
    out = sim(modelName);
    so = out.get('log_soc'); th = out.get('log_thermal'); bm = out.get('log_bms');
    t = so.Time; Ys = tsmat(so);
    Th = interp1(th.Time, tsmat(th), t, 'previous', 'extrap');
    B  = interp1(bm.Time, tsmat(bm), t, 'previous', 'extrap');
    ok = t >= 240;
    Ys(~ok, 9) = NaN;                                    % CNN has no output before its first 240 s window
    e = 100*(Ys(ok, [8 9 2]) - Ys(ok, 1));
    R(k).name = S{k,1};
    R(k).rmse = sqrt(mean(e.^2, 1));                     % EKF, CNN, adaptive
    R(k).alpha = mean(Ys(ok, 4));
    R(k).Tc_max = max(Th(:,3)); R(k).Ts_max = max(Th(:,2));
    R(k).warn = 100*mean(B(:,3) > 0); R(k).derate = 100*mean(B(:,4) > 0);
    R(k).shut = 100*mean(B(:,5) > 0);
    R(k).Ilim_min = min(B(:,1)) / PACK.Np;               % per cell
    tm = t/60;
    % SOC
    ax = nexttile(tl); hold(ax, 'on');
    plot(ax, tm, 100*Ys(:,1), 'k', 'LineWidth', 1.6);
    plot(ax, tm, 100*Ys(:,8), '--', tm, 100*Ys(:,9), ':', tm, 100*Ys(:,2), 'LineWidth', 1.0);
    grid(ax, 'on'); ylabel(ax, 'SOC %'); xlim(ax, [0 tm(end)]);
    title(ax, sprintf('%s | fusion RMSE %.2f %% (alpha_{EKF} %.2f)', S{k,1}, R(k).rmse(3), R(k).alpha), ...
        'FontSize', 8, 'Interpreter', 'tex');
    if k == 1, legend(ax, 'true', 'EKF', 'CNN', 'fusion', 'Location', 'best'); end
    % temperatures + warning bands
    ax = nexttile(tl); hold(ax, 'on');
    shade(ax, tm, B(:,3) > 0, [1.0 0.85 0.4]);
    shade(ax, tm, B(:,4) > 0, [1.0 0.6 0.3]);
    shade(ax, tm, B(:,5) > 0, [0.9 0.2 0.2]);
    plot(ax, tm, Th(:,1), 'b:', tm, Th(:,2), 'k', tm, Th(:,3), 'r', tm, Ys(:,10), 'm--', 'LineWidth', 1.0);
    yline(ax, 45, ':', 'warn'); yline(ax, 50, '--', 'derate'); yline(ax, 60, '-', 'cut-off');
    grid(ax, 'on'); ylabel(ax, 'T (C)'); xlim(ax, [0 tm(end)]); ylim(ax, [min(Th(:)) - 2, 66]);
    title(ax, sprintf('T_{core} max %.1f C', R(k).Tc_max), 'FontSize', 8);
    if k == 1, legend(ax, 'ambient', 'surface', 'core (plant)', 'core (EKF est.)', 'Location', 'best'); end
    % BMS current limit
    ax = nexttile(tl); hold(ax, 'on');
    plot(ax, tm, B(:,1)/PACK.Np, 'r', tm, -B(:,2)/PACK.Np, 'b', tm, out_cell_current(out, t), 'k', 'LineWidth', 1.0);
    grid(ax, 'on'); ylabel(ax, 'A per cell'); xlim(ax, [0 tm(end)]);
    title(ax, sprintf('BMS: warn %.0f%% | derate %.0f%% | shut-down %.0f%% of time', ...
        R(k).warn, R(k).derate, R(k).shut), 'FontSize', 8);
    if k == 1, legend(ax, 'discharge limit', 'charge limit', 'cell current', 'Location', 'best'); end
    fprintf('%-46s Tcore max %5.1f C | warn %5.1f%% derate %5.1f%% shutdown %5.1f%% | SOC RMSE EKF %.2f CNN %.2f fusion %.2f (alpha %.2f)\n', ...
        S{k,1}, R(k).Tc_max, R(k).warn, R(k).derate, R(k).shut, R(k).rmse, R(k).alpha);
end
xlabel(tl, 'time (min)');
title(tl, 'Extreme heat (simulation): SOC estimation | cell temperatures with BMS warning bands | BMS current limits');
save(fullfile(here, 'extreme_heat_results.mat'), 'R', 'S');

function i = out_cell_current(out, t)
c = out.get('log_cell');
Dc = tsmat(c); i = interp1(c.Time, Dc(:,1), t, 'previous', 'extrap');
end

function shade(ax, t, on, colr)
on = on(:)'; d = diff([0 on 0]);
s = find(d == 1); e = find(d == -1) - 1;
for k = 1:numel(s)
    patch(ax, [t(s(k)) t(e(k)) t(e(k)) t(s(k))], [-40 -40 80 80], colr, ...
        'EdgeColor', 'none', 'FaceAlpha', 0.35, 'HandleVisibility', 'off');
end
end
