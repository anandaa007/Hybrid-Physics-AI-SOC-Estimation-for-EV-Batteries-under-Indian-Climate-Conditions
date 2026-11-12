%% =========================================================================
%% run_Validation_LGHG2.m
%% Checks the battery model in EV_BMS_SOC_Estimation_System against MEASURED
%% LG 18650HG2 data that were held out of the identification (first 2 h of
%% the UDDS drive cycle at each chamber temperature, -20 .. 40 C). The model is driven by the measured cell
%% current (mode 2, one cell, BMS limits off) and the chamber temperature;
%% its terminal voltage and surface temperature are compared with the
%% measurement, and with the Python model as a cross-check.
%% =========================================================================
clear; clc;
here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
cd(here);
modelName = 'EV_BMS_SOC_Estimation_System';
rmsf = @(x) sqrt(mean(x(:).^2, 'omitnan'));
run(fullfile(here, 'ET_Battery_Params.m'));
D = load(fullfile(here, 'LGHG2_validation_cases.mat'));
ensure_model_built(modelName);
load_system(fullfile(here, [modelName '.slx']));
set_param(modelName, 'StopTime', 'SCN.t_end');   % each scenario runs for its own length

n = double(D.n_cases);
T = zeros(n, 6);
figure('Name', 'Validation against measured LG HG2 data', 'Color', 'w');
for k = 1:n
    c = D.(sprintf('case%d', k));
    Icell = [c.t, c.I];
    [VEH, PACK, BMS, SCN] = ev_scenario('4W', [c.t, c.Tamb], 1, ...
        struct('mode', 2, 'Icell', Icell, 'soc0', c.soc0, 'bms_enable', 0));
    PACK.Ns = 1; PACK.Np = 1; BMS.vec(13) = 1;            % one cell
    SCN.T0 = c.T0;
    out = sim(modelName);
    cl = out.get('log_cell'); th = out.get('log_thermal');
    Dcl = tsmat(cl); Dth = tsmat(th);
    V  = interp1(cl.Time, Dcl(:,2), c.t);
    Ts = interp1(th.Time, Dth(:,2), c.t);
    Tc = interp1(th.Time, Dth(:,3), c.t);
    T(k,:) = [c.T_chamber, 1e3*rmsf(V - c.V_meas), rmsf(Ts - c.Ts_meas), ...
              max(c.Ts_meas) - c.T_chamber, 1e3*rmsf(V - c.py_v), max(Tc - Ts)];
    subplot(n, 2, 2*k-1);
    plot(c.t/60, c.V_meas, 'k', c.t/60, V, 'r--', 'LineWidth', 1); grid on;
    ylabel('V'); title(sprintf('%+g C: terminal voltage', c.T_chamber));
    if k == 1, legend('measured', 'Simulink model'); end
    subplot(n, 2, 2*k);
    plot(c.t/60, c.Ts_meas, 'k', c.t/60, Ts, 'r--', c.t/60, Tc, 'm:', 'LineWidth', 1); grid on;
    ylabel('C'); title(sprintf('%+g C: cell temperature', c.T_chamber));
    if k == 1, legend('surface measured', 'surface model', 'core model'); end
end
xlabel('time (min)');
tbl = array2table(T, 'VariableNames', {'Chamber_C', 'V_RMSE_mV', 'Tsurf_RMSE_C', ...
    'SelfHeating_meas_C', 'Simulink_vs_Python_mV', 'Core_minus_surface_max_C'});
disp(tbl);
writetable(tbl, fullfile(here, 'validation_results.csv'));
