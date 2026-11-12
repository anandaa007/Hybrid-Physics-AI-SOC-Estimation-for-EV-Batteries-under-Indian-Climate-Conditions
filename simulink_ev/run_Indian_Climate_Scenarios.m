%% =========================================================================
%% run_Indian_Climate_Scenarios.m
%% Runs EV_BMS_ElectroThermal_Model over Indian climate / road scenarios and
%% plots what the BMS sees: temperatures, SOC, energy use, derating, SOH.
%% Press F5 in this folder. Builds the model first if it is missing.
%% =========================================================================
clear; clc;
here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
cd(here);
modelName = 'EV_BMS_SOC_Estimation_System';
run(fullfile(here, 'ET_Battery_Params.m'));            % cell model -> ET

%% Scenario table --------------------------------------------------------
% name, ambient [C], vehicle, MIDC cycles, road grade [%], pack Ru factor,
% charge current per cell [A] (0 = drive the MIDC; <0 = constant-current charge)
S = {
 'Leh-Ladakh winter (-10 C)',          -10, '4W', 6, 0, 1, 0
 'North-India winter morning (0 C)',     0, '4W', 6, 0, 1, 0
 'Bengaluru (25 C)',                    25, '4W', 6, 0, 1, 0
 'Chennai / Mumbai summer (35 C)',      35, '4W', 6, 0, 1, 0
 'Delhi summer (45 C)',                 45, '4W', 6, 0, 1, 0
 'Rajasthan heat wave (50 C)',          50, '4W', 6, 0, 1, 0
 'Parked in sun, pack at 55 C',         55, '4W', 6, 0, 1, 0
 'Extreme pack 60 C',                   60, '4W', 6, 0, 1, 0
 'Western Ghats climb 6% (35 C)',       35, '4W', 3, 6, 1, 0
 'Scooter, Delhi summer, sealed pack',  45, '2W', 6, 0, 3, 0
 'Fast charge 1.3C at 25 C',            25, '4W', 0, 0, 1, -4
 'Fast charge 1.3C at 45 C',            45, '4W', 0, 0, 1, -4
 'Charging at -10 C (BMS must block)', -10, '4W', 0, 0, 1, -4
};
ET0 = ET;
ensure_model_built(modelName);
load_system(fullfile(here, [modelName '.slx']));
set_param(modelName, 'StopTime', 'SCN.t_end');   % each scenario runs for its own length

R = struct([]);
for k = 1:size(S, 1)
    ET = apply_cooling(ET0, S{k,6});
    if S{k,7} < 0                         % constant-current charging
        tt = (0:1700)';      % 28 min CC charge from 20 % SOC (ends before the 4.2 V limit)
        [VEH, PACK, BMS, SCN] = ev_scenario(S{k,3}, S{k,2}, 1, ...
            struct('mode', 2, 'Icell', [tt, S{k,7}*ones(size(tt))], 'soc0', 0.20));
    else
        [VEH, PACK, BMS, SCN] = ev_scenario(S{k,3}, S{k,2}, S{k,4}, ...
            struct('grade_pct', S{k,5}, 'soc0', 0.95));
    end
    out = sim(modelName);
    v  = out.get('log_vehicle');  c = out.get('log_cell');
    th = out.get('log_thermal');  q = out.get('log_heat');  b = out.get('log_bms');
    pk = out.get('log_pack');
    Dv = tsmat(v); Dc = tsmat(c); Dth = tsmat(th); Dq = tsmat(q); Db = tsmat(b); Dp = tsmat(pk);
    t = v.Time / 60;
    R(k).name = S{k,1}; R(k).t = t;
    R(k).v = Dv(:,1); R(k).P = Dv(:,2); R(k).km = Dv(end,3); R(k).kWh = Dv(end,4);
    R(k).I = Dc(:,1); R(k).V = Dc(:,2); R(k).SOC = Dc(:,3); R(k).SOH = Dc(:,4);
    R(k).Tamb = Dth(:,1); R(k).Ts = Dth(:,2); R(k).Tc = Dth(:,3);
    R(k).Q = Dq;  R(k).Idis = Db(:,1); R(k).flags = Db(:,3:9);
    E_req = trapz(v.Time, max(Dv(:,2), 0)) / 3.6e6;            % kWh asked by the drive
    E_del = trapz(pk.Time, max(Dp(:,3), 0)) / 3.6e6;           % kWh the BMS allowed
    R(k).unmet = 100 * max(E_req - E_del, 0) / max(E_req, 1e-9);
    so = out.get('log_soc'); Ys = tsmat(so);
    ok = so.Time >= 240;                                   % after the first CNN window
    R(k).soc_rmse = 100*sqrt(mean((Ys(ok, [8 9 2]) - Ys(ok, 1)).^2, 1));   % EKF, CNN, adaptive
    fprintf('%-36s %6.1f km | %6.1f Wh/km | SOC %5.1f%% -> %5.1f%% | Tcore max %5.1f C | derated %5.1f%% of time | energy refused by BMS %5.1f%%\n', ...
        S{k,1}, R(k).km, 1000*R(k).kWh/max(R(k).km,1e-3), 100*R(k).SOC(1), 100*R(k).SOC(end), ...
        max(R(k).Tc), 100*mean(R(k).flags(:,2) > 0), R(k).unmet);
    fprintf('%-36s SOC estimation RMSE: EKF %.2f %% | CNN %.2f %% | adaptive fusion %.2f %%\n', '', R(k).soc_rmse);
end
save(fullfile(here, 'scenario_results.mat'), 'R', 'S');

%% Figures ----------------------------------------------------------------
cm = parula(size(S,1) + 1);
f1 = figure('Name', 'Temperatures across Indian climates', 'Color', 'w');
for k = 1:size(S,1)
    subplot(2,1,1); plot(R(k).t, R(k).Tc, 'Color', cm(k,:), 'LineWidth', 1.3); hold on;
    subplot(2,1,2); plot(R(k).t, 100*R(k).SOC, 'Color', cm(k,:), 'LineWidth', 1.3); hold on;
end
subplot(2,1,1); ylabel('Cell core temperature (C)'); grid on;
yline(BMS.T_derate, 'r--', 'derating starts'); yline(BMS.T_cutoff, 'r-', 'BMS cut-off');
title('Core temperature over repeated MIDC driving'); legend(S(:,1), 'Location', 'eastoutside');
subplot(2,1,2); ylabel('SOC (%)'); xlabel('time (min)'); grid on;

f2 = figure('Name', 'Heat generation breakdown', 'Color', 'w');
k = 5;                                   % Delhi summer
area(R(k).t, R(k).Q(:,1:3)); hold on; plot(R(k).t, R(k).Q(:,4), 'k', 'LineWidth', 1);
legend('ohmic I^2R_0', 'charge transfer I\eta_{ct}', 'polarisation', 'reversible (entropic)');
xlabel('time (min)'); ylabel('heat per cell (W)'); grid on;
title(['Heat sources: ' S{k,1}]);

f3 = figure('Name', 'Scenario summary', 'Color', 'w');
amb = strcat(cellfun(@num2str, S(:,2), 'UniformOutput', false), 'C');
subplot(1,3,1); bar(arrayfun(@(r) max(r.Tc), R)); set(gca, 'XTickLabel', amb); ylabel('max T_{core} (C)'); xlabel('ambient (C)'); grid on;
whkm = arrayfun(@(r) 1000*r.kWh/r.km, R); whkm([R.km] < 1) = NaN;    % charging cases have no distance
subplot(1,3,2); bar(whkm); set(gca, 'XTickLabel', amb); ylabel('energy (Wh/km)'); xlabel('ambient (C)'); grid on;
subplot(1,3,3); bar(arrayfun(@(r) 100*mean(r.flags(:,2) > 0), R)); set(gca, 'XTickLabel', amb); ylabel('time derated (%)'); xlabel('ambient (C)'); grid on;
sgtitle('EV BMS electro-thermal model: Indian climate scenarios (MIDC)');
fprintf('\nFigures drawn. Results saved to scenario_results.mat\n');
