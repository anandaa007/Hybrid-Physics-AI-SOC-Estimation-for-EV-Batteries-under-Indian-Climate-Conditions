function [VEH, PACK, BMS, SCN] = ev_scenario(vehicle, T_amb_C, n_cycles, opts)
%EV_SCENARIO  Vehicle, pack, BMS and driving scenario for the EV BMS model.
%   [VEH,PACK,BMS,SCN] = ev_scenario('4W', 45, 5)
%   vehicle : '4W' compact electric car (Nexon-EV class, ~31 kWh pack)
%             '2W' electric scooter (~3 kWh pack)
%   T_amb_C : ambient temperature, degC (scalar) or [t_s, T_C] profile
%   n_cycles: number of MIDC cycles driven back to back
%   opts    : optional struct: grade_pct, soc0, SOH0, aux_W, mode, Icell
%             mode = 1 vehicle drive cycle (default), 2 measured cell current
if nargin < 4, opts = struct(); end
g = @(f, d) getfield_default(opts, f, d);

switch upper(vehicle)
    case '4W'
        VEH = struct('name', 'Compact e-car (Nexon-EV class)', 'm', 1500, ...
            'Cd', 0.33, 'A', 2.35, 'Crr', 0.013, 'rho', 1.18, 'kr', 1.05, ...
            'eta_drive', 0.88, 'eta_regen', 0.65, 'P_regen_max', 30e3, ...
            'P_trac_max', 95e3, 'v_regen_min', 5);
        PACK = struct('Ns', 96, 'Np', 30);
        urban = false;
        aux = hvac_load_W(T_amb_C, 400, 2500);
    case '2W'
        VEH = struct('name', 'Electric scooter', 'm', 195, ...
            'Cd', 0.70, 'A', 0.85, 'Crr', 0.016, 'rho', 1.18, 'kr', 1.03, ...
            'eta_drive', 0.85, 'eta_regen', 0.50, 'P_regen_max', 2e3, ...
            'P_trac_max', 6e3, 'v_regen_min', 5);
        PACK = struct('Ns', 20, 'Np', 14);
        urban = true;
        aux = 60;
    otherwise
        error('vehicle must be ''4W'' or ''2W''');
end
VEH.P_aux = g('aux_W', aux);
VEH.vec = [VEH.m; VEH.Cd; VEH.A; VEH.Crr; VEH.rho; VEH.kr; VEH.eta_drive; ...
           VEH.eta_regen; VEH.P_regen_max; VEH.P_aux; VEH.v_regen_min; VEH.P_trac_max];

% ---- BMS limits (per cell; LG HG2 datasheet: 20 A cont. discharge, 4 A std charge)
BMS = struct('I_dis_cont', 20, 'I_chg_cont', 4, ...
    'T_warn', 45, 'T_derate', 50, 'T_cutoff', 60, ...       % high-temperature protection
    'T_chg_min', 0, 'T_chg_low', 10, 'T_chg_max', 50, ...   % charging window
    'T_cold', -20, 'V_max', 4.20, 'V_min', 2.80, 'SOC_min', 0.05, ...
    'enable', g('bms_enable', 1));
BMS.vec = [BMS.I_dis_cont; BMS.I_chg_cont; BMS.T_warn; BMS.T_derate; ...
    BMS.T_cutoff; BMS.T_chg_min; BMS.T_chg_low; BMS.T_chg_max; BMS.T_cold; ...
    BMS.V_max; BMS.V_min; BMS.SOC_min; PACK.Np; BMS.enable];

% ---- driving scenario
mode = g('mode', 1);
if mode == 1
    [t, v] = make_MIDC(n_cycles, urban);
else
    Ic = g('Icell', [0 0; 1 0]);
    t = Ic(:, 1); v = zeros(size(t));
end
SCN = struct();
SCN.mode = mode;
SCN.speed = [t, v];
gp = g('grade_pct', 0);
if isscalar(gp), SCN.grade = [t, gp * ones(size(t))]; else, SCN.grade = gp; end
if isscalar(T_amb_C), SCN.Tamb = [t, T_amb_C * ones(size(t))]; else, SCN.Tamb = T_amb_C; end
if mode == 2, SCN.Icell = g('Icell', [0 0; 1 0]); else, SCN.Icell = [t, zeros(size(t))]; end
SCN.soc0 = g('soc0', 0.95);
SCN.T0 = SCN.Tamb(1, 2);                 % cell starts at ambient (soaked)
SCN.SOH0 = g('SOH0', 1.0);
SCN.t_end = t(end);
% ---- BMS sensors (board of the project: INA240 + shunt, INA226, DS18B20)
SCN.i_gain = 0.005;  SCN.i_offset = 0.02;  SCN.i_noise = 0.01;   % 0.5 %, 20 mA, 10 mA rms
SCN.v_noise = 0.002; SCN.v_lsb = 1.25e-3;                        % 2 mV rms, INA226 LSB
SCN.t_offset = 0.3;  SCN.t_lsb = 0.0625;                         % DS18B20 offset, LSB
% ---- SOC estimator: live sensors (1) or recorded LG HG2 test data (2)
SCN.est_source = g('est_source', 1);
% The estimator's SOC is referred to 3.15 Ah (the prepared LG HG2 data it was
% trained on); the plant's to its 2.78 Ah C/20 capacity. Same charge, two
% scales: SOC_est = 1 - (1 - SOC_plant) * 2.78/3.15.
SCN.Cn_ratio = g('Cn_ratio', 2.7798/3.15);
% the model always references the recorded-data inputs; give them a default
if evalin('base', 'exist(''REC'', ''var'')') == 0
    assignin('base', 'REC', struct('I', [0 0; 1 0], 'V', [0 3.7; 1 3.7], ...
        'T', [0 25; 1 25], 'SOC', [0 0.5; 1 0.5]));
end
end

function P = hvac_load_W(T, base, peak)
% Cabin HVAC + auxiliaries: grows with distance from 22 degC
% (air-conditioning in Indian summer, heating in cold regions).
if ~isscalar(T), T = mean(T(:, 2)); end
P = base + min(peak, 75 * abs(T - 22));
end

function v = getfield_default(s, f, d)
if isfield(s, f), v = s.(f); else, v = d; end
end
