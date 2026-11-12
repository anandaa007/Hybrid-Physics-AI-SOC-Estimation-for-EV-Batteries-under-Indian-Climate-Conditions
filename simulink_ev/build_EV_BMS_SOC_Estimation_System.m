%% =========================================================================
%% build_EV_BMS_SOC_Estimation_System.m
%% -------------------------------------------------------------------------
%% Builds  EV_BMS_SOC_Estimation_System.slx  -- THE FINAL PROJECT MODEL:
%% EV + nonlinear electro-thermal battery (plant)  ->  BMS sensors  ->
%% real-time SOC estimator (electro-thermal EKF + 1-D CNN + confidence-based
%% adaptive fusion, 1 Hz, as on the STM32)  ->  BMS protection.
%%
%%  [1] Drive cycle & climate  ->  [2] EV vehicle dynamics  ->  [3] Pack current
%%      & BMS current limits   ->  [4] Nonlinear electro-thermal cell model
%%      (parameter maps at SOC x T_core, Butler-Volmer + 2-RC + hysteresis,
%%      Bernardi heat, core/surface thermal model, Xu ageing / SOH)
%%  ->  [5] Pack outputs (voltage, power, energy, Wh/km)
%%  ->  [6] BMS protection & derating (thermal, voltage, SOC)  ->  back to [3]
%%
%% Cell: LG 18650HG2 (NMC), identified from the McMaster LG HG2 data at
%%       -20, -10, 0, 10, 25 and 40 degC; Arrhenius laws extend the tables to
%%       -30 .. 80 degC (outside -20..40 degC = extrapolation, labelled).
%% Requires MATLAB + Simulink only. Tested structure: R2021a or later.
%% Run: run_Indian_Climate_Scenarios  (builds the model if it is missing)
%% =========================================================================

%% 0. Setup -----------------------------------------------------------------
modelName = 'EV_BMS_SOC_Estimation_System';
here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
cd(here);
if ~exist('ET', 'var'), run(fullfile(here, 'ET_Battery_Params.m')); end
if ~exist('SCN', 'var'), [VEH, PACK, BMS, SCN] = ev_scenario('4W', 35, 1); end
if bdIsLoaded(modelName), close_system(modelName, 0); end
if exist(fullfile(here, [modelName '.slx']), 'file')
    delete(fullfile(here, [modelName '.slx']));
end
new_system(modelName);
open_system(modelName);
set_param(modelName, 'SolverType', 'Fixed-step', 'Solver', 'ode4', ...
    'FixedStep', '0.25', 'StopTime', 'SCN.t_end', ...
    'ReturnWorkspaceOutputs', 'on', 'SignalLogging', 'on');
M = modelName;

%% 1. DRIVE CYCLE & CLIMATE ---------------------------------------------------
S1 = [M '/1_Drive_Cycle_and_Climate'];
mkSub(S1, [40 80 230 380], [0.85 0.95 1.0]);
srcs = {'Vehicle_speed_kmh', 'SCN.speed', 'on';
        'Road_grade_pct',    'SCN.grade', 'on';
        'Ambient_temp_C',    'SCN.Tamb',  'on';
        'Measured_cell_current_A', 'SCN.Icell', 'off'};
for k = 1:4
    add_block('simulink/Sources/From Workspace', [S1 '/' srcs{k,1}], ...
        'VariableName', srcs{k,2}, 'Interpolate', srcs{k,3}, ...
        'OutputAfterFinalValue', 'Holding final value', ...
        'Position', [40 40+80*(k-1) 200 70+80*(k-1)]);
    add_block('simulink/Ports & Subsystems/Out1', [S1 '/' outName(k)], ...
        'Port', num2str(k), 'Position', [300 47+80*(k-1) 330 63+80*(k-1)]);
    L(S1, [srcs{k,1} '/1'], [outName(k) '/1'], sigName(k));
end

%% 2. EV VEHICLE DYNAMICS -----------------------------------------------------
S2 = [M '/2_EV_Vehicle_Dynamics'];
mkSub(S2, [300 60 480 200], [0.9 1.0 0.85]);
inP(S2, 'v_kmh', 1, [20 50 50 64]);
inP(S2, 'grade_pct', 2, [20 150 50 164]);
add_block('simulink/Math Operations/Gain', [S2 '/kmh_to_mps'], 'Gain', '1/3.6', ...
    'Position', [90 45 130 69]);
add_block('simulink/Continuous/Derivative', [S2 '/Acceleration_dv_dt'], ...
    'Position', [170 85 210 115]);
add_block('simulink/Sources/Constant', [S2 '/Vehicle_parameters'], 'Value', 'VEH.vec', ...
    'Position', [150 190 230 210]);
mfb([S2 '/Traction_Power_Model'], code_vehicle(), [280 40 450 220]);
outP(S2, 'P_batt_W', 1, [520 50 550 64]);
outP(S2, 'F_road_N', 2, [520 120 550 134]);
L(S2, 'v_kmh/1', 'kmh_to_mps/1', 'v [km/h]');
L(S2, 'kmh_to_mps/1', 'Traction_Power_Model/1', 'v [m/s]');
L(S2, 'kmh_to_mps/1', 'Acceleration_dv_dt/1', '');
L(S2, 'Acceleration_dv_dt/1', 'Traction_Power_Model/2', 'a [m/s^2]');
L(S2, 'grade_pct/1', 'Traction_Power_Model/3', 'grade [%]');
L(S2, 'Vehicle_parameters/1', 'Traction_Power_Model/4', 'VEH');
L(S2, 'Traction_Power_Model/1', 'P_batt_W/1', 'P_batt [W]');
L(S2, 'Traction_Power_Model/2', 'F_road_N/1', 'F_road [N]');

%% 3. PACK CURRENT & BMS CURRENT LIMITS ---------------------------------------
S3 = [M '/3_Pack_Current_and_BMS_Limits'];
mkSub(S3, [560 60 760 260], [1.0 0.95 0.85]);
inP(S3, 'P_batt_W', 1, [20 40 50 54]);
inP(S3, 'V_pack_prev_V', 2, [20 90 50 104]);
inP(S3, 'I_cell_measured_A', 3, [20 150 50 164]);
inP(S3, 'I_dis_max_A', 4, [20 220 50 234]);
inP(S3, 'I_chg_max_A', 5, [20 270 50 284]);
add_block('simulink/Discontinuities/Saturation', [S3 '/V_floor'], 'UpperLimit', 'inf', ...
    'LowerLimit', '1', 'Position', [80 85 110 109]);
add_block('simulink/Math Operations/Divide', [S3 '/I_request_P_over_V'], ...
    'Position', [150 45 190 100]);
add_block('simulink/Math Operations/Gain', [S3 '/Cell_to_pack_x_Np'], 'Gain', 'PACK.Np', ...
    'Position', [150 145 200 169]);
add_block('simulink/Sources/Constant', [S3 '/Input_mode'], 'Value', 'SCN.mode', ...
    'Position', [150 100 200 120]);
add_block('simulink/Signal Routing/Switch', [S3 '/Mode_switch_1vehicle_2measured'], ...
    'Criteria', 'u2 > Threshold', 'Threshold', '1.5', 'Position', [260 60 300 160]);
add_block('simulink/Math Operations/Gain', [S3 '/Negate_charge_limit'], 'Gain', '-1', ...
    'Position', [260 265 300 289]);
add_block('simulink/Discontinuities/Saturation Dynamic', [S3 '/BMS_current_limiter'], ...
    'Position', [360 160 410 240]);
add_block('simulink/Math Operations/Gain', [S3 '/Pack_to_cell_div_Np'], 'Gain', '1/PACK.Np', ...
    'Position', [460 185 510 215]);
outP(S3, 'I_pack_A', 1, [560 150 590 164]);
outP(S3, 'I_cell_A', 2, [560 193 590 207]);
L(S3, 'P_batt_W/1', 'I_request_P_over_V/1', 'P_batt [W]');
L(S3, 'V_pack_prev_V/1', 'V_floor/1', 'V_pack(k-1) [V]');
L(S3, 'V_floor/1', 'I_request_P_over_V/2', '');
L(S3, 'I_cell_measured_A/1', 'Cell_to_pack_x_Np/1', 'I_cell,meas [A]');
L(S3, 'I_request_P_over_V/1', 'Mode_switch_1vehicle_2measured/3', 'I_pack,vehicle [A]');
L(S3, 'Input_mode/1', 'Mode_switch_1vehicle_2measured/2', 'mode');
L(S3, 'Cell_to_pack_x_Np/1', 'Mode_switch_1vehicle_2measured/1', 'I_pack,meas [A]');
L(S3, 'I_dis_max_A/1', 'BMS_current_limiter/1', 'I_dis,max [A]');
L(S3, 'Mode_switch_1vehicle_2measured/1', 'BMS_current_limiter/2', 'I_pack,request [A]');
L(S3, 'I_chg_max_A/1', 'Negate_charge_limit/1', 'I_chg,max [A]');
L(S3, 'Negate_charge_limit/1', 'BMS_current_limiter/3', '-I_chg,max [A]');
L(S3, 'BMS_current_limiter/1', 'I_pack_A/1', 'I_pack [A]');
L(S3, 'BMS_current_limiter/1', 'Pack_to_cell_div_Np/1', '');
L(S3, 'Pack_to_cell_div_Np/1', 'I_cell_A/1', 'I_cell [A]');

%% 4. NONLINEAR ELECTRO-THERMAL CELL MODEL ------------------------------------
S4 = [M '/4_Cell_ElectroThermal_Model'];
mkSub(S4, [840 60 1060 340], [1.0 1.0 0.75]);
inP(S4, 'I_cell_A', 1, [20 200 50 214]);
inP(S4, 'T_ambient_C', 2, [20 470 50 484]);

% 4a parameter maps  f(SOC, T_core)
S4a = [S4 '/4a_Parameter_Maps_SOC_Tcore'];
mkSub(S4a, [300 40 440 160], [0.95 0.9 1.0]);
inP(S4a, 'SOC', 1, [20 100 50 114]);
inP(S4a, 'T_core_C', 2, [20 420 50 434]);
add_block('simulink/Signal Routing/Mux', [S4a '/Mux_parameters'], 'Inputs', '10', ...
    'Position', [430 20 436 600]);
tab2 = {'OCV_V', 'ET.OCV'; 'R0_ohm', 'ET.R0'; 'R1_ohm', 'ET.R1'; 'tau1_s', 'ET.tau1';
        'R2_ohm', 'ET.R2'; 'tau2_s', 'ET.tau2'};
for k = 1:6
    b = ['LUT2D_' tab2{k,1}];
    add_block('simulink/Lookup Tables/2-D Lookup Table', [S4a '/' b], ...
        'Table', tab2{k,2}, 'BreakpointsForDimension1', 'ET.soc_bp', ...
        'BreakpointsForDimension2', 'ET.T_bp', 'ExtrapMethod', 'Clip', ...
        'Position', [200 20+55*(k-1) 290 60+55*(k-1)]);
    L(S4a, 'SOC/1', [b '/1'], '');
    L(S4a, 'T_core_C/1', [b '/2'], '');
    L(S4a, [b '/1'], ['Mux_parameters/' num2str(k)], tab2{k,1});
end
tab1 = {'I0_exchange_A', 'ET.I0', 'ET.T_bp', 'T_core_C';
        'Hysteresis_M_V', 'ET.hyst_M', 'ET.T_bp', 'T_core_C';
        'Entropic_dUdT_VperK', 'ET.dUdT', 'ET.soc_bp', 'SOC';
        'Capacity_factor', 'ET.cap_factor', 'ET.T_bp', 'T_core_C'};
for k = 1:4
    b = ['LUT1D_' tab1{k,1}];
    add_block('simulink/Lookup Tables/1-D Lookup Table', [S4a '/' b], ...
        'Table', tab1{k,2}, 'BreakpointsForDimension1', tab1{k,3}, ...
        'ExtrapMethod', 'Clip', 'Position', [200 360+55*(k-1) 290 395+55*(k-1)]);
    L(S4a, [tab1{k,4} '/1'], [b '/1'], '');
    L(S4a, [b '/1'], ['Mux_parameters/' num2str(6+k)], tab1{k,1});
end
outP(S4a, 'par', 1, [480 300 510 314]);
L(S4a, 'Mux_parameters/1', 'par/1', 'par[10]');

% 4b nonlinear electrical dynamics + heat generation
S4b = [S4 '/4b_Electrical_Nonlinear_Dynamics'];
mkSub(S4b, [520 160 700 320], [1.0 0.85 0.85]);
inP(S4b, 'I_cell_A', 1, [20 40 50 54]);
inP(S4b, 'par', 2, [20 100 50 114]);
inP(S4b, 'T_core_C', 3, [20 160 50 174]);
inP(S4b, 'SOH', 4, [20 220 50 234]);
add_block('simulink/Sources/Constant', [S4b '/Cell_constants'], 'Value', 'ET.cst', ...
    'Position', [80 270 160 290]);
mfb([S4b '/Cell_Equations'], code_cell(), [220 30 400 300]);
add_block('simulink/Continuous/Integrator', [S4b '/States_SOC_V1_V2_h'], ...
    'InitialCondition', '[SCN.soc0; 0; 0; 0]', 'Position', [460 30 500 70]);
add_block('simulink/Signal Routing/Demux', [S4b '/Demux_states'], 'Outputs', '4', ...
    'Position', [540 20 546 110]);
for k = 2:4
    add_block('simulink/Sinks/Terminator', [S4b '/Term' num2str(k)], ...
        'Position', [580 15+22*k 595 30+22*k]);
end
outP(S4b, 'V_terminal_V', 1, [640 140 670 154]);
outP(S4b, 'SOC', 2, [640 30 670 44]);
outP(S4b, 'Q_heat_W', 3, [640 200 670 214]);
outP(S4b, 'eta_ct_V', 4, [640 260 670 274]);
L(S4b, 'I_cell_A/1', 'Cell_Equations/1', 'I [A]');
L(S4b, 'States_SOC_V1_V2_h/1', 'Cell_Equations/2', 'x = [SOC V1 V2 h]');
L(S4b, 'par/1', 'Cell_Equations/3', 'par');
L(S4b, 'Cell_constants/1', 'Cell_Equations/4', 'cst');
L(S4b, 'T_core_C/1', 'Cell_Equations/5', 'T_core [C]');
L(S4b, 'SOH/1', 'Cell_Equations/6', 'SOH');
L(S4b, 'Cell_Equations/1', 'States_SOC_V1_V2_h/1', 'dx/dt');
L(S4b, 'States_SOC_V1_V2_h/1', 'Demux_states/1', '');
L(S4b, 'Demux_states/1', 'SOC/1', 'SOC');
for k = 2:4, L(S4b, ['Demux_states/' num2str(k)], ['Term' num2str(k) '/1'], ''); end
L(S4b, 'Cell_Equations/2', 'V_terminal_V/1', 'V_t [V]');
L(S4b, 'Cell_Equations/3', 'Q_heat_W/1', 'Q = [ohm ct pol rev total] [W]');
L(S4b, 'Cell_Equations/4', 'eta_ct_V/1', 'eta_ct [V]');

% 4c thermal model (core / surface)
S4c = [S4 '/4c_Thermal_Model_Core_Surface'];
mkSub(S4c, [800 380 980 500], [0.85 0.9 1.0]);
inP(S4c, 'Q_heat_W', 1, [20 40 50 54]);
inP(S4c, 'T_ambient_C', 2, [20 120 50 134]);
add_block('simulink/Signal Routing/Selector', [S4c '/Select_Q_total'], ...
    'InputPortWidth', '5', 'Indices', '5', 'Position', [90 35 130 59]);
add_block('simulink/Signal Routing/Mux', [S4c '/Mux_Q_Tamb'], 'Inputs', '2', ...
    'Position', [180 40 186 130]);
add_block('simulink/Continuous/State-Space', [S4c '/Two_state_thermal_Cc_Cs_Rc_Ru'], ...
    'A', 'ET.A_th', 'B', 'ET.B_th', 'C', 'eye(2)', 'D', 'zeros(2)', ...
    'X0', '[SCN.T0; SCN.T0]', 'Position', [240 60 360 110]);
add_block('simulink/Signal Routing/Demux', [S4c '/Demux_T'], 'Outputs', '2', ...
    'Position', [400 55 406 115]);
outP(S4c, 'T_core_C', 1, [460 55 490 69]);
outP(S4c, 'T_surface_C', 2, [460 100 490 114]);
L(S4c, 'Q_heat_W/1', 'Select_Q_total/1', 'Q vector');
L(S4c, 'Select_Q_total/1', 'Mux_Q_Tamb/1', 'Q_total [W]');
L(S4c, 'T_ambient_C/1', 'Mux_Q_Tamb/2', 'T_amb [C]');
L(S4c, 'Mux_Q_Tamb/1', 'Two_state_thermal_Cc_Cs_Rc_Ru/1', '[Q; T_amb]');
L(S4c, 'Two_state_thermal_Cc_Cs_Rc_Ru/1', 'Demux_T/1', '[T_core; T_surf]');
L(S4c, 'Demux_T/1', 'T_core_C/1', 'T_core [C]');
L(S4c, 'Demux_T/2', 'T_surface_C/1', 'T_surf [C]');

% 4d ageing / SOH (Xu et al. 2018, throughput form)
S4d = [S4 '/4d_Ageing_SOH_Xu2018'];
mkSub(S4d, [520 380 700 500], [0.9 0.9 0.9]);
inP(S4d, 'I_cell_A', 1, [20 40 50 54]);
inP(S4d, 'SOC', 2, [20 90 50 104]);
inP(S4d, 'T_core_C', 3, [20 140 50 154]);
add_block('simulink/Sources/Constant', [S4d '/Ageing_coefficients'], 'Value', '[ET.xu(:); SCN.SOH0; ET.Cn_Ah]', ...
    'Position', [60 190 140 210]);
add_block('simulink/Continuous/Integrator', [S4d '/Degradation_fd'], ...
    'InitialCondition', '0', 'Position', [380 40 420 70]);
mfb([S4d '/Xu_Stress_Model'], code_ageing(), [200 30 320 220]);
outP(S4d, 'SOH', 1, [480 120 510 134]);
L(S4d, 'I_cell_A/1', 'Xu_Stress_Model/1', 'I [A]');
L(S4d, 'SOC/1', 'Xu_Stress_Model/2', 'SOC');
L(S4d, 'T_core_C/1', 'Xu_Stress_Model/3', 'T_core [C]');
L(S4d, 'Degradation_fd/1', 'Xu_Stress_Model/4', 'f_d');
L(S4d, 'Ageing_coefficients/1', 'Xu_Stress_Model/5', 'xu');
L(S4d, 'Xu_Stress_Model/1', 'Degradation_fd/1', 'df_d/dt');
L(S4d, 'Xu_Stress_Model/2', 'SOH/1', 'SOH');

% 4 wiring and outputs
outP(S4, 'V_cell_V', 1, [1060 100 1090 114]);
outP(S4, 'SOC', 2, [1060 160 1090 174]);
outP(S4, 'T_core_C', 3, [1060 400 1090 414]);
outP(S4, 'T_surface_C', 4, [1060 450 1090 464]);
outP(S4, 'Q_heat_W', 5, [1060 260 1090 274]);
outP(S4, 'SOH', 6, [1060 520 1090 534]);
outP(S4, 'eta_ct_V', 7, [1060 310 1090 324]);
L(S4, 'I_cell_A/1', '4b_Electrical_Nonlinear_Dynamics/1', 'I_cell [A]');
L(S4, '4a_Parameter_Maps_SOC_Tcore/1', '4b_Electrical_Nonlinear_Dynamics/2', 'par(SOC,T_core)');
L(S4, '4c_Thermal_Model_Core_Surface/1', '4b_Electrical_Nonlinear_Dynamics/3', 'T_core [C]');
L(S4, '4d_Ageing_SOH_Xu2018/1', '4b_Electrical_Nonlinear_Dynamics/4', 'SOH');
L(S4, '4b_Electrical_Nonlinear_Dynamics/2', '4a_Parameter_Maps_SOC_Tcore/1', 'SOC');
L(S4, '4c_Thermal_Model_Core_Surface/1', '4a_Parameter_Maps_SOC_Tcore/2', 'T_core [C]');
L(S4, '4b_Electrical_Nonlinear_Dynamics/3', '4c_Thermal_Model_Core_Surface/1', 'Q [W]');
L(S4, 'T_ambient_C/1', '4c_Thermal_Model_Core_Surface/2', 'T_amb [C]');
L(S4, 'I_cell_A/1', '4d_Ageing_SOH_Xu2018/1', '');
L(S4, '4b_Electrical_Nonlinear_Dynamics/2', '4d_Ageing_SOH_Xu2018/2', '');
L(S4, '4c_Thermal_Model_Core_Surface/1', '4d_Ageing_SOH_Xu2018/3', '');
L(S4, '4b_Electrical_Nonlinear_Dynamics/1', 'V_cell_V/1', 'V_cell [V]');
L(S4, '4b_Electrical_Nonlinear_Dynamics/2', 'SOC/1', 'SOC');
L(S4, '4c_Thermal_Model_Core_Surface/1', 'T_core_C/1', 'T_core [C]');
L(S4, '4c_Thermal_Model_Core_Surface/2', 'T_surface_C/1', 'T_surf [C]');
L(S4, '4b_Electrical_Nonlinear_Dynamics/3', 'Q_heat_W/1', 'Q [W]');
L(S4, '4d_Ageing_SOH_Xu2018/1', 'SOH/1', 'SOH');
L(S4, '4b_Electrical_Nonlinear_Dynamics/4', 'eta_ct_V/1', 'eta_ct [V]');

%% 5. PACK OUTPUTS -------------------------------------------------------------
S5 = [M '/5_Pack_Outputs_Energy'];
mkSub(S5, [1160 60 1340 220], [0.85 1.0 1.0]);
inP(S5, 'V_cell_V', 1, [20 40 50 54]);
inP(S5, 'I_pack_A', 2, [20 110 50 124]);
inP(S5, 'v_kmh', 3, [20 200 50 214]);
add_block('simulink/Math Operations/Gain', [S5 '/Series_cells_x_Ns'], 'Gain', 'PACK.Ns', ...
    'Position', [90 35 140 59]);
add_block('simulink/Math Operations/Product', [S5 '/Pack_power_V_x_I'], ...
    'Position', [200 60 240 110]);
add_block('simulink/Math Operations/Gain', [S5 '/W_to_kW_h'], 'Gain', '1/3.6e6', ...
    'Position', [280 75 330 99]);
add_block('simulink/Continuous/Integrator', [S5 '/Integrate_energy'], 'Position', [370 70 410 104]);
add_block('simulink/Math Operations/Gain', [S5 '/kmh_to_kms'], 'Gain', '1/3600', ...
    'Position', [90 195 140 219]);
add_block('simulink/Continuous/Integrator', [S5 '/Integrate_distance'], 'Position', [200 190 240 224]);
outP(S5, 'V_pack_V', 1, [480 35 510 49]);
outP(S5, 'P_pack_W', 2, [480 120 510 134]);
outP(S5, 'E_kWh', 3, [480 80 510 94]);
outP(S5, 'Distance_km', 4, [480 200 510 214]);
L(S5, 'V_cell_V/1', 'Series_cells_x_Ns/1', 'V_cell [V]');
L(S5, 'Series_cells_x_Ns/1', 'V_pack_V/1', 'V_pack [V]');
L(S5, 'Series_cells_x_Ns/1', 'Pack_power_V_x_I/1', '');
L(S5, 'I_pack_A/1', 'Pack_power_V_x_I/2', 'I_pack [A]');
L(S5, 'Pack_power_V_x_I/1', 'W_to_kW_h/1', 'P_pack [W]');
L(S5, 'Pack_power_V_x_I/1', 'P_pack_W/1', '');
L(S5, 'W_to_kW_h/1', 'Integrate_energy/1', '');
L(S5, 'Integrate_energy/1', 'E_kWh/1', 'E [kWh]');
L(S5, 'v_kmh/1', 'kmh_to_kms/1', 'v [km/h]');
L(S5, 'kmh_to_kms/1', 'Integrate_distance/1', '');
L(S5, 'Integrate_distance/1', 'Distance_km/1', 'distance [km]');

%% 6. BMS PROTECTION & DERATING ------------------------------------------------
S6 = [M '/6_BMS_Protection_and_Derating'];
mkSub(S6, [1160 300 1340 460], [1.0 0.8 0.8]);
inP(S6, 'V_cell_V', 1, [20 40 50 54]);
inP(S6, 'T_core_C', 2, [20 90 50 104]);
inP(S6, 'T_surface_C', 3, [20 140 50 154]);
inP(S6, 'SOC', 4, [20 190 50 204]);
add_block('simulink/Sources/Constant', [S6 '/BMS_thresholds'], 'Value', 'BMS.vec', ...
    'Position', [60 240 140 260]);
mfb([S6 '/Protection_Logic'], code_bms(), [200 30 360 270]);
outP(S6, 'I_dis_max_A', 1, [420 50 450 64]);
outP(S6, 'I_chg_max_A', 2, [420 120 450 134]);
outP(S6, 'Flags', 3, [420 200 450 214]);
L(S6, 'V_cell_V/1', 'Protection_Logic/1', 'V_cell [V]');
L(S6, 'T_core_C/1', 'Protection_Logic/2', 'T_core [C]');
L(S6, 'T_surface_C/1', 'Protection_Logic/3', 'T_surf [C]');
L(S6, 'SOC/1', 'Protection_Logic/4', 'SOC');
L(S6, 'BMS_thresholds/1', 'Protection_Logic/5', 'BMS');
L(S6, 'Protection_Logic/1', 'I_dis_max_A/1', 'I_dis,max pack [A]');
L(S6, 'Protection_Logic/2', 'I_chg_max_A/1', 'I_chg,max pack [A]');
L(S6, 'Protection_Logic/3', 'Flags/1', 'flags[7]');

%% 7. TOP-LEVEL WIRING (unit delays break the algebraic loops) -----------------
add_block('simulink/Discrete/Memory', [M '/Delay_V_pack'], 'InitialCondition', ...
    'PACK.Ns*3.7', 'Position', [620 330 660 360]);
add_block('simulink/Discrete/Memory', [M '/Delay_I_dis_max'], 'InitialCondition', ...
    'BMS.I_dis_cont*PACK.Np', 'Position', [620 400 660 430]);
add_block('simulink/Discrete/Memory', [M '/Delay_I_chg_max'], 'InitialCondition', ...
    'BMS.I_chg_cont*PACK.Np', 'Position', [620 460 660 490]);
L(M, '1_Drive_Cycle_and_Climate/1', '2_EV_Vehicle_Dynamics/1', 'v_vehicle [km/h]');
L(M, '1_Drive_Cycle_and_Climate/2', '2_EV_Vehicle_Dynamics/2', 'road grade [%]');
L(M, '1_Drive_Cycle_and_Climate/3', '4_Cell_ElectroThermal_Model/2', 'T_ambient [C]');
L(M, '1_Drive_Cycle_and_Climate/4', '3_Pack_Current_and_BMS_Limits/3', 'I_cell,measured [A]');
L(M, '1_Drive_Cycle_and_Climate/1', '5_Pack_Outputs_Energy/3', '');
L(M, '2_EV_Vehicle_Dynamics/1', '3_Pack_Current_and_BMS_Limits/1', 'P_battery [W]');
L(M, 'Delay_V_pack/1', '3_Pack_Current_and_BMS_Limits/2', 'V_pack(k-1) [V]');
L(M, 'Delay_I_dis_max/1', '3_Pack_Current_and_BMS_Limits/4', 'I_dis,max(k-1) [A]');
L(M, 'Delay_I_chg_max/1', '3_Pack_Current_and_BMS_Limits/5', 'I_chg,max(k-1) [A]');
L(M, '3_Pack_Current_and_BMS_Limits/2', '4_Cell_ElectroThermal_Model/1', 'I_cell [A]');
L(M, '3_Pack_Current_and_BMS_Limits/1', '5_Pack_Outputs_Energy/2', 'I_pack [A]');
L(M, '4_Cell_ElectroThermal_Model/1', '5_Pack_Outputs_Energy/1', 'V_cell [V]');
L(M, '5_Pack_Outputs_Energy/1', 'Delay_V_pack/1', 'V_pack [V]');
L(M, '4_Cell_ElectroThermal_Model/1', '6_BMS_Protection_and_Derating/1', '');
L(M, '4_Cell_ElectroThermal_Model/3', '6_BMS_Protection_and_Derating/2', 'T_core [C]');
L(M, '4_Cell_ElectroThermal_Model/4', '6_BMS_Protection_and_Derating/3', 'T_surface [C]');
L(M, '4_Cell_ElectroThermal_Model/2', '6_BMS_Protection_and_Derating/4', 'SOC');
L(M, '6_BMS_Protection_and_Derating/1', 'Delay_I_dis_max/1', 'I_dis,max [A]');
L(M, '6_BMS_Protection_and_Derating/2', 'Delay_I_chg_max/1', 'I_chg,max [A]');

%% 8. LOGGING AND SCOPES --------------------------------------------------------
logs = {'log_vehicle', {'1_Drive_Cycle_and_Climate/1', '2_EV_Vehicle_Dynamics/1', ...
                        '5_Pack_Outputs_Energy/4', '5_Pack_Outputs_Energy/3'}, ...
            'v [km/h], P_batt [W], distance [km], E [kWh]';
        'log_cell',    {'3_Pack_Current_and_BMS_Limits/2', '4_Cell_ElectroThermal_Model/1', ...
                        '4_Cell_ElectroThermal_Model/2', '4_Cell_ElectroThermal_Model/6', ...
                        '4_Cell_ElectroThermal_Model/7'}, ...
            'I_cell [A], V_cell [V], SOC, SOH, eta_ct [V]';
        'log_thermal', {'1_Drive_Cycle_and_Climate/3', '4_Cell_ElectroThermal_Model/4', ...
                        '4_Cell_ElectroThermal_Model/3'}, 'T_amb, T_surf, T_core [C]';
        'log_heat',    {'4_Cell_ElectroThermal_Model/5'}, ...
            'Q_ohmic, Q_charge-transfer, Q_polarisation, Q_reversible, Q_total [W]';
        'log_bms',     {'6_BMS_Protection_and_Derating/1', '6_BMS_Protection_and_Derating/2', ...
                        '6_BMS_Protection_and_Derating/3'}, ...
            'I_dis,max, I_chg,max [A], flags: OT warn, OT derate, OT shutdown, UT charge inhibit, OV, UV, low SOC';
        'log_pack',    {'5_Pack_Outputs_Energy/1', '3_Pack_Current_and_BMS_Limits/1', ...
                        '5_Pack_Outputs_Energy/2'}, 'V_pack [V], I_pack [A], P_pack [W]'};
y0 = 560;
for k = 1:size(logs, 1)
    nm = logs{k,1}; srcs = logs{k,2};
    mux = [M '/Mux_' nm];
    add_block('simulink/Signal Routing/Mux', mux, 'Inputs', num2str(numel(srcs)), ...
        'Position', [1420 y0+90*(k-1) 1426 y0+90*(k-1)+60]);
    for j = 1:numel(srcs)
        add_line(M, srcs{j}, ['Mux_' nm '/' num2str(j)], 'autorouting', 'on');
    end
    add_block('simulink/Sinks/To Workspace', [M '/' nm], 'VariableName', nm, ...
        'SaveFormat', 'Timeseries', 'Position', [1480 y0+90*(k-1)+15 1560 y0+90*(k-1)+45]);
    add_block('simulink/Sinks/Scope', [M '/Scope_' nm(5:end)], ...
        'Position', [1480 y0+90*(k-1)+55 1510 y0+90*(k-1)+80]);
    L(M, ['Mux_' nm '/1'], [nm '/1'], logs{k,3});
    add_line(M, ['Mux_' nm '/1'], ['Scope_' nm(5:end) '/1'], 'autorouting', 'on');
end


%% 7. BMS SENSORS (INA240 current, INA226 voltage, DS18B20 temperature) ------
S7 = [M '/7_BMS_Sensors_INA240_INA226_DS18B20'];
mkSub(S7, [1160 520 1340 640], [0.9 0.9 1.0]);
inP(S7, 'I_cell_true_A', 1, [20 40 50 54]);
inP(S7, 'V_cell_true_V', 2, [20 140 50 154]);
inP(S7, 'T_surface_true_C', 3, [20 240 50 254]);
add_block('simulink/Math Operations/Gain', [S7 '/INA240_gain_error'], 'Gain', '1 + SCN.i_gain', ...
    'Position', [90 35 140 59]);
add_block('simulink/Sources/Random Number', [S7 '/INA240_noise'], 'Mean', 'SCN.i_offset', ...
    'Variance', 'SCN.i_noise^2', 'Seed', '11', 'SampleTime', '1', 'Position', [90 75 140 100]);
add_block('simulink/Math Operations/Sum', [S7 '/Add_I_offset_noise'], 'Inputs', '++', ...
    'Position', [180 40 200 90]);
add_block('simulink/Sources/Random Number', [S7 '/INA226_noise'], 'Mean', '0', ...
    'Variance', 'SCN.v_noise^2', 'Seed', '22', 'SampleTime', '1', 'Position', [90 175 140 200]);
add_block('simulink/Math Operations/Sum', [S7 '/Add_V_noise'], 'Inputs', '++', ...
    'Position', [180 140 200 190]);
add_block('simulink/Discontinuities/Quantizer', [S7 '/INA226_LSB_1p25mV'], ...
    'QuantizationInterval', 'SCN.v_lsb', 'Position', [230 150 270 180]);
add_block('simulink/Math Operations/Bias', [S7 '/DS18B20_offset'], 'Bias', 'SCN.t_offset', ...
    'Position', [90 235 140 259]);
add_block('simulink/Discontinuities/Quantizer', [S7 '/DS18B20_LSB_0p0625C'], ...
    'QuantizationInterval', 'SCN.t_lsb', 'Position', [180 230 220 264]);
for k = 1:3
    add_block('simulink/Discrete/Zero-Order Hold', [S7 '/ADC_1Hz_' num2str(k)], 'SampleTime', '1', ...
        'Position', [300 40+100*(k-1) 330 70+100*(k-1)]);
end
outP(S7, 'I_meas_A', 1, [380 47 410 61]);
outP(S7, 'V_meas_V', 2, [380 147 410 161]);
outP(S7, 'T_meas_C', 3, [380 247 410 261]);
L(S7, 'I_cell_true_A/1', 'INA240_gain_error/1', 'I_true [A]');
L(S7, 'INA240_gain_error/1', 'Add_I_offset_noise/1', '');
L(S7, 'INA240_noise/1', 'Add_I_offset_noise/2', 'offset + noise');
L(S7, 'Add_I_offset_noise/1', 'ADC_1Hz_1/1', '');
L(S7, 'ADC_1Hz_1/1', 'I_meas_A/1', 'I_meas [A]');
L(S7, 'V_cell_true_V/1', 'Add_V_noise/1', 'V_true [V]');
L(S7, 'INA226_noise/1', 'Add_V_noise/2', 'noise');
L(S7, 'Add_V_noise/1', 'INA226_LSB_1p25mV/1', '');
L(S7, 'INA226_LSB_1p25mV/1', 'ADC_1Hz_2/1', '');
L(S7, 'ADC_1Hz_2/1', 'V_meas_V/1', 'V_meas [V]');
L(S7, 'T_surface_true_C/1', 'DS18B20_offset/1', 'T_surf [C]');
L(S7, 'DS18B20_offset/1', 'DS18B20_LSB_0p0625C/1', '');
L(S7, 'DS18B20_LSB_0p0625C/1', 'ADC_1Hz_3/1', '');
L(S7, 'ADC_1Hz_3/1', 'T_meas_C/1', 'T_meas [C]');

%% 8. ESTIMATOR INPUT: live sensors (1) or recorded LG HG2 test data (2) -----
S8 = [M '/8_Estimator_Input_Select'];
mkSub(S8, [1400 520 1560 680], [0.95 0.95 0.95]);
inP(S8, 'I_meas_A', 1, [20 40 50 54]);
inP(S8, 'V_meas_V', 2, [20 100 50 114]);
inP(S8, 'T_meas_C', 3, [20 160 50 174]);
inP(S8, 'SOC_plant', 4, [20 220 50 234]);
add_block('simulink/Math Operations/Gain', [S8 '/SOC_to_estimator_definition'], 'Gain', 'SCN.Cn_ratio', ...
    'Position', [80 215 140 239]);
add_block('simulink/Math Operations/Bias', [S8 '/Offset_1_minus_ratio'], 'Bias', '1 - SCN.Cn_ratio', ...
    'Position', [160 215 210 239]);
add_block('simulink/Discrete/Zero-Order Hold', [S8 '/SOC_ref_1Hz'], 'SampleTime', '1', ...
    'Position', [230 212 260 242]);
recs = {'REC_I_A', 'REC.I'; 'REC_V_V', 'REC.V'; 'REC_T_C', 'REC.T'; 'REC_SOC_ref', 'REC.SOC'};
add_block('simulink/Sources/Constant', [S8 '/Estimator_source'], 'Value', 'SCN.est_source', ...
    'Position', [300 300 360 320]);
for k = 1:4
    add_block('simulink/Sources/From Workspace', [S8 '/' recs{k,1}], 'VariableName', recs{k,2}, ...
        'Interpolate', 'off', 'SampleTime', '1', 'OutputAfterFinalValue', 'Holding final value', ...
        'Position', [300 360+50*(k-1) 400 385+50*(k-1)]);
    add_block('simulink/Signal Routing/Switch', [S8 '/Select_' num2str(k)], 'Criteria', ...
        'u2 > Threshold', 'Threshold', '1.5', 'Position', [460 30+60*(k-1) 500 80+60*(k-1)]);
end
outP(S8, 'I_A', 1, [560 47 590 61]);
outP(S8, 'V_V', 2, [560 107 590 121]);
outP(S8, 'T_C', 3, [560 167 590 181]);
outP(S8, 'SOC_reference', 4, [560 227 590 241]);
L(S8, 'SOC_plant/1', 'SOC_to_estimator_definition/1', 'SOC (C/20 basis)');
L(S8, 'SOC_to_estimator_definition/1', 'Offset_1_minus_ratio/1', '');
L(S8, 'Offset_1_minus_ratio/1', 'SOC_ref_1Hz/1', '');
live = {'I_meas_A/1', 'V_meas_V/1', 'T_meas_C/1', 'SOC_ref_1Hz/1'};
for k = 1:4
    L(S8, [recs{k,1} '/1'], ['Select_' num2str(k) '/1'], 'recorded');
    L(S8, 'Estimator_source/1', ['Select_' num2str(k) '/2'], '');
    L(S8, live{k}, ['Select_' num2str(k) '/3'], 'live');
end
L(S8, 'Select_1/1', 'I_A/1', 'I [A]');
L(S8, 'Select_2/1', 'V_V/1', 'V [V]');
L(S8, 'Select_3/1', 'T_C/1', 'T [C]');
L(S8, 'Select_4/1', 'SOC_reference/1', 'SOC_ref');

%% 9. REAL-TIME SOC ESTIMATOR (1 Hz, as on the STM32F407) --------------------
S9 = [M '/9_SOC_Estimator_EKF_CNN_Fusion_1Hz'];
mkSub(S9, [1620 520 1820 760], [0.8 1.0 0.8]);
inP(S9, 'I_A', 1, [20 60 50 74]);
inP(S9, 'V_V', 2, [20 140 50 154]);
inP(S9, 'T_C', 3, [20 220 50 234]);
mfb([S9 '/9a_ElectroThermal_EKF'], estimator_code('ekf'), [180 30 360 230]);
mfb([S9 '/9b_1D_CNN_SOC_Estimator'], estimator_code('cnn'), [180 280 360 420]);
mfb([S9 '/9c_Confidence_Adaptive_Fusion'], estimator_code('fusion'), [480 30 680 420]);
L(S9, 'V_V/1', '9a_ElectroThermal_EKF/1', 'V [V]');
L(S9, 'I_A/1', '9a_ElectroThermal_EKF/2', 'I [A]');
L(S9, 'T_C/1', '9a_ElectroThermal_EKF/3', 'T_surf [C]');
L(S9, 'V_V/1', '9b_1D_CNN_SOC_Estimator/1', '');
L(S9, 'I_A/1', '9b_1D_CNN_SOC_Estimator/2', '');
L(S9, 'T_C/1', '9b_1D_CNN_SOC_Estimator/3', '');
fu = '9c_Confidence_Adaptive_Fusion';
L(S9, '9a_ElectroThermal_EKF/1', [fu '/1'], 'SOC_EKF');
L(S9, '9a_ElectroThermal_EKF/4', [fu '/2'], 'innovation [V]');
L(S9, '9a_ElectroThermal_EKF/5', [fu '/3'], 'S [V^2]');
L(S9, '9a_ElectroThermal_EKF/6', [fu '/4'], 'P_SOC');
L(S9, '9a_ElectroThermal_EKF/7', [fu '/5'], 'dOCV/dSOC');
L(S9, '9b_1D_CNN_SOC_Estimator/1', [fu '/6'], 'SOC_CNN');
L(S9, '9b_1D_CNN_SOC_Estimator/2', [fu '/7'], 'CNN valid');
L(S9, '9b_1D_CNN_SOC_Estimator/3', [fu '/8'], 'Mahalanobis d2');
L(S9, '9b_1D_CNN_SOC_Estimator/4', [fu '/9'], 'max |z|');
L(S9, 'I_A/1', [fu '/10'], '');
L(S9, '9a_ElectroThermal_EKF/8', [fu '/11'], 'SOC0 (key-on OCV)');
add_block('simulink/Sinks/Terminator', [S9 '/Term_Tamb_est'], 'Position', [420 120 435 135]);
L(S9, '9a_ElectroThermal_EKF/3', 'Term_Tamb_est/1', 'T_amb estimate');
outs = {'SOC_adaptive_fusion', [fu '/1'];  'SOC_fixed_fusion', [fu '/2'];
        'alpha_EKF_weight', [fu '/3'];      'conf_EKF', [fu '/4'];
        'conf_CNN', [fu '/5'];              'SOC_coulomb_counting', [fu '/6'];
        'SOC_EKF', '9a_ElectroThermal_EKF/1'; 'SOC_CNN', '9b_1D_CNN_SOC_Estimator/1';
        'T_core_estimated_C', '9a_ElectroThermal_EKF/2'};
for k = 1:size(outs, 1)
    outP(S9, outs{k,1}, k, [780 30+40*(k-1) 810 44+40*(k-1)]);
    L(S9, outs{k,2}, [outs{k,1} '/1'], strrep(outs{k,1}, '_', ' '));
end

%% 10. ESTIMATOR WIRING, LOGGING AND SCOPES ---------------------------------
L(M, '3_Pack_Current_and_BMS_Limits/2', '7_BMS_Sensors_INA240_INA226_DS18B20/1', '');
L(M, '4_Cell_ElectroThermal_Model/1', '7_BMS_Sensors_INA240_INA226_DS18B20/2', '');
L(M, '4_Cell_ElectroThermal_Model/4', '7_BMS_Sensors_INA240_INA226_DS18B20/3', '');
meas_names = {'I_meas [A]', 'V_meas [V]', 'T_meas [C]'};
for k = 1:3
    L(M, ['7_BMS_Sensors_INA240_INA226_DS18B20/' num2str(k)], ['8_Estimator_Input_Select/' num2str(k)], ...
        meas_names{k});
end
L(M, '4_Cell_ElectroThermal_Model/2', '8_Estimator_Input_Select/4', '');
for k = 1:3
    L(M, ['8_Estimator_Input_Select/' num2str(k)], ['9_SOC_Estimator_EKF_CNN_Fusion_1Hz/' num2str(k)], '');
end
add_block('simulink/Signal Routing/Mux', [M '/Mux_log_soc'], 'Inputs', '11', ...
    'Position', [1900 520 1906 820]);
L(M, '8_Estimator_Input_Select/4', 'Mux_log_soc/1', 'SOC_ref');
for k = 1:9
    add_line(M, ['9_SOC_Estimator_EKF_CNN_Fusion_1Hz/' num2str(k)], ['Mux_log_soc/' num2str(k+1)], ...
        'autorouting', 'on');
end
L(M, '8_Estimator_Input_Select/1', 'Mux_log_soc/11', '');
add_block('simulink/Sinks/To Workspace', [M '/log_soc'], 'VariableName', 'log_soc', ...
    'SaveFormat', 'Timeseries', 'Position', [1960 650 2040 680]);
L(M, 'Mux_log_soc/1', 'log_soc/1', ['[SOC_ref, adaptive, fixed, alpha, conf_EKF, conf_CNN, ' ...
    'CC, EKF, CNN, T_core_est, I]']);
add_block('simulink/Signal Routing/Mux', [M '/Mux_scope_soc'], 'Inputs', '5', ...
    'Position', [1900 860 1906 960]);
L(M, '8_Estimator_Input_Select/4', 'Mux_scope_soc/1', '');
L(M, '9_SOC_Estimator_EKF_CNN_Fusion_1Hz/6', 'Mux_scope_soc/2', '');
L(M, '9_SOC_Estimator_EKF_CNN_Fusion_1Hz/7', 'Mux_scope_soc/3', '');
L(M, '9_SOC_Estimator_EKF_CNN_Fusion_1Hz/8', 'Mux_scope_soc/4', '');
L(M, '9_SOC_Estimator_EKF_CNN_Fusion_1Hz/1', 'Mux_scope_soc/5', '');
add_block('simulink/Sinks/Scope', [M '/Scope_SOC_true_CC_EKF_CNN_Fusion'], 'Position', [1960 895 1990 925]);
L(M, 'Mux_scope_soc/1', 'Scope_SOC_true_CC_EKF_CNN_Fusion/1', 'SOC: ref, CC, EKF, CNN, fusion');

%% 11. ANNOTATIONS, LAYOUT, SAVE --------------------------------------------------
txt = sprintf(['EV BMS SOC ESTIMATION SYSTEM: EV + nonlinear electro-thermal LG 18650HG2 battery -> sensors -> EKF + 1-D CNN + adaptive fusion (1 Hz)\n' ...
    'Pack %ds x %dp | identified -20..40 C (McMaster LG HG2), Arrhenius-extended to %g..%g C\n' ...
    'Cell: OCV(SOC,T), R0/R1/R2/tau(SOC,T_core), Butler-Volmer charge transfer, hysteresis,\n' ...
    'Bernardi heat (ohmic + charge-transfer + polarisation + entropic), core/surface thermal model,\n' ...
    'Xu (2018) ageing. BMS: thermal derating 50-60 C, charge inhibit < 0 C, voltage/SOC limits.'], ...
    PACK.Ns, PACK.Np, ET.T_bp(1), ET.T_bp(end));
try
    a = Simulink.Annotation(M, txt); a.Position = [40 -120 900 -40];
catch
end
for s = {S1, S2, S3, S4, S4a, S4b, S4c, S4d, S5, S6, S7, S8, S9}
    try, Simulink.BlockDiagram.arrangeSystem(s{1}); catch, end
end
save_system(M, fullfile(here, [M '.slx']));
fprintf('Model "%s" built and saved in %s\n', M, here);

%% =========================================================================
%% Local helper functions
%% =========================================================================
function mkSub(path, pos, color)
add_block('simulink/Ports & Subsystems/Subsystem', path, 'Position', pos);
Simulink.SubSystem.deleteContents(path);
set_param(path, 'BackgroundColor', sprintf('[%g %g %g]', color));
end

function inP(sys, name, port, pos)
add_block('simulink/Ports & Subsystems/In1', [sys '/' name], 'Port', num2str(port), ...
    'Position', pos);
end

function outP(sys, name, port, pos)
add_block('simulink/Ports & Subsystems/Out1', [sys '/' name], 'Port', num2str(port), ...
    'Position', pos);
end

function L(sys, src, dst, name)
h = add_line(sys, src, dst, 'autorouting', 'on');
if ~isempty(name)
    try, set_param(h, 'Name', name); catch, end
end
end

function mfb(path, code, pos)
add_block('simulink/User-Defined Functions/MATLAB Function', path, 'Position', pos);
rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', path);
ch.Script = code;
end

function mfb1Hz(path)
% run a MATLAB Function block at the estimator rate (1 s)
rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', path);
try, ch.SampleTime = '1'; catch, end
end

function s = outName(k)
n = {'v_kmh', 'grade_pct', 'T_ambient_C', 'I_cell_measured_A'}; s = n{k};
end

function s = sigName(k)
n = {'v [km/h]', 'grade [%]', 'T_amb [C]', 'I_cell,meas [A]'}; s = n{k};
end

function c = code_vehicle()
c = strjoin({
'function [P_batt, F_road] = Traction_Power_Model(v, a, grade, p)'
'%#codegen'
'% Longitudinal EV dynamics. v [m/s], a [m/s^2], grade [%], p = VEH.vec'
'% p = [m Cd A Crr rho kr eta_drive eta_regen P_regen_max P_aux v_regen_min P_trac_max]'
'm = p(1); Cd = p(2); A = p(3); Crr = p(4); rho = p(5); kr = p(6);'
'eta_d = p(7); eta_r = p(8); Preg_max = p(9); P_aux = p(10);'
'v_reg_min = p(11)/3.6; Ptr_max = p(12);'
'th = atan(grade/100);'
'F_aero  = 0.5*rho*Cd*A*v^2;'
'F_roll  = m*9.81*Crr*cos(th)*(v > 0.1);'
'F_grade = m*9.81*sin(th);'
'F_inert = kr*m*a;'
'F_road = F_aero + F_roll + F_grade + F_inert;'
'P_wheel = F_road*v;'
'if P_wheel >= 0'
'    P_mot = min(P_wheel, Ptr_max)/eta_d;          % traction'
'elseif v > v_reg_min'
'    P_mot = max(P_wheel*eta_r, -Preg_max);        % regenerative braking'
'else'
'    P_mot = 0;                                    % friction brakes only'
'end'
'P_batt = P_mot + P_aux;'
'end'}, newline);
end

function c = code_cell()
c = strjoin({
'function [dx, Vt, Q, eta_ct] = Cell_Equations(I, x, par, cst, Tc, SOH)'
'%#codegen'
'% Nonlinear electro-thermal cell, I > 0 discharge. x = [SOC; V1; V2; h]'
'% par = [OCV R0 R1 tau1 R2 tau2 I0 M dUdT capf] at (SOC, T_core)'
'% cst = [Cn_Ah eta_chg gamma k_R]'
'soc = x(1); v1 = x(2); v2 = x(3); h = x(4);'
'Cn = cst(1); eta_chg = cst(2); gam = cst(3); kR = cst(4);'
'SOH = max(SOH, 0.5);'
'g = 1 + kR*(1 - SOH);                   % resistance growth with ageing'
'OCV = par(1); R0 = g*par(2); R1 = g*par(3); tau1 = max(par(4), 1e-3);'
'R2 = g*par(5); tau2 = max(par(6), 1e-3); I0 = max(par(7), 1e-3)/g;'
'M = par(8); dUdT = par(9); capf = par(10);'
'TK = Tc + 273.15;'
'% Butler-Volmer charge-transfer overpotential (theoretical Tafel slope)'
'eta_ct = 2*8.314462618*TK/96485.33212*asinh(I/(2*I0));'
'if I >= 0'
'    eta = 1;'
'else'
'    eta = eta_chg;'
'end'
'dsoc = -eta*I/(3600*Cn*capf*SOH);'
'dv1 = (R1*I - v1)/tau1;                 % fast polarisation'
'dv2 = (R2*I - v2)/tau2;                 % diffusion polarisation'
'dh  = -gam*abs(dsoc)*(h + sign(I));     % one-state hysteresis (Plett)'
'dx = [dsoc; dv1; dv2; dh];'
'Vt = OCV + M*h - I*R0 - eta_ct - v1 - v2;'
'% Bernardi heat generation [W]: Q = I*(OCV - Vt) - I*T*dU/dT, split by source'
'Q_ohm = I^2*R0;'
'Q_ct  = I*eta_ct;'
'Q_pol = I*(v1 + v2);                   % Bernardi: current x polarisation'
'Q_rev = -I*TK*dUdT;'
'Q = [Q_ohm; Q_ct; Q_pol; Q_rev; Q_ohm + Q_ct + Q_pol + Q_rev];'
'end'}, newline);
end

function c = code_ageing()
c = strjoin({
'function [dfd, SOH] = Xu_Stress_Model(I, soc, Tc, fd, xu)'
'%#codegen'
'% Semi-empirical degradation, Xu et al., IEEE Trans. Smart Grid 9(2), 2018.'
'% xu = [a_sei b_sei k_d1 k_d2 k_d3 k_s s_ref k_T T_ref k_t T_min DoD_ref SOH0 Cn]'
'a = xu(1); b = xu(2); kd1 = xu(3); kd2 = xu(4); kd3 = xu(5);'
'ks = xu(6); sref = xu(7); kT = xu(8); Tref = xu(9) + 273.15; kt = xu(10);'
'Tmin = xu(11); DoD = xu(12); SOH0 = xu(13); Cn = xu(14);'
'T = max(Tc, Tmin) + 273.15;             % law valid above 15 C (paper)'
'S_T = exp(kT*(T - Tref)*Tref/T);'
'S_s = exp(ks*(soc - sref));'
'S_d = 1/(kd1*DoD^kd2 + kd3);            % stress of one cycle of depth DoD'
'cyc_rate = abs(I)/(2*3600*Cn)/DoD;      % cycles of depth DoD per second'
'dfd = (kt + cyc_rate*S_d)*S_s*S_T;      % calendar + cycling'
'L = 1 - a*exp(-b*fd) - (1 - a)*exp(-fd);'
'SOH = SOH0*(1 - L);'
'end'}, newline);
end

function c = code_bms()
c = strjoin({
'function [I_dis_max, I_chg_max, flags] = Protection_Logic(V, Tc, Ts, soc, b)'
'%#codegen'
'% b = [I_dis I_chg T_warn T_derate T_cutoff T_chg_min T_chg_low T_chg_max'
'%      T_cold V_max V_min SOC_min Np enable]'
'Idis = b(1); Ichg = b(2); Tw = b(3); Td = b(4); Tcut = b(5);'
'Tcmin = b(6); Tclow = b(7); Tcmax = b(8); Tcold = b(9);'
'Vmax = b(10); Vmin = b(11); SOCmin = b(12); Np = b(13); en = b(14);'
'T = max(Tc, Ts);                        % hottest node (core estimate)'
'% discharge derating: full below T_derate, linear to zero at T_cutoff'
'fd = min(max((Tcut - T)/(Tcut - Td), 0), 1);'
'if T < Tcold, fd = 0.25*fd; end         % extreme cold: limit power'
'% charge window: none below T_chg_min (Li plating), half up to T_chg_low'
'if T < Tcmin'
'    fc = 0;'
'elseif T < Tclow'
'    fc = 0.5;'
'else'
'    fc = min(max((Tcmax - T)/(Tcmax - Td + 1e-6), 0), 1);'
'end'
'OV = double(V >= Vmax); UV = double(V <= Vmin); LS = double(soc <= SOCmin);'
'if OV > 0, fc = 0; end'
'if UV > 0 || LS > 0, fd = 0; end'
'flags = [double(T >= Tw); double(T >= Td); double(T >= Tcut); ...'
'         double(T < Tcmin); OV; UV; LS];'
'if en > 0'
'    I_dis_max = fd*Idis*Np; I_chg_max = fc*Ichg*Np;'
'else'
'    I_dis_max = 1e6; I_chg_max = 1e6;    % protection disabled (validation runs)'
'end'
'end'}, newline);
end
