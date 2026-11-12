function [t, v_kmh] = make_MIDC(n_repeat, urban_only)
%MAKE_MIDC  Modified Indian Driving Cycle (MIDC) speed profile, 1 s steps.
%   [t, v] = make_MIDC(n_repeat, urban_only)
%   MIDC = 4 x ECE-15 urban cycle (195 s each, max 50 km/h) followed by the
%   extra-urban EUDC cycle for low-powered vehicles (max 90 km/h), 1180 s,
%   as used for Indian (ARAI) type approval. urban_only = true returns only
%   the 4 x ECE-15 part (used here for two-wheelers).
%   n_repeat: number of back-to-back MIDC cycles (default 1).
if nargin < 1, n_repeat = 1; end
if nargin < 2, urban_only = false; end
% ECE-15 elementary urban cycle: [duration_s, speed_at_end_kmh], linear ramps
ece = [11 0; 4 15; 8 15; 5 0; 21 0; 12 32; 24 32; 11 0; 21 0; 26 50; ...
       12 50; 8 35; 13 35; 12 0; 7 0];
% EUDC, low-powered vehicle version (speed capped at 90 km/h)
eudc = [20 0; 41 70; 50 70; 8 50; 69 50; 13 70; 50 70; 35 90; 30 90; ...
        30 90; 34 0; 20 0];
seg = repmat(ece, 4, 1);
if ~urban_only, seg = [seg; eudc]; end
v = 0;
for k = 1:size(seg, 1)
    d = seg(k, 1); v_end = seg(k, 2); v0 = v(end);
    v = [v, v0 + (v_end - v0) * (1:d) / d]; %#ok<AGROW>
end
v = repmat(v(2:end), 1, n_repeat);
v_kmh = [0, v]';
t = (0:numel(v_kmh) - 1)';
end
