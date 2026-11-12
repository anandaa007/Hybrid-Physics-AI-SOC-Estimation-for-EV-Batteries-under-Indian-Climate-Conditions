function D = tsmat(ts)
%TSMAT  Logged timeseries data as an N-by-k matrix (one row per time step).
%   To Workspace blocks log a column-vector signal as a k-by-1-by-N array;
%   this returns it as N-by-k so it can be indexed as D(:, j) and passed to
%   interp1 together with ts.Time.
D = ts.Data;
n = numel(ts.Time);
if ndims(D) == 3
    D = reshape(permute(D, [3 1 2]), size(D, 3), []);
elseif size(D, 1) ~= n && size(D, 2) == n
    D = D.';
end
end
