function X = states_from_log(x)
%STATES_FROM_LOG Return a logged state timeseries as an N x 4 matrix [q1 q2 dq1 dq2].
    X = squeeze(x.Data);                                 % 1x4xN or 4x1xN -> 4xN
    if size(X,1) == 4 && size(X,2) ~= 4, X = X.'; end    % -> N x 4
end

%[appendix]{"version":"1.0"}
%---
