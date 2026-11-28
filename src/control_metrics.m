function m = control_metrics(t, X, u, tStep, tEnd, q1Final)
%CONTROL_METRICS Step-response metrics on q1 between tStep and tEnd.
idx  = t >= tStep & t < tEnd;
tt   = t(idx) - tStep;   q1 = X(idx,1);   q10 = q1(1);
d    = q1Final - q10;                                   % commanded change
last = find(abs(q1 - q1Final) > 0.02*abs(d), 1, 'last');
m.settling   = tt(min(numel(tt), max(1, last + 1)));    % [s]
m.undershoot = 100*max(0, max(-sign(d)*(q1 - q10)))/abs(d);   % [%], non-minimum-phase effect
m.maxq2      = max(abs(rad2deg(X(idx,2))));             % [deg]
m.maxu       = max(abs(u(idx)));                        % [N]
end