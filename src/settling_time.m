function [Ts, info] = settling_time(t, y, varargin)
%SETTLING_TIME Settling time of a response, from simulation data.
%
%   Ts = settling_time(t, y)
%   Ts = settling_time(t, y, 'Tolerance', 0.02, 'FinalValue', yf, ...
%                      'StartTime', t0, 'EndTime', t1)
%   [Ts, info] = settling_time(...)
%
%   The settling time is the time, measured from StartTime, after which the
%   response stays inside the band |y - yf| <= Tolerance * |yf - y0| until
%   EndTime, where y0 is the value at StartTime.
%
%   INPUTS
%     t : time vector [s], any sampling (ode45, Simulink logs)
%     y : response, same length as t
%
%   OPTIONAL NAME-VALUE PAIRS
%     'Tolerance'  : band width as a fraction of the step size (default 0.02 = 2%)
%     'FinalValue' : target value yf (default: last value of y in the window)
%     'StartTime'  : start of the step, e.g. the time the reference changes
%                    (default: t(1))
%     'EndTime'    : end of the analysis window, e.g. the next reference
%                    change (default: t(end))
%
%   OUTPUTS
%     Ts   : settling time [s], counted from StartTime. NaN if the response
%            never stays inside the band before EndTime.
%     info : struct with fields
%              tSettle  absolute time at which the response settles [s]
%              y0, yf   initial and final values used
%              band     half-width of the tolerance band
%              settled  true if the response settled inside the window
%
%   EXAMPLE (wheel step from 0 to -5 m at t = 5 s, next change at t = 15 s)
%     X  = states_from_log(x);
%     Ts = settling_time(x.Time, X(:,1), 'FinalValue', -5, ...
%                        'StartTime', 5, 'EndTime', 15)

    p = inputParser;
    addParameter(p, 'Tolerance', 0.02, @(v) isscalar(v) && v > 0);
    addParameter(p, 'FinalValue', []);
    addParameter(p, 'StartTime', []);
    addParameter(p, 'EndTime', []);
    parse(p, varargin{:});
    opt = p.Results;

    % --- check inputs ---
    t = t(:);  y = squeeze(y);  y = y(:);
    if numel(t) ~= numel(y)
        error('settling_time:size', 't has %d samples but y has %d.', numel(t), numel(y));
    end
    t0 = opt.StartTime;  if isempty(t0), t0 = t(1);   end
    t1 = opt.EndTime;    if isempty(t1), t1 = t(end); end

    % --- analysis window ---
    idx = find(t >= t0 & t <= t1);
    if numel(idx) < 2
        error('settling_time:window', 'Fewer than two samples between StartTime and EndTime.');
    end
    tw = t(idx);  yw = y(idx);

    y0 = yw(1);
    yf = opt.FinalValue;  if isempty(yf), yf = yw(end); end

    step = abs(yf - y0);
    if step == 0
        error('settling_time:noStep', 'Initial and final values are equal: no step to settle.');
    end
    band = opt.Tolerance * step;

    % --- last sample outside the band: the response settles right after it ---
    outside = abs(yw - yf) > band;
    last = find(outside, 1, 'last');

    if isempty(last)                 % inside the band for the whole window
        tSettle = tw(1);
        settled = true;
    elseif last == numel(yw)         % still outside at the end of the window
        tSettle = NaN;
        settled = false;
    else
        tSettle = tw(last + 1);
        settled = true;
    end

    Ts = tSettle - t0;
    info = struct('tSettle', tSettle, 'y0', y0, 'yf', yf, 'band', band, 'settled', settled);
end
