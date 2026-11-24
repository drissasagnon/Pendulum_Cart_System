function animate_pendulum_cart(t, q1, q2, u, params, varargin)
%ANIMATE_PENDULUM_CART Animate the single-wheel inverted pendulum and plot q1, q2 and u.
%
%   animate_pendulum_cart(t, q1, q2, u, params)
%   animate_pendulum_cart(..., 'q1_ref', r1, 'q2_ref', r2, 'FPS', 30, 'VideoName', 'out.mp4')
%
%   INPUTS
%     t       : time vector, any sampling (ode45 or Simulink "To Workspace")  [s]
%     q1      : wheel (cart) position, same length as t                        [m]
%     q2      : pendulum angle from upward vertical, same length as t          [rad]
%     u       : horizontal force, same length as t, or a scalar                [N]
%     params  : struct with fields l (rod length) and CartRadius (wheel radius R)
%
%   OPTIONAL NAME-VALUE PAIRS
%     'q1_ref'    : q1 reference, same length as t                             [m]
%     'q2_ref'    : q2 reference, same length as t                             [rad]
%     'FPS'       : playback / export frame rate (default 30)
%     'VideoName' : if given, the animation is saved to this .mp4 file
%     'ShowLive'  : false hides the figure while exporting a video (default true)
%
%   KINEMATICS
%     x_wheel    = q1
%     x_pendulum = q1 - l*sin(q2)
%     y_pendulum = R + l*cos(q2)
%     wheel spin (rolling without slipping): phi = -q1/R

    %% Options
    p = inputParser;
    addParameter(p, 'VideoName', '');
    addParameter(p, 'FPS', 30);
    addParameter(p, 'q1_ref', []);
    addParameter(p, 'q2_ref', []);
    addParameter(p, 'ShowLive', true);
    addParameter(p, 'model', 1)
    parse(p, varargin{:});
    videoName = p.Results.VideoName;
    fps       = p.Results.FPS;
    showLive  = p.Results.ShowLive;
    saveVideo = ~isempty(videoName);
    model = p.Results.model;

    l = params.l;
    R = params.CartRadius;

    %% Check inputs
    t  = t(:);
    q1 = q1(:);  q2 = q2(:);  u = u(:);
    if isscalar(u), u = u*ones(size(t)); end            % constant input, e.g. open loop
    q1_ref = p.Results.q1_ref(:);
    q2_ref = rad2deg(p.Results.q2_ref(:));              % plotted in degrees

    signals = {q1, q2, u, q1_ref, q2_ref};
    names   = {'q1', 'q2', 'u', 'q1_ref', 'q2_ref'};
    for i = 1:numel(signals)
        if ~isempty(signals{i}) && numel(signals{i}) ~= numel(t)
            error('animate_pendulum_cart:size', ...
                '%s has %d samples but t has %d. Resample it on t first.', ...
                names{i}, numel(signals{i}), numel(t));
        end
    end

    [t, iu] = unique(t);                                % Simulink can repeat time points
    q1 = q1(iu);  q2 = q2(iu);  u = u(iu);
    if ~isempty(q1_ref), q1_ref = q1_ref(iu); end
    if ~isempty(q2_ref), q2_ref = q2_ref(iu); end
    if numel(t) < 2
        error('animate_pendulum_cart:time', 'Need at least two time samples.');
    end

    %% Resample on a uniform time grid (one sample per frame)
    tU  = (t(1):1/fps:t(end))';
    q1U = interp1(t, q1, tU);
    q2U = interp1(t, rad2deg(q2), tU);                  % [deg]
    uU  = interp1(t, u, tU);

    ref1 = ~isempty(q1_ref);
    ref2 = ~isempty(q2_ref);
    if ref1, q1_refU = interp1(t, q1_ref, tU); end
    if ref2, q2_refU = interp1(t, q2_ref, tU); end

    %% Precompute kinematics for every frame
    x_pendulumU = q1U - l*sin(deg2rad(q2U));
    y_pendulumU = R + l*cos(deg2rad(q2U));

    % 3 wheel spokes, 120 deg apart, drawn with one line handle
    % (NaN separates the segments: cheaper than 3 handles)
    nSpokes  = 3;
    spokeOff = (0:nSpokes-1) * (2*pi/nSpokes);
    phiU     = -q1U / R;                                % wheel spin angle
    spokesXU = nan(numel(tU), 3*nSpokes);               % per spoke: [hub, tip, NaN]
    spokesYU = nan(numel(tU), 3*nSpokes);
    for s = 1:nSpokes
        spokeAngleU = pi/2 + phiU + spokeOff(s);
        cols = (s-1)*3 + (1:2);
        spokesXU(:, cols) = [q1U, q1U + R*cos(spokeAngleU)];
        spokesYU(:, cols) = [R*ones(size(spokeAngleU)), R + R*sin(spokeAngleU)];
    end

    %% Figure (figure 1, reset at every call)
    fig = figure(1);
    clf(fig, 'reset');
    if saveVideo && ~showLive
        fig.Visible = 'off';                            % faster export, no live preview
    end

    % Animation (top left)
    ax = subplot(2, 2, 1); hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
    viewWidth = 4;
    xlim(ax, [q1U(1) - viewWidth/2, q1U(1) + viewWidth/2]);
    ylim(ax, [-2*R, l + R]);
    xlabel(ax, 'x [m]'); ylabel(ax, 'y [m]');
    if model == 1
        hTitle = title(ax, 't = 0.00 s | linear mdl');
    else
        hTitle = title(ax, 't = 0.00 s | nonlinear mdl');
    end

    groundMargin = 5;
    plot(ax, [min(q1U) - groundMargin, max(q1U) + groundMargin], [-0.02 -0.02], ...
        'Color', 'black', 'LineWidth', 5);
    angle  = linspace(0, 2*pi, 100);
    disk_x = R*sin(angle);
    disk_y = R + R*cos(angle);
    wheelPatch = patch(ax, 'XData', disk_x, 'YData', disk_y, 'FaceColor', [0.3 0.3 0.6]);
    spokeLine  = plot(ax, spokesXU(1,:), spokesYU(1,:), 'w-', 'LineWidth', 2);
    refLine    = plot(ax, [0 0], [R, R + l], '--', 'Color', [0.2 0.7 0.2]);
    rodLine    = plot(ax, [0 0], [0 0], 'k-', 'LineWidth', 2);
    massMarker = plot(ax, 0, 0, 'o', 'MarkerSize', 15, ...
                      'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r');

    % q1(t) (top right)
    ax2 = subplot(2, 2, 2); hold(ax2, 'on'); grid(ax2, 'on');
    xlim(ax2, [tU(1) tU(end)]);
    ylabel(ax2, 'q_1 [m]'); xlabel(ax2, 'Time [s]');
    q1_line = animatedline(ax2, 'Color', 'b');
    if ref1
        q1_ref_line = animatedline(ax2, 'Color', 'r');
        ylim(ax2, [min([q1U; q1_refU]) max([q1U; q1_refU])] + [-0.1 0.1]);
        legend(ax2, 'q_1', 'q_{1 ref}');
    else
        ylim(ax2, [min(q1U) max(q1U)] + [-0.1 0.1]);
        legend(ax2, 'q_1');
    end

    % q2(t) (bottom right)
    ax4 = subplot(2, 2, 4); hold(ax4, 'on'); grid(ax4, 'on');
    xlim(ax4, [tU(1) tU(end)]);
    ylabel(ax4, 'q_2 [°]'); xlabel(ax4, 'Time [s]');
    q2_line = animatedline(ax4, 'Color', 'r');
    if ref2
        q2_ref_line = animatedline(ax4, 'Color', 'g');
        ylim(ax4, [min([q2U; q2_refU]) max([q2U; q2_refU])] + [-0.1 0.1]);
        legend(ax4, 'q_2', 'q_{2 ref}');
    else
        ylim(ax4, [min(q2U) max(q2U)] + [-0.1 0.1]);
        legend(ax4, 'q_2');
    end

    % u(t) (bottom left)
    ax3 = subplot(2, 2, 3); hold(ax3, 'on'); grid(ax3, 'on');
    xlim(ax3, [tU(1) tU(end)]);
    ylim(ax3, [min(uU) max(uU)] + [-0.1 0.1]);
    ylabel(ax3, 'u [N]'); xlabel(ax3, 'Time [s]');
    u_line = animatedline(ax3, 'Color', 'g');

    % No interactive toolbars: avoids per-frame interaction overhead
    for a = [ax ax2 ax3 ax4]
        try
            a.Toolbar.Visible = 'off';
            disableDefaultInteractivity(a);
        catch
            % older MATLAB versions may not support these: safe to skip
        end
    end

    %% Video writer
    if saveVideo
        vw = VideoWriter(videoName, 'MPEG-4');
        vw.FrameRate = fps;
        open(vw);
    end

    %% Animation loop
    for k = 1:numel(tU)
        x_wheel    = q1U(k);
        x_pendulum = x_pendulumU(k);
        y_pendulum = y_pendulumU(k);

        xlim(ax, [x_wheel - viewWidth/2, x_wheel + viewWidth/2]);   % camera follows the wheel
        set(wheelPatch, 'XData', x_wheel + disk_x, 'YData', disk_y);
        set(spokeLine,  'XData', spokesXU(k,:), 'YData', spokesYU(k,:));
        set(rodLine,    'XData', [x_wheel, x_pendulum], 'YData', [R, y_pendulum]);
        set(massMarker, 'XData', x_pendulum, 'YData', y_pendulum);
        set(refLine,    'XData', [x_wheel, x_wheel], 'YData', [R, R + l]);

        addpoints(u_line,  tU(k), uU(k));
        addpoints(q1_line, tU(k), x_wheel);
        addpoints(q2_line, tU(k), q2U(k));
        if ref1, addpoints(q1_ref_line, tU(k), q1_refU(k)); end
        if ref2, addpoints(q2_ref_line, tU(k), q2_refU(k)); end
        
        if model == 1
            set(hTitle, 'String', sprintf('t = %.2f s | linear mdl', tU(k)));
        else
            set(hTitle, 'String', sprintf('t = %.2f s | nonlinear mdl', tU(k)));
        end

        if saveVideo
            drawnow;                                    % full draw so getframe captures this frame
            writeVideo(vw, getframe(fig));
        else
            pause(1/fps);                               % paces playback and refreshes the figure
        end
    end

    if saveVideo
        close(vw);
        fprintf('Animation saved to %s\n', videoName);
    end
end