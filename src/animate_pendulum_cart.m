function animate_pendulum_cart(t, q1, q2, u, params, varargin)
%ANIMATE_PENDULUM_CART Animate the single-wheel inverted pendulum and plot q1, q2 and u.
%
%   animate_pendulum_cart(t, q1, q2, u, params)
%   animate_pendulum_cart(..., 'q1_ref', r1, 'q2_ref', r2, 'FPS', 30, 'VideoName', 'out.mp4')
%
%   INPUTS
%     t       : time vector, any sampling (ode45 or Simulink logs)      [s]
%     q1      : wheel (cart) position, same length as t                  [m]
%     q2      : pendulum angle from upward vertical, same length as t    [rad]
%     u       : horizontal force, same length as t, or a scalar          [N]
%     params  : struct with fields l (rod length) and CartRadius (wheel radius R)
%
%   OPTIONAL NAME-VALUE PAIRS
%     'q1_ref'    : q1 reference, same length as t                       [m]
%     'q2_ref'    : q2 reference, same length as t                       [rad]
%     'FPS'       : frame rate (default 30)
%     'VideoName' : if given, every frame is saved to this .mp4 file
%     'ShowLive'  : false hides the window while exporting a video (default true)
%     'Animate'   : false draws a static figure at once: complete curves and
%                   the final pose (default true). Use false in live scripts.
%
%   Live playback runs in real time: if drawing is slower than the frame
%   rate, frames are skipped instead of slowing the animation down.
%
%   KINEMATICS
%     x_pendulum = q1 - l*sin(q2),   y_pendulum = R + l*cos(q2)
%     wheel spin (rolling without slipping): phi = -q1/R

    %% Options
    p = inputParser;
    addParameter(p, 'VideoName', '');
    addParameter(p, 'FPS', 30);
    addParameter(p, 'q1_ref', []);
    addParameter(p, 'q2_ref', []);
    addParameter(p, 'ShowLive', true);
    addParameter(p, 'Animate', true);
    parse(p, varargin{:});
    opt       = p.Results;
    fps       = opt.FPS;
    saveVideo = ~isempty(opt.VideoName);

    l = params.l;
    R = params.CartRadius;

    %% Check inputs
    t  = t(:);
    q1 = q1(:);  q2 = q2(:);  u = u(:);
    if isscalar(u), u = u*ones(size(t)); end            % constant input, e.g. open loop
    q1_ref = opt.q1_ref(:);
    q2_ref = opt.q2_ref(:);

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
    N   = numel(tU);
    q1U = interp1(t, q1, tU);
    q2U = interp1(t, q2, tU);                           % [rad]
    uU  = interp1(t, u,  tU);
    hasRef1 = ~isempty(q1_ref);
    hasRef2 = ~isempty(q2_ref);
    if hasRef1, q1rU = interp1(t, q1_ref, tU); end
    if hasRef2, q2rU = rad2deg(interp1(t, q2_ref, tU)); end
    q2dU = rad2deg(q2U);                                % for plotting [deg]

    %% Precompute kinematics for every frame
    xP = q1U - l*sin(q2U);
    yP = R + l*cos(q2U);

    nSpokes  = 3;
    spokeOff = (0:nSpokes-1) * (2*pi/nSpokes);
    phiU     = -q1U / R;                                % wheel spin angle
    spokesX  = nan(N, 3*nSpokes);                       % per spoke: [hub, tip, NaN]
    spokesY  = nan(N, 3*nSpokes);
    for s = 1:nSpokes
        a    = pi/2 + phiU + spokeOff(s);
        cols = (s-1)*3 + (1:2);
        spokesX(:, cols) = [q1U, q1U + R*cos(a)];
        spokesY(:, cols) = [R*ones(size(a)), R + R*sin(a)];
    end

    %% Figure: one new figure per call. subplot keeps a fixed layout, so
    %  changing axis limits or text does not trigger a layout recomputation.
    % 'Visible','on' set explicitly opens a separate window even from a live
    % script, where inline figures would not show the animation while it plays.
    figArgs = {'Name', 'Pendulum-cart animation', 'NumberTitle', 'off', 'Color', 'w'};
    if saveVideo && ~opt.ShowLive
        figArgs = [figArgs, {'Visible', 'off'}];        % faster export, no live preview
    elseif opt.Animate
        figArgs = [figArgs, {'Visible', 'on'}];         % own window for live playback
    end                                                 % static figure: inline in live scripts
    fig = figure(figArgs{:});

    % Animation (top left)
    ax = subplot(2, 2, 1, 'Parent', fig); hold(ax, 'on'); grid(ax, 'on');
    viewWidth = 4;
    xlim(ax, q1U(1) + [-1 1]*viewWidth/2);
    ylim(ax, [-2*R, l + R]);
    ax.DataAspectRatio = [1 1 1];                       % like axis equal, set once
    ax.XLimMode = 'manual';  ax.YLimMode = 'manual';
    xlabel(ax, 'x [m]'); ylabel(ax, 'y [m]');
    plot(ax, [min(q1U) - 5, max(q1U) + 5], [-0.02 -0.02], 'k-', 'LineWidth', 5);
    ang    = linspace(0, 2*pi, 100);
    diskX  = R*sin(ang);
    diskY  = R + R*cos(ang);
    wheel  = patch(ax, 'XData', q1U(1) + diskX, 'YData', diskY, 'FaceColor', [0.3 0.3 0.6]);
    spokes = plot(ax, spokesX(1,:), spokesY(1,:), 'w-', 'LineWidth', 2);
    vRef   = plot(ax, [q1U(1) q1U(1)], [R, R + l], '--', 'Color', [0.2 0.7 0.2]);
    rod    = plot(ax, [q1U(1) xP(1)], [R yP(1)], 'k-', 'LineWidth', 2);
    mass   = plot(ax, xP(1), yP(1), 'o', 'MarkerSize', 15, ...
                  'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r');
    % time shown as text inside the axes: updating it does not move the layout
    tText  = text(ax, 0.02, 0.95, 't = 0.00 s', 'Units', 'normalized', ...
                  'FontWeight', 'bold', 'VerticalAlignment', 'top');

    % q1(t) (top right)
    ax2 = subplot(2, 2, 2, 'Parent', fig); hold(ax2, 'on'); grid(ax2, 'on');
    xlim(ax2, [tU(1) tU(end)]);
    ylabel(ax2, 'q_1 [m]'); xlabel(ax2, 'Time [s]');
    q1Line = animatedline(ax2, 'Color', 'b');
    if hasRef1
        q1RefLine = animatedline(ax2, 'Color', 'r');
        ylim(ax2, [min([q1U; q1rU]) max([q1U; q1rU])] + [-0.1 0.1]);
        legend(ax2, 'q_1', 'q_{1 ref}', 'AutoUpdate', 'off');
    else
        ylim(ax2, [min(q1U) max(q1U)] + [-0.1 0.1]);
    end

    % u(t) (bottom left)
    ax3 = subplot(2, 2, 3, 'Parent', fig); hold(ax3, 'on'); grid(ax3, 'on');
    xlim(ax3, [tU(1) tU(end)]);
    ylim(ax3, [min(uU) max(uU)] + [-0.1 0.1]);
    ylabel(ax3, 'u [N]'); xlabel(ax3, 'Time [s]');
    uLine = animatedline(ax3, 'Color', [0 0.6 0]);

    % q2(t) (bottom right)
    ax4 = subplot(2, 2, 4, 'Parent', fig); hold(ax4, 'on'); grid(ax4, 'on');
    xlim(ax4, [tU(1) tU(end)]);
    ylabel(ax4, 'q_2 [deg]'); xlabel(ax4, 'Time [s]');
    q2Line = animatedline(ax4, 'Color', 'r');
    if hasRef2
        q2RefLine = animatedline(ax4, 'Color', [0 0.6 0]);
        ylim(ax4, [min([q2dU; q2rU]) max([q2dU; q2rU])] + [-0.1 0.1]);
        legend(ax4, 'q_2', 'q_{2 ref}', 'AutoUpdate', 'off');
    else
        ylim(ax4, [min(q2dU) max(q2dU)] + [-0.1 0.1]);
    end

    % No interactive toolbars: less work for the renderer on every frame
    for a = [ax ax2 ax3 ax4]
        a.Toolbar.Visible = 'off';
        disableDefaultInteractivity(a);
    end

    drawnow;   % let the figure finish initializing before the loop starts

    %% Static figure: full curves and the final pose, drawn at once (no loop)
    if ~opt.Animate && ~saveVideo
        showFrame(1, N);
        drawnow;
        return
    end

    %% Video export: every frame is drawn and captured
    if saveVideo
        vw = VideoWriter(opt.VideoName, 'MPEG-4');
        vw.FrameRate = fps;
        open(vw);
        cleanupVideo = onCleanup(@() close(vw));        %#ok<NASGU> closes the file even after an error
        for k = 1:N
            if ~isgraphics(fig), break; end
            showFrame(k, k);
            drawnow;
            writeVideo(vw, getframe(fig));
        end
        fprintf('Animation saved to %s\n', opt.VideoName);
        return
    end

    %% Live playback: real time, skipping frames if drawing is too slow
    clock0 = tic;
    kLast  = 0;
    while kLast < N
        if ~isgraphics(fig), return; end                % window closed: stop cleanly
        k = min(N, max(kLast + 1, floor(toc(clock0)*fps) + 1));   % frame for "now"
        showFrame(kLast + 1, k);
        kLast = k;
        drawnow;                                        % frames are skipped above, so a full draw is affordable
        wait = kLast/fps - toc(clock0);                 % time until the next frame is due
        if wait > 0, pause(wait); end
    end
    if isgraphics(fig), drawnow; end                    % make sure the last frame is shown

    %% Nested function: draw frame k, adding plot points from frame k0 to k
    function showFrame(k0, k)
        xc = q1U(k);
        ax.XLim = xc + [-1 1]*viewWidth/2;              % camera follows the wheel
        set(wheel,  'XData', xc + diskX);
        set(spokes, 'XData', spokesX(k,:), 'YData', spokesY(k,:));
        set(rod,    'XData', [xc xP(k)], 'YData', [R yP(k)]);
        set(mass,   'XData', xP(k), 'YData', yP(k));
        set(vRef,   'XData', [xc xc]);
        tText.String = sprintf('t = %.2f s', tU(k));

        idx = k0:k;                                     % include skipped frames in the curves
        addpoints(q1Line, tU(idx), q1U(idx));
        addpoints(uLine,  tU(idx), uU(idx));
        addpoints(q2Line, tU(idx), q2dU(idx));
        if hasRef1, addpoints(q1RefLine, tU(idx), q1rU(idx)); end
        if hasRef2, addpoints(q2RefLine, tU(idx), q2rU(idx)); end
    end
end