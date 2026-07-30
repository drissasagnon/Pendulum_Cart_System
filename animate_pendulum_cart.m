function animate_pendulum_cart(t, q1, q2, u, params, varargin)
%ANIMATE_PENDULUM_CART Summary of this function goes here
% As it name suggests, this function will provide a visualization of the
% pendulum cart system.
%   INPUTS
%     t       : time vector, any sampling (e.g. from ode45 or a Simulink
%               "To Workspace" block)
%     q1      : cart displacement q1(t), same length as t   [m]
%     q2      : pendulum angle q2(t), same length as t      [rad]
%     u       : control input u(t), same length as t        [N.m]
%     params  : struct with fields l (arm length), CartRadius (= wheel
%               radius R, used both for drawing and for the rolling
%               constraint phi = q1/R)
%   OPTIONAL NAME-VALUE PAIRS
%     'VideoName' : if provided, saves the animation to this .mp4 file
%     'FPS'       : playback / export frame rate (default 30)
%     'ShowLive'  : if false and VideoName is set, hides the figure while
%                   rendering (faster export, no live preview). Default true.
%% KINEMATIC CONVENTION
%   x_cart      = q1
%   x_pendulum  = q1 - l * sin(q2)
%   y_pendulum  = CartRadius + l * cos(q2)
%
%   Rolling without slipping (wheel-driven actuation model):
%   phi = -q1 / CartRadius   (minus sign: rolling in +x spins clockwise)
    clf(gcf)
    p = inputParser;
    addParameter(p, 'VideoName', '');
    addParameter(p, 'FPS', 30);
    addParameter(p, 'q1_ref', []);
    addParameter(p, 'q2_ref', []);
    addParameter(p, 'ShowLive', true);
    parse(p, varargin{:});
    videoName = p.Results.VideoName;
    fps       = p.Results.FPS;
    q1_ref    = p.Results.q1_ref;
    q2_ref    = p.Results.q2_ref;
    showLive  = p.Results.ShowLive;

    l = params.l;
    CartRadius = params.CartRadius;   % also used as the wheel radius R

    %% Resample
    tUniform = (t(1):1/fps:t(end))';
    q1U      = interp1(t,q1,tUniform);
    q2U      = interp1(t,q2,tUniform);
    uU       = interp1(t,u,tUniform);

    ref1 = false;
    ref2 = false;
    if ~isempty(q1_ref)
        q1_refU = interp1(t,q1_ref, tUniform);
        ref1 = true;
    end
    if ~isempty(q2_ref)
        q2_refU = interp1(t,q2_ref, tUniform);
        ref2 = true;
    end

    saveVideo = ~isempty(videoName);

    % Precompute per-frame kinematics once (avoid recomputing in loop)
    x_pendulumU = q1U - l*sin(q2U);
    y_pendulumU = CartRadius + l*cos(q2U);

    % NEW: precompute 3 wheel spokes (120 deg apart), rolling without slipping.
    % All 3 spokes share one line handle, using NaN to break the path into
    % disconnected segments (cheaper than 3 separate plot handles).
    nSpokes  = 3;
    spokeOff = (0:nSpokes-1) * (2*pi/nSpokes);     % [0, 120, 240] deg
    phiU     = -q1U / CartRadius;                  % wheel spin angle

    spokesXU = nan(length(tUniform), nSpokes*3);   % 3 cols per spoke: [hub, tip, NaN]
    spokesYU = nan(length(tUniform), nSpokes*3);
    for s = 1:nSpokes
        spokeAngleU = pi/2 + phiU + spokeOff(s);
        tipX = q1U + CartRadius*cos(spokeAngleU);
        tipY = CartRadius + CartRadius*sin(spokeAngleU);
        cols = (s-1)*3 + (1:2);                    % leave 3rd column as NaN
        spokesXU(:, cols) = [q1U, tipX];
        spokesYU(:, cols) = [CartRadius*ones(size(tipY)), tipY];
    end

    %% UNE seule figure, un seul layout
    fig = figure(1);
    if saveVideo && ~showLive
        fig.Visible = 'off';   % skip screen compositing when only exporting
    end
    set(fig, 'GraphicsSmoothing', 'off');

    % --- panneau animation (gauche) ---
    ax = subplot(2,2,1); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
    viewWidth = 4;
    xlim(ax, [q1U(1)-viewWidth/2, q1U(1)+viewWidth/2]);
    ylim(ax, [-2*CartRadius, l+CartRadius]);
    xlabel(ax,'x [m]'); ylabel(ax,'y [m]');
    hTitle = title(ax,'t = 0.00 s');   % create once, update string later

    groundMargin = 5;
    plot(ax, [min(q1U)-groundMargin, max(q1U)+groundMargin], [-0.02 -0.02], ...
        'Color','black','LineWidth',5);
    angle = linspace(0,2*pi,100);
    disk_x = CartRadius*sin(angle); disk_y = CartRadius+CartRadius*cos(angle);
    cartPatch = patch(ax, 'XData', disk_x, 'YData', disk_y, 'FaceColor',[0.3 0.3 0.6]);

    % NEW: spokes marking the wheel's rotation (3 spokes, one line handle)
    spokeLine = plot(ax, spokesXU(1,:), spokesYU(1,:), 'w-', 'LineWidth', 2);

    refLine = plot(ax, [0 0], [CartRadius, CartRadius+l], '--', 'Color',[0.2 0.7 0.2]);
    rodLine = plot(ax, [0 0], [0 0], 'k-', 'LineWidth', 2);
    massMarker = plot(ax, 0, 0, 'o', 'MarkerSize',15,'MarkerFaceColor','r','MarkerEdgeColor','r');

    % --- panneau q1(t) (haut droite) ---
    ax2 = subplot(2,2,2); hold(ax2,'on'); grid(ax2,'on');
    xlim(ax2, [tUniform(1) tUniform(end)]);
    ylim(ax2, [min(q1U) max(q1U)] + [-0.1 0.1]);
    ylabel(ax2,'q_1 [m]'); xlabel(ax2,'Time [s]');
    q1_line = animatedline(ax2, 'Color','b');
    if ref1
        q1_ref_line = animatedline(ax2, 'Color','r');
        legend(ax2, 'q_1', 'q_{1 ref}');
    end

    % --- panneau q2(t) (bas droite) ---
    ax4 = subplot(2,2,4); hold(ax4,'on'); grid(ax4,'on');
    xlim(ax4, [tUniform(1) tUniform(end)]);
    ylim(ax4, [min(q2U) max(q2U)] + [-0.1 0.1]);
    ylabel(ax4,'q_2 [rad]'); xlabel(ax4,'Time [s]');
    q2_line = animatedline(ax4, 'Color','r');
    if ref2
        q2_ref_line = animatedline(ax4, 'Color','g');
        legend(ax4, 'q_2', 'q_{2 ref}');   % FIX: was mistakenly labeled q_1/q_{1 ref}
    end

    % --- panneau u(t) (bas gauche) ---
    ax3 = subplot(2,2,3); hold(ax3,'on'); grid(ax3,'on');
    xlim(ax3, [tUniform(1) tUniform(end)]);
    ylim(ax3, [min(uU) max(uU)] + [-0.1 0.1]);
    ylabel(ax3,'u [N.m]'); xlabel(ax3,'Time [s]');
    u_line = animatedline(ax3, 'Color','g');

    % Strip interactive chrome (toolbar/hit-testing) — cheap, avoids
    % per-frame overhead from axes interaction handling
    for a = [ax ax2 ax3 ax4]
        try
            a.Toolbar.Visible = 'off';
            disableDefaultInteractivity(a);
        catch
            % older MATLAB versions may not support these; safe to skip
        end
    end

    %% Video writer
    if saveVideo
        vw = VideoWriter(videoName, 'MPEG-4'); vw.FrameRate = fps; open(vw);
    end

    %% Boucle
    for k = 1:length(tUniform)
        x_cart = q1U(k);
        x_pendulum = x_pendulumU(k);
        y_pendulum = y_pendulumU(k);

        xlim(ax, [x_cart-viewWidth/2, x_cart+viewWidth/2]);   % caméra suit le chariot
        set(cartPatch, 'XData', x_cart + disk_x, 'YData', disk_y);
        set(spokeLine, 'XData', spokesXU(k,:), 'YData', spokesYU(k,:));
        set(rodLine, 'XData',[x_cart, x_pendulum], 'YData',[CartRadius, y_pendulum]);
        set(massMarker, 'XData', x_pendulum, 'YData', y_pendulum);
        set(refLine, 'XData',[x_cart, x_cart], 'YData',[CartRadius, CartRadius+l]);

        addpoints(u_line, tUniform(k), uU(k));
        addpoints(q1_line, tUniform(k), x_cart);
        addpoints(q2_line, tUniform(k), q2U(k));
        if ref1
            addpoints(q1_ref_line, tUniform(k), q1_refU(k));
        end
        if ref2
            addpoints(q2_ref_line, tUniform(k), q2_refU(k));
        end

        set(hTitle, 'String', sprintf('t = %.2f s', tUniform(k)));

        if saveVideo
            drawnow;                 % full draw so getframe captures the right frame
            writeVideo(vw, getframe(fig));
        else
            % drawnow limitrate;       % throttled draw is enough for live viewing
            pause(1/fps);            % only pace playback when there's no video clock
        end
    end

    if saveVideo
        close(vw);
        fprintf('Animation saved to %s\n', videoName);
    end
end