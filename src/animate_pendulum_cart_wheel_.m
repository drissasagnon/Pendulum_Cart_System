function animate_pendulum_cart_wheel(t, q1, q2, params, varargin)
%ANIMATE_PENDULUM_CART Animate the wheel-driven cart-pendulum system.
%
%   animate_pendulum_cart(t, q1, q2, params)
%   animate_pendulum_cart(t, q1, q2, params, 'VideoName', 'out.mp4')
%
%   INPUTS
%     t       : time vector, any sampling
%     q1      : cart/wheel-center displacement q1(t)         [m]
%     q2      : pendulum angle q2(t)                         [rad]
%     params  : struct with fields L, CartRadius (= wheel radius R)
%
%   OPTIONAL NAME-VALUE PAIRS
%     'VideoName' : if provided, saves the animation to this .mp4 file
%     'FPS'       : playback / export frame rate (default 30)
%
%   NEW IN THIS VERSION: a spoke is drawn on the wheel and rotates
%   according to the rolling-without-slipping constraint q1 = R*phi,
%   i.e. phi = q1/R -- this makes the wheel visibly "roll" as the cart
%   translates, consistent with the wheel-driven actuation model.

    p = inputParser;
    addParameter(p, 'VideoName', '');
    addParameter(p, 'FPS', 30);
    parse(p, varargin{:});
    videoName = p.Results.VideoName;
    fps       = p.Results.FPS;

    l = params.L;
    R = params.CartRadius;   % wheel radius (used both for drawing and rolling)

    %% Resample to a uniform time base for smooth playback
    tUniform = (t(1):1/fps:t(end))';
    q1U      = interp1(t, q1, tUniform);
    q2U      = interp1(t, q2, tUniform);

    %% Geometry
    CartRadius = R;
    pivotY = CartRadius;   % pendulum pivot sits on top of the wheel

    %% Figure setup
    fig = figure(1);

    ax = subplot(2,2,[1,3]); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
    viewWidth = 4;
    xlim(ax, [q1U(1)-viewWidth/2, q1U(1)+viewWidth/2]);
    ylim(ax, [-2*CartRadius, l+CartRadius]);
    xlabel(ax,'x [m]'); ylabel(ax,'y [m]'); title(ax,'t = 0.00 s');

    groundMargin = 5;
    plot(ax, [min(q1U)-groundMargin, max(q1U)+groundMargin], [-0.02 -0.02], ...
         'Color','black','LineWidth',5);

    angle = linspace(0,2*pi,100);
    disk_x = CartRadius*sin(angle); disk_y = CartRadius+CartRadius*cos(angle);
    cartPatch = patch(ax, 'XData', disk_x, 'YData', disk_y, 'FaceColor',[0.3 0.3 0.6]);

    % NEW: spoke marking the wheel's rotation (rolling without slipping)
    spokeLine = plot(ax, [0 0], [CartRadius CartRadius], 'w-', 'LineWidth', 2);

    refLine = plot(ax, [0 0], [CartRadius, CartRadius+l], '--', 'Color',[0.2 0.7 0.2]);
    rodLine = plot(ax, [0 0], [0 0], 'k-', 'LineWidth', 2);
    massMarker = plot(ax, 0, 0, 'o', 'MarkerSize',15,'MarkerFaceColor','r','MarkerEdgeColor','r');

    % --- panneau q1(t) ---
    ax2 = subplot(2,2,2); hold(ax2,'on'); grid(ax2,'on');
    xlim(ax2, [tUniform(1) tUniform(end)]);
    ylim(ax2, [min(q1U) max(q1U)] + [-0.05 0.05]);
    ylabel(ax2,'q_1 [m]'); xlabel(ax2,'Time [s]');
    q1_line = animatedline(ax2, 'Color','b');

    % --- panneau q2(t) ---
    ax3 = subplot(2,2,4); hold(ax3,'on'); grid(ax3,'on');
    xlim(ax3, [tUniform(1) tUniform(end)]);
    ylim(ax3, [min(q2U) max(q2U)] + [-0.05 0.05]);
    ylabel(ax3,'q_2 [rad]'); xlabel(ax3,'Time [s]');
    q2_line = animatedline(ax3, 'Color','r');

    %% Video writer
    saveVideo = ~isempty(videoName);
    if saveVideo
        vw = VideoWriter(videoName, 'MPEG-4'); vw.FrameRate = fps; open(vw);
    end

    %% Boucle
    for k = 1:length(tUniform)
        x_cart = q1U(k);
        theta  = q2U(k);

        xlim(ax, [x_cart-viewWidth/2, x_cart+viewWidth/2]);

        cartX = x_cart + disk_x; cartY = disk_y;
        set(cartPatch, 'XData', cartX, 'YData', cartY);

        % Rolling-without-slipping: phi = q1/R.
        % A wheel moving in +x spins CLOCKWISE, hence the minus sign.
        phi = -x_cart / R;
        spokeAngle = pi/2 + phi;         % start pointing "up" at t=0
        spokeX = x_cart + [0, CartRadius*cos(spokeAngle)];
        spokeY = CartRadius + [0, CartRadius*sin(spokeAngle)];
        set(spokeLine, 'XData', spokeX, 'YData', spokeY);

        x_pendulum = x_cart - l*sin(theta);
        y_pendulum = CartRadius + l*cos(theta);
        set(rodLine, 'XData',[x_cart, x_pendulum], 'YData',[CartRadius, y_pendulum]);
        set(massMarker, 'XData', x_pendulum, 'YData', y_pendulum);
        set(refLine, 'XData',[x_cart, x_cart], 'YData',[CartRadius, CartRadius+l]);

        addpoints(q1_line, tUniform(k), x_cart);
        addpoints(q2_line, tUniform(k), theta);

        title(ax, sprintf('t = %.2f s', tUniform(k)));
        drawnow limitrate;
        pause(1/fps);

        if saveVideo, writeVideo(vw, getframe(fig)); end
    end

    if saveVideo, close(vw); fprintf('Animation saved to %s\n', videoName); end
end
