% === Leer datos ===
data = readtable('vuelo_datos.csv');
x_k = data.x_kalman; y_k = data.y_kalman; z_k = data.z_kalman;
x_lh = data.x_lh;     y_lh = data.y_lh;     z_lh = data.z_lh;
roll = deg2rad(data.roll);
pitch = deg2rad(data.pitch);
yaw = deg2rad(data.yaw);
t = data.time;
n = length(t);

% === Preguntar si se quiere grabar video ===
guardar = input('Vous voulez sauvegarder la vidéo .mp4? (y/n): ', 's');
grabar = strcmpi(guardar, 'y');

% === Estilo visual ===
fig = figure('Color', [0.05 0.05 0.05], 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);
axis equal;
hold on;
grid on;
xlabel('X (m)', 'Color', 'w');
ylabel('Y (m)', 'Color', 'w');
zlabel('Z (m)', 'Color', 'w');
title('Position corrigée avec lighthouse', 'Color', 'w');
set(gca, 'Color', [0.05 0.05 0.05], 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');
view(3);

margin = 0.05;
xlim([min([x_k; x_lh])-margin, max([x_k; x_lh])+margin]);
ylim([min([y_k; y_lh])-margin, max([y_k; y_lh])+margin]);
zlim([min([z_k; z_lh])-margin, max([z_k; z_lh])+margin]);

light('Position', [1 1 1], 'Style', 'infinite');
material dull;

% === Trayectorias ===
traj_k = animatedline('Color', [0.2 0.6 1], 'LineWidth', 2, 'DisplayName', 'Kalman');
traj_lh = animatedline('Color', [1 0.4 0.4], 'LineStyle', '--', 'LineWidth', 2, 'DisplayName', 'Lighthouse');

% === Emoji o símbolo del dron ===
drone_emoji = '🚁';
emoji = text(x_k(1), y_k(1), z_k(1), drone_emoji, 'FontSize', 20, 'HorizontalAlignment', 'center','Color','w');

% === Flechas de orientación ===
arrow_len = 0.1;
fwd_arrow = quiver3(x_k(1), y_k(1), z_k(1), 0, 0, 0, ...
    'Color', 'y', 'LineWidth', 2, 'DisplayName', 'Axe X');
side_arrow = quiver3(x_k(1), y_k(1), z_k(1), 0, 0, 0, ...
    'Color', 'g', 'LineWidth', 2, 'DisplayName', 'Axe Y');
up_arrow = quiver3(x_k(1), y_k(1), z_k(1), 0, 0, 0, ...
    'Color', 'r', 'LineWidth', 2, 'DisplayName', 'Axe Z');

legend('TextColor', 'w', 'Location', 'northwest');

% === Preparar video si aplica ===
if grabar
    video = VideoWriter('vol.mp4', 'MPEG-4');
    video.Quality = 100;
    video.FrameRate = 1 / median(diff(t));
    open(video);
end

% === Animación ===
for i = 1:n-1
    addpoints(traj_k, x_k(i), y_k(i), z_k(i));
    addpoints(traj_lh, x_lh(i), y_lh(i), z_lh(i));
    set(emoji, 'Position', [x_k(i), y_k(i), z_k(i)]);

    R = eul2rotm([yaw(i), pitch(i), roll(i)], 'ZYX');
    dir_x = R(:,1); dir_y = R(:,2); dir_z = R(:,3);

    set(fwd_arrow, 'XData', x_k(i), 'YData', y_k(i), 'ZData', z_k(i), ...
        'UData', arrow_len * dir_x(1), 'VData', arrow_len * dir_x(2), 'WData', arrow_len * dir_x(3));
    set(side_arrow, 'XData', x_k(i), 'YData', y_k(i), 'ZData', z_k(i), ...
        'UData', arrow_len * dir_y(1), 'VData', arrow_len * dir_y(2), 'WData', arrow_len * dir_y(3));
    set(up_arrow, 'XData', x_k(i), 'YData', y_k(i), 'ZData', z_k(i), ...
        'UData', arrow_len * dir_z(1), 'VData', arrow_len * dir_z(2), 'WData', arrow_len * dir_z(3));

    drawnow;

    if grabar
        frame = getframe(fig);
        writeVideo(video, frame);
    end

    pause(t(i+1) - t(i));
end

% === Cierre de video ===
if grabar
    close(video);
    disp('Vidéo gardée comme vol.mp4');
else
    disp('Sans garder :(.');
end
