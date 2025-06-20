% Prueba 1 como en python

data = readtable('vuelo_datos.csv');

x_k = data.x_kalman;
y_k = data.y_kalman;
z_k = data.z_kalman;

x_lh = data.x_lh;
y_lh = data.y_lh;
z_lh = data.z_lh;

t = data.time;
n = length(t);

figure;
hold on;
grid on;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('Animación 3D Kalman vs Lighthouse');
xlim([min([x_k; x_lh]) max([x_k; x_lh])]);
ylim([min([y_k; y_lh]) max([y_k; y_lh])]);
zlim([min([z_k; z_lh]) max([z_k; z_lh])]);
view(3);

kalman_traj = animatedline('Color', 'b', 'LineWidth', 2, 'DisplayName', 'Kalman');
lh_traj = animatedline('Color', 'r', 'LineStyle', '--', 'LineWidth', 2, 'DisplayName', 'Lighthouse');
legend('show', 'Location', 'northwest');

for i = 1:n
    addpoints(kalman_traj, x_k(i), y_k(i), z_k(i));
    addpoints(lh_traj, x_lh(i), y_lh(i), z_lh(i));
    drawnow;

    if i < n
        pause(t(i+1) - t(i));  % sincronización real
    end
end


%% Prueba 2 video matlab

% === Leer CSV ===
data = readtable('vuelo_datos.csv');
x_k = data.x_kalman; y_k = data.y_kalman; z_k = data.z_kalman;
x_lh = data.x_lh;     y_lh = data.y_lh;     z_lh = data.z_lh;
t = data.time;
n = length(t);

% === Inicializar figura ===
figure;
axis equal;
hold on;
grid on;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('Animación 3D Kalman vs Lighthouse con marcador simbólico');
xlim([min([x_k; x_lh]) max([x_k; x_lh])]);
ylim([min([y_k; y_lh]) max([y_k; y_lh])]);
zlim([min([z_k; z_lh]) max([z_k; z_lh])]);
view(3);

traj_k = animatedline('Color', 'b', 'LineWidth', 2, 'DisplayName', 'Kalman');
traj_lh = animatedline('Color', 'r', 'LineStyle', '--', 'LineWidth', 2, 'DisplayName', 'Lighthouse');
legend('show');

% === Marcador móvil ===
% dron = plot3(x_k(1), y_k(1), z_k(1), 'ko', 'MarkerFaceColor', 'g', 'MarkerSize', 10);
dron = text(x_k(1), y_k(1), z_k(1), '🚁', 'FontSize', 18);

% === Animación ===
for i = 1:n
    addpoints(traj_k, x_k(i), y_k(i), z_k(i));
    addpoints(traj_lh, x_lh(i), y_lh(i), z_lh(i));

    % Mover el "dron" marcador
    % set(dron, 'XData', x_k(i), 'YData', y_k(i), 'ZData', z_k(i));
    set(dron, 'Position', [x_k(i), y_k(i), z_k(i)]);

    drawnow;

    if i < n
        pause(t(i+1) - t(i));
    end
end