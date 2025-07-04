%% Script final con STL, orientación, trayectorias y motores (sin arrow_len manual)

writeRefinedCrazyflieSTL("drone.stl");
gm = fegeometry("drone.stl");
pdegplot(gm,FaceLabels="on",FaceAlpha=0.3)

% === 1) Leer CSV ===
data = readtable('vuelo_datos10.csv');
x_k = data.x_kalman;   y_k = data.y_kalman;   z_k = data.z_kalman;
x_lh = data.x_lh;      y_lh = data.y_lh;      z_lh = data.z_lh;
roll  = deg2rad(data.roll);    pitch = deg2rad(data.pitch);    yaw = deg2rad(data.yaw);
m1 = data.motor_m1;    m2 = data.motor_m2;    m3 = data.motor_m3;    m4 = data.motor_m4;
t = data.time;         n = length(t);
f_val = data.F;
r_val = data.R;
z_val = data.Z; 


% === 2) Leer STL ===
[faces, verts0] = readSTLpatch('drone.stl');

% === 3) Posiciones locales de los rotores corregidas (en extremos) ===
armDist = 0.04/2 + 0.012;
rotorOffsets = [
    +armDist, -armDist, 0;  % motor 1: arriba derecha
    -armDist, -armDist, 0;  % motor 2: abajo derecha
    -armDist, +armDist, 0;  % motor 3: abajo izquierda
    +armDist, +armDist, 0   % motor 4: arriba izquierda
];
motor_pwms = [m1, m2, m3, m4];
max_pwm = max(motor_pwms, [], 'all');
norm_pwms = motor_pwms / max_pwm;  % escala [0,1]

% === 4) Preguntar grabación ===
resp = input('¿Grabar en MP4? (y/n): ','s');
grabar = strcmpi(resp,'y');
if grabar
    v = VideoWriter('cf21_flight.mp4','MPEG-4');
    v.Quality   = 100;
    v.FrameRate = 1/median(diff(t));
    open(v);
end

% === 5) Figura ===
fig = figure('Color',[0.05 0.05 0.05],...
             'Units','normalized','OuterPosition',[0 0 1 1]);
ax = axes('Parent',fig); axis normal; hold(ax,'on'); grid(ax,'on');
set(ax,'XColor','w','YColor','w','ZColor','w','Color',[0.05 0.05 0.05]);
xlabel(ax,'X (m)','Color','w');   ylabel(ax,'Y (m)','Color','w');   zlabel(ax,'Z (m)','Color','w');
title(ax,'Crazyflie 2.1 – Trayectoria & Motores','Color','w');
m = 0.05;
xlim([min([x_k;x_lh])-m, max([x_k;x_lh])+m]);
ylim([min([y_k;y_lh])-m, max([y_k;y_lh])+m]);
zlim([min([f_val;z_k;z_lh])-m, max([r_val;z_k;z_lh])+m]);
% pbaspect(ax, [4 4 1]);
% pbaspect auto;

light('Parent',ax,'Position',[1 1 5],'Style','infinite');
material(ax,'dull');

% === 6) Trayectorias ===
hKalman = animatedline(ax,'Color',[0.4 0.6 1],'LineWidth',2,'DisplayName','Kalman');
hLh     = animatedline(ax,'Color',[1 0.4 0.4],'LineStyle','--','LineWidth',2,'DisplayName','Lighthouse');

% === 7) Dron STL ===
drone_patch = patch(ax,'Faces',faces,'Vertices',verts0,...
    'FaceColor',[0.3 0.3 0.9],'EdgeColor','none',...
    'FaceLighting','gouraud','AmbientStrength',0.3,'HandleVisibility','off');

% === 8) Flechas de orientación general ===
scale_ref = 0.05;
hX = quiver3(ax,0,0,0,0,0,0,'y','LineWidth',2,'MaxHeadSize',1,'DisplayName','Axe X');
hY = quiver3(ax,0,0,0,0,0,0,'g','LineWidth',2,'MaxHeadSize',1,'DisplayName','Axe Y');
hZ = quiver3(ax,0,0,0,0,0,0,'r','LineWidth',2,'MaxHeadSize',1,'DisplayName','Axe Z');

% === 9) Flechas de los rotores (colores azules con leves diferencias) ===
rotor_colors = [
    0.0, 0.6, 1.0;   % Motor 1
    0.0, 0.7, 1.0;   % Motor 2
    0.0, 0.8, 1.0;   % Motor 3
    0.0, 0.9, 1.0    % Motor 4
];
rotor_arrows = gobjects(4,1);
for k = 1:4
    rotor_arrows(k) = quiver3(ax,0,0,0,0,0,0, ...
        'Color', rotor_colors(k,:), ...
        'LineWidth',1.5,'MaxHeadSize',1,'AutoScale','off', ...
        'DisplayName',sprintf('Motor %d',k));
end

% === 9.2) Acumulación SLAM de puntos estimados de obstáculos y techo ===
obs_f_X = []; obs_f_Y = []; obs_f_Z = [];
obs_r_X = []; obs_r_Y = []; obs_r_Z = [];

h_obs_f = scatter3(ax, [], [], [], 10, [1, 0.5, 0], 'filled', 'DisplayName','Obstáculo Inferior','MarkerFaceColor','flat');
h_obs_r = scatter3(ax, [], [], [], 10, [0.4, 0.6, 1], 'filled', 'DisplayName','Techo');


legend(ax, [hKalman,hLh,hX,hY,hZ,rotor_arrows',h_obs_f,h_obs_r], ...
    {'Kalman','Lighthouse','Axe X','Axe Y','Axe Z', ...
     'Motor 1','Motor 2','Motor 3','Motor 4', ...
     'Obstáculo Inferior','Techo'}, ...
    'TextColor','w','Location','northwest');


% === 11) Animación ===
for i = 1:n-1
    % Trayectorias
    addpoints(hKalman, x_k(i), y_k(i), z_k(i));
    addpoints(hLh,     x_lh(i), y_lh(i), z_lh(i));
    

    % Rotación cuerpo→mundo y patch
    R = eul2rotm([yaw(i),pitch(i),roll(i)],'ZYX');
    verts = (R*verts0.').' + [x_k(i),y_k(i),z_k(i)];
    set(drone_patch,'Vertices',verts);

    % Orientación general
    dir = R(:,1:3);
    set(hX, 'XData',x_k(i),'YData',y_k(i),'ZData',z_k(i), ...
            'UData',scale_ref*dir(1,1),'VData',scale_ref*dir(2,1),'WData',scale_ref*dir(3,1));
    set(hY, 'XData',x_k(i),'YData',y_k(i),'ZData',z_k(i), ...
            'UData',scale_ref*dir(1,2),'VData',scale_ref*dir(2,2),'WData',scale_ref*dir(3,2));
    set(hZ, 'XData',x_k(i),'YData',y_k(i),'ZData',z_k(i), ...
            'UData',scale_ref*dir(1,3),'VData',scale_ref*dir(2,3),'WData',scale_ref*dir(3,3));
   
    % === SLAM: acumular puntos estimados para f y r ===
    xf = x_k(i);  yf = y_k(i);

    obs_f_X(end+1) = xf;
    obs_f_Y(end+1) = yf;
    obs_f_Z(end+1) = f_val(i);

    obs_r_X(end+1) = xf;
    obs_r_Y(end+1) = yf;
    obs_r_Z(end+1) = r_val(i);

    set(h_obs_f, 'XData', obs_f_X, 'YData', obs_f_Y, 'ZData', obs_f_Z);
    set(h_obs_r, 'XData', obs_r_X, 'YData', obs_r_Y, 'ZData', obs_r_Z);




    % Motores: longitud proporcional al PWM normalizado
   for k = 1:4
        p_local  = rotorOffsets(k,:);
        p_world  = (R*p_local.').' + [x_k(i),y_k(i),z_k(i)];
        up       = R*[0;0;1];
        len_k    = scale_ref * norm_pwms(i,k);
        set(rotor_arrows(k), ...
            'XData',p_world(1),'YData',p_world(2),'ZData',p_world(3), ...
            'UData',len_k*up(1),'VData',len_k*up(2),'WData',len_k*up(3));
    end

    % Suave rotación de cámara
    az = 30 + 0.25*i;
    view(ax,az,20);

    drawnow;
    if grabar
        writeVideo(v,getframe(fig));
    end
    pause(t(i+1)-t(i));
end

% === 12) Cerrar video ===
if grabar
    close(v);
    disp('✅ Video guardado: cf21_flight.mp4');
else
    disp('🎬 Animación finalizada sin guardar');
end


%% Función auxiliar para leer STL ASCII
function [F, V] = readSTLpatch(filename)
    fid = fopen(filename,'r');
    V = []; F = []; tris = []; vmap = containers.Map(); idx = 0;
    while ~feof(fid)
        line = strtrim(fgetl(fid));
        if startsWith(line,'vertex')
            c = sscanf(line,'vertex %f %f %f')';
            key = sprintf('%.6f,%.6f,%.6f',c);
            if ~isKey(vmap,key)
                idx = idx + 1;
                V(idx,:) = c;
                vmap(key) = idx;
            end
            F(end+1) = vmap(key);
            if mod(numel(F),3)==0
                tris(end+1,:) = F(end-2:end);
            end
        end
    end
    fclose(fid);
    F = tris;
end
