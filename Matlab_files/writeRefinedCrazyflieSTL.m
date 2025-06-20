function writeRefinedCrazyflieSTL(filename)
    % === Parámetros ajustados al diseño del Crazyflie 2.1 ===
    bodyL = 0.025; bodyW = 0.025; bodyH = 0.008;
    armL = 0.04;   armW = 0.0025; armH = 0.0025;
    motorR = 0.003; motorH = 0.01;
    propR = 0.01;  propT = 0.0008;
    offset = 0.026;

    FV = []; FF = []; offsetIndex = 0;

    % === Cuerpo central ===
    [V,F] = makeTriangulatedCuboid(bodyL, bodyW, bodyH, [0 0 0]);
    FV = [FV; V]; FF = [FF; F + offsetIndex]; offsetIndex = offsetIndex + size(V,1);

    % === Posiciones diagonales de los brazos ===
    d = offset / sqrt(2);
    arm_pos = [ d,  d, 0;
               -d,  d, 0;
               -d, -d, 0;
                d, -d, 0];

    % === Brazos en cruz (inclinados) ===
    arm_dirs = [ cos(pi/4), sin(pi/4);
                 cos(3*pi/4), sin(3*pi/4);
                 cos(-3*pi/4), sin(-3*pi/4);
                 cos(-pi/4), sin(-pi/4) ];
    for i = 1:4
        center = arm_pos(i,:);
        rot = atan2(arm_dirs(i,2), arm_dirs(i,1));
        Rz = [cos(rot), -sin(rot), 0;
              sin(rot),  cos(rot), 0;
              0        , 0        , 1];
        [V,F] = makeTriangulatedCuboid(armL, armW, armH, [0 0 0]);
        V = (Rz * V.').';
        V = V + center;
        FV = [FV; V]; FF = [FF; F + offsetIndex]; offsetIndex = offsetIndex + size(V,1);
    end

    % === Motores ===
    for i = 1:4
        dir2D = arm_dirs(i,:);
        end_pos = arm_pos(i,1:2) + (armL/2) * dir2D;
        pos = [end_pos, motorH/2];
        [V,F] = makeTriangulatedCylinder(motorR, motorH, 16, pos);
        FV = [FV; V]; FF = [FF; F + offsetIndex]; offsetIndex = offsetIndex + size(V,1);
    end

    % === Hélices encima del motor ===
    for i = 1:4
        dir2D = arm_dirs(i,:);
        end_pos = arm_pos(i,1:2) + (armL/2) * dir2D;
        pos = [end_pos, motorH + propT/2];
        [V,F] = makeTriangulatedCylinder(propR, propT, 12, pos);
        FV = [FV; V]; FF = [FF; F + offsetIndex]; offsetIndex = offsetIndex + size(V,1);
    end

    % === Exportar STL ===
    fid = fopen(filename, 'w');
    fprintf(fid, 'solid drone\n');
    for i = 1:size(FF,1)
        v1 = FV(FF(i,1),:); v2 = FV(FF(i,2),:); v3 = FV(FF(i,3),:);
        n  = cross(v2 - v1, v3 - v1);
        n  = n / norm(n + eps);
        fprintf(fid, ' facet normal %.6f %.6f %.6f\n', n);
        fprintf(fid, '  outer loop\n');
        fprintf(fid, '   vertex %.6f %.6f %.6f\n', v1);
        fprintf(fid, '   vertex %.6f %.6f %.6f\n', v2);
        fprintf(fid, '   vertex %.6f %.6f %.6f\n', v3);
        fprintf(fid, '  endloop\n');
        fprintf(fid, ' endfacet\n');
    end
    fprintf(fid, 'endsolid drone\n');
    fclose(fid);
    fprintf('✅ STL refinado exportado como %s\n', filename);
end

% ==== FUNCIONES AUXILIARES ====
function [V,F] = makeTriangulatedCuboid(L, W, H, center)
    [X,Y,Z] = ndgrid([-0.5,0.5]*L, [-0.5,0.5]*W, [-0.5,0.5]*H);
    V = [X(:), Y(:), Z(:)] + center;
    F = [1 2 4; 1 4 3; 5 8 7; 5 7 6;
         1 5 6; 1 6 2; 2 6 7; 2 7 4;
         4 7 8; 4 8 3; 3 8 5; 3 5 1];
end

function [V,F] = makeTriangulatedCylinder(r, h, n, center)
    F = [];
    theta = linspace(0, 2*pi, n+1); theta(end) = [];
    x = r * cos(theta); y = r * sin(theta);
    V_bot = [x', y', -h/2 * ones(n,1)];
    V_top = [x', y',  h/2 * ones(n,1)];
    V = [V_bot; V_top];
    for i = 1:n
        i2 = mod(i, n) + 1;
        F(end+1,:) = [i, i2, i2+n];
        F(end+1,:) = [i, i2+n, i+n];
    end
    V = [V; 0,0,-h/2]; cb = size(V,1);
    for i = 1:n
        i2 = mod(i,n)+1; F(end+1,:) = [i2, i, cb];
    end
    V = [V; 0,0,h/2]; ct = size(V,1);
    for i = 1:n
        i2 = mod(i,n)+1; F(end+1,:) = [i+n, i2+n, ct];
    end
    V = V + center;
end
