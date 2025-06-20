function writeSimpleDroneSTL(filename)
    % Parámetros del fuselaje
    bodyL = 0.04;    % 4 cm
    bodyT = 0.01;    % 1 cm de grosor
    % Parámetros de los rotores
    rotorL = 0.01;   % 1 cm
    rotorT = 0.005;  % 0.5 cm grosor
    
    % Vértices de un cubo unitario centrado en el origen:
    V0 = [ -0.5 -0.5 -0.5
           +0.5 -0.5 -0.5
           +0.5 +0.5 -0.5
           -0.5 +0.5 -0.5
           -0.5 -0.5 +0.5
           +0.5 -0.5 +0.5
           +0.5 +0.5 +0.5
           -0.5 +0.5 +0.5 ];
    % Caras (dos triángulos por cara)
    F0 = [1 2 3; 1 3 4;  % fondo
          5 8 7; 5 7 6;  % tapa
          1 5 6; 1 6 2;  % frontal
          2 6 7; 2 7 3;  % derecha
          3 7 8; 3 8 4;  % trasera
          4 8 5; 4 5 1]; % izquierda

    % Lista de todos los par (V,F) de cubos: fuselaje + 4 rotores
    cubes = {};
    % 1) Fuselaje
    cubes{end+1} = struct(...
        'V', V0 .* [bodyL bodyL bodyT], ...
        'F', F0);
    % 2‑5) Rotores en (+x,0), (–x,0), (0,+y), (0,–y)
    armDist = bodyL/2; 
    rotorOffsets = [ armDist  0  0
                    -armDist  0  0
                     0  armDist 0
                     0 -armDist 0];
    for k=1:4
        offs = rotorOffsets(k,:);
        cubes{end+1} = struct(...
            'V', V0 .* [rotorL rotorL rotorT] + offs, ...
            'F', F0);
    end

    % Abrir fichero STL ASCII
    fid = fopen(filename,'w');
    fprintf(fid,'solid drone\n');
    for c=1:numel(cubes)
        V = cubes{c}.V;
        F = cubes{c}.F;
        for i=1:size(F,1)
            tri = F(i,:);
            v1 = V(tri(1),:);
            v2 = V(tri(2),:);
            v3 = V(tri(3),:);
            % cálculo normal
            n = cross(v2-v1, v3-v1);
            n = n / norm(n + eps);
            fprintf(fid,' facet normal %.6f %.6f %.6f\n',n);
            fprintf(fid,'  outer loop\n');
            fprintf(fid,'   vertex %.6f %.6f %.6f\n',v1);
            fprintf(fid,'   vertex %.6f %.6f %.6f\n',v2);
            fprintf(fid,'   vertex %.6f %.6f %.6f\n',v3);
            fprintf(fid,'  endloop\n');
            fprintf(fid,' endfacet\n');
        end
    end
    fprintf(fid,'endsolid drone\n');
    fclose(fid);
    fprintf('STL generado en %s\n', filename);
end
