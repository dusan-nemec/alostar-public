clear;
close;
clc;

%% vstupne premenne
% M vektor krutiacich momentov trakcnych kolies
% A vektor krutiacich momentov natacacich kolies
% p vektor parametrov (mobilny robot a trakcna + natacacia jednotka)
% s stavovy vektor (w, vx, vy, x, y, psi, Q1, a1, a1', .... Qn, an, an')

% vnutorne premenne
% x stavovy vektor (w', vx', vy', Nx, Ny, Tx1, Ty1, N1, Q1', a1', ..., Txn, Tyn, Nn, Qn', an')

% nacitanie logov
lineID = 0;

fName = 'log90-3.txt';
fileID = fopen(fName,"r");

if (fileID < 0)
    error("error opening file %s\n\n", fName);
end

while ~feof(fileID)
    
    nline = fgetl(fileID);
    tmp = strsplit(nline);

    if (lineID == 0)
        lineID = lineID + 1;
        colName = "" + tmp;
        continue;
    end

    if (lineID == 1)
        tStamp = str2double(tmp{1});
    end

    for i=1:1:numel(tmp)
        
        arr(lineID, i) = str2double(tmp{i});

        if (i == 1)
           arr(lineID, i) = arr(lineID, i) - tStamp;
        end

    end

    lineID = lineID + 1;

end

data = array2table(arr);
data.Properties.VariableNames = colName;

fclose(fileID);


% definovanie momentov
M = 0.0 * [1.0, 1.0, 1.0, 1.0];
Mc = 0.0 * [1.0, 1.0, 1.0, 1.0];

% vektor parametrov
Jw = 0.05; % just the wheel
friction = 0.90;

p = struct(...
    'g', 9.80665, ...                                               % gravitacne zrychlenie [m/s2]
    'mr', 95, ...                                                   % hmotnost robota [kg]
    'Jr', 5.375, ...                                                % moment zotrvacnosti robota
    'xT', 0.028, ...                                                % poloha taziska mobilneho robota od stredu kinematiky :: x-ova zlozka [m]
    'yT', 0.0, ...                                                  % poloha taziska mobilneho robota od stredu kinematiky :: y-ova zlozka [m]
    'zT', 0.2, ...                                                  % poloha taziska mobilneho robota od stredu kinematiky :: z-ova zlozka [m]
    ...
    'R1', 0.1625, ...                                               % polomer kolesa [m]
    'x1', 0.2975, ...                                               % poloha kolesa vzhladom na stred kinematiky [m]
    'y1', 0.2850, ...                                               % poloha kolesa vzhladom na stred kinematiky [m]
    'e1', 0.0, ...                                                  % vyosenie kolesa (kastorove koleso) [m]
    'Jc1', 1.0, ...
    'Jw1', Jw, ...                                             % moment zotrvacnosti kolesa
    'u1', friction, ...                                                 % koeficient smykoveho trenia kolesa
    ...
    'R2', 0.1625, ...                                               % polomer kolesa [m]
    'x2', 0.2975, ...                                               % poloha kolesa vzhladom na stred kinematiky [m]
    'y2', -0.2850, ...                                              % poloha kolesa vzhladom na stred kinematiky [m]
    'e2', 0.0, ...                                                  % vyosenie kolesa (kastorove koleso) [m]
    'Jc2', 1.0, ...
    'Jw2', Jw, ...                                             % moment zotrvacnosti kolesa
    'u2', friction, ...                                                 % koeficient smykoveho trenia kolesa
    ...
    'R3', 0.1625, ...                                               % polomer kolesa [m]
    'x3', -0.2975, ...                                              % poloha kolesa vzhladom na stred kinematiky [m]
    'y3', -0.2850, ...                                              % poloha kolesa vzhladom na stred kinematiky [m]
    'e3', 0.0, ...                                                  % vyosenie kolesa (kastorove koleso) [m]
    'Jc3', 1.0, ...
    'Jw3', Jw, ...                                             % moment zotrvacnosti kolesa
    'u3', friction, ...                                                 % koeficient smykoveho trenia kolesa
    ...
    'R4', 0.1625, ...                                               % polomer kolesa [m]
    'x4', -0.2975, ...                                              % poloha kolesa vzhladom na stred kinematiky [m]
    'y4', 0.2850, ...                                               % poloha kolesa vzhladom na stred kinematiky [m]
    'e4', 0.0, ...                                                  % vyosenie kolesa (kastorove koleso) [m]
    'Jc4', 1.0, ...
    'Jw4', Jw, ...                                             % moment zotrvacnosti kolesa
    'u4', friction ...                                                  % koeficient smykoveho trenia kolesa
    );


% stavovy vektor
s = struct( ...
    'x', 0.0, ...                                                   % globalna pozicia MR :: X [m]
    'y', 0.0, ...                                                   % globalna pozicia MR :: Y [m]
    'psi', 0.0, ...                                                 % globalna orientacia MR :: PSI [rad]
    ...
    'vx', 0.0, ...                                                  % [m/s]
    'vy', 0.0, ...                                                  % [m/s]
    'w', 0.0, ...                                                   % [rad/s]
    ...
    'Q1', 0.0, ...                                                  % [rad/s]
    'a1', 0.0, ...                                                  % uhol natocenia kolesa v globalnych suradniciach:: [rad]
    'dQ1', 0.0, ...                                                 % [rad/s^2]
    'da1', 0.0, ...
    ...
    'Q2', 0.0, ...                                                  % [rad/s]
    'a2', 0.0, ...                                                  % uhol natocenia kolesa v globalnych suradniciach:: [rad]
    'dQ2', 0.0, ...                                                 % [rad/s^2]
    'da2', 0.0, ...
    ...
    'Q3', 0.0, ...                                                  % [rad/s]
    'a3', 0.0, ...                                                  % uhol natocenia kolesa v globalnych suradniciach:: [rad]
    'dQ3', 0.0, ...                                                 % [rad/s^2]
    'da3', 0.0, ...
    ...
    'Q4', 0.0, ...                                                  % [rad/s]
    'a4', 0.0, ...                                                  % uhol natocenia kolesa v globalnych suradniciach:: [rad]
    'dQ4', 0.0, ...                                                 % [rad/s^2]
    'da4', 0.0 ...
    );


%% RIESENIE

i = 1;
dt = 0.001;
prescale = int32(1);
N = int32(data.t(end) / dt) + 1;
hs = repmat(s, N/prescale, 1);
hM = zeros(N/prescale, 5);
hx = zeros(N/prescale, 37);

for k = 1:N
    
    warning('off', 'MATLAB:rankDeficientMatrix');
    
    t = double(k-1) * dt;

    if (i <= numel(data.t) && t >= data.t(i))
        Mscale = 57.0;
        M(1) = data.M1(i) / Mscale;
        M(2) = data.M3(i) / Mscale;
        M(3) = data.M4(i) / Mscale;
        M(4) = data.M2(i) / Mscale;
        
        i = i + 1;
    end

    x = solve_system(s, p, M, Mc);
    x_prev = x;

    s.w = s.w + x(1)*dt;
    s.vx = s.vx + x(2)*dt;
    s.vy = s.vy + x(3)*dt;

    s.Q1 = s.Q1 + x(9)*dt;
    s.Q2 = s.Q2 + x(14)*dt;
    s.Q3 = s.Q3 + x(19)*dt;
    s.Q4 = s.Q4 + x(24)*dt;

    s.psi = s.psi + s.w*dt;
    s.x = s.x + s.vx*dt;
    s.y = s.y + s.vy*dt;

    s.a1 = s.psi;
    s.a2 = s.psi;
    s.a3 = s.psi;
    s.a4 = s.psi;

    s.da1 = s.w;
    s.da2 = s.w;
    s.da3 = s.w;
    s.da4 = s.w;

    if mod(k, prescale) == 0
        hk = int32(k/prescale + 1);
        hs(hk) = s;
        hM(hk,1) = M(1);
        hM(hk,2) = M(2);
        hM(hk,3) = M(3);
        hM(hk,4) = M(4);
        hM(hk,5) = t;
        hx(hk,:) = x';
    end
end

%% graf
close all
figure(1);
plot(hM(:,5), hM(:,1:4));
tVal = (1:numel(hs))' * dt * double(prescale);
%xlim([25 30]);

figure(2);
plot([hs.x], [hs.y]);
xlabel('x');
ylabel('y');

figure(21);
plot(tVal, [hs.y]);
xlabel('t');
ylabel('y')

figure(22);
plot(tVal, [hs.x]);
xlabel('t')
ylabel('x')

figure(23);
plot(tVal, [hs.w]);
xlabel('t')
ylabel('w [rad/s]')

figure(24);
plot(tVal, [hs.psi] * 180/pi, '.');
xlabel('t')
ylabel('psi [deg]')


figure(4);
plot(tVal, [hs.Q1] * 180/pi, [data.t], [data.Q1] / 6);
ylabel('Q1 [dps]');
legend('model', 'true');
%xlim([25 30]);


Tx_glob = hx(:, [6, 11, 16, 21]);
Ty_glob = hx(:,[7 12 17 22]);
psi = [hs.psi]';
Tx1 = Tx_glob(:, 1) .* cos(psi) + Ty_glob(:, 1) .* sin(psi);
Ty1 = Ty_glob(:, 1) .* cos(psi) - Tx_glob(:, 1) .* sin(psi);


figure(5);
plot(tVal, Ty1);
ylabel('Ty1');
%xlim([25 30]);

figure(6);
plot([data.t], [data.M1, data.M2, data.M3, data.M4]);
xlabel('t')
ylabel('M1')
%xlim([25 30]);

figure(7);
plot(tVal, [hs.w], [data.t], [data.wz] * pi/180);
ylabel('w [rad/s]');
%xlim([25 30]);
legend('model', 'true');

figure(8);
vx1_glob = hx(:, 30);
vy1_glob = hx(:, 31);
vx1 = vx1_glob .* cos(psi) + vy1_glob .* sin(psi);
vy1 = vy1_glob .* cos(psi) - vx1_glob .* sin(psi);
vob1 = -p.R1 * [hs.Q1]';
plot(tVal, [vx1, vy1, vob1]');
ylabel('v1 [m/s]');
legend('x', 'y', 'obvod');
%xlim([25 30]);

%% FUNKCIE

% riesenie systemu
function x = solve_system(s, p, M, Mc)

    % vypocet globalnej polohy taziska
    pos = get_r(s.x, s.y, s.psi, p.xT, p.yT, p.zT, 0, 0);
    rT = struct('x', pos(1), 'y', pos(2), 'z', pos(3));

    
    % vypocet globalnej polohy dotykoveho bodu
    pos = get_r(s.x, s.y, s.psi, p.x1, p.y1, 0, p.e1, s.a1);
    r1 = struct('x', pos(1), 'y', pos(2), 'z', pos(3));
    
    pos = get_r(s.x, s.y, s.psi, p.x2, p.y2, 0, p.e2, s.a2);
    r2 = struct('x', pos(1), 'y', pos(2), 'z', pos(3));
    
    pos = get_r(s.x, s.y, s.psi, p.x3, p.y3, 0, p.e3, s.a3);
    r3 = struct('x', pos(1), 'y', pos(2), 'z', pos(3));
    
    pos = get_r(s.x, s.y, s.psi, p.x4, p.y4, 0, p.e4, s.a4);    
    r4 = struct('x', pos(1), 'y', pos(2), 'z', pos(3));

    
    % vypocet derivovanej globalnej polohy dotykoveho bodu (pomocna premenna)
    pos = get_dr(s.psi, p.x1, p.y1, p.e1, s.a1, s.da1, s.w);
    dr1 = struct('x', pos(1), 'y', pos(2), 'z', pos(3));
    
    pos = get_dr(s.psi, p.x2, p.y2, p.e2, s.a2, s.da2, s.w);
    dr2 = struct('x', pos(1), 'y', pos(2), 'z', pos(3));

    pos = get_dr(s.psi, p.x3, p.y3, p.e3, s.a3, s.da3, s.w);
    dr3 = struct('x', pos(1), 'y', pos(2), 'z', pos(3));
    
    pos = get_dr(s.psi, p.x4, p.y4, p.e4, s.a4, s.da4, s.w);
    dr4 = struct('x', pos(1), 'y', pos(2), 'z', pos(3));


    % vypocet rychlosti taziska
    v = [s.vx - s.w * ((p.xT * sin(s.psi)) + (p.yT * cos(s.psi)));
         s.vy - s.w * ((p.yT * sin(s.psi)) - (p.xT * cos(s.psi)))];
    vT = struct('x', v(1), 'y', v(2), 'z', 0);
    

    % vypocet rychlosti dotykovych bodov
    v = [s.vx - (s.w * (r1.y - s.y)) - (p.R1 * s.Q1 * cos(s.a1));
         s.vy + (s.w * (r1.x - s.x)) - (p.R1 * s.Q1 * sin(s.a1))];    
    v1 = struct('x', v(1), 'y', v(2), 'z', 0);
    pv1 = sqrt(v1.x^2 + v1.y^2);


    v = [s.vx - (s.w * (r2.y - s.y)) - (p.R2 * s.Q2 * cos(s.a2));
         s.vy + (s.w * (r2.x - s.x)) - (p.R2 * s.Q2 * sin(s.a2))];    
    v2 = struct('x', v(1), 'y', v(2), 'z', 0);
    pv2 = sqrt(v2.x^2 + v2.y^2);

    % ZMENA s.a3
    v = [s.vx - (s.w * (r3.y - s.y)) - (p.R3 * s.Q3 * cos(s.a3));
         s.vy + (s.w * (r3.x - s.x)) - (p.R3 * s.Q3 * sin(s.a3))];    
    v3 = struct('x', v(1), 'y', v(2), 'z', 0);
    pv3 = sqrt(v3.x^2 + v3.y^2);

    v = [s.vx - (s.w * (r4.y - s.y)) - (p.R4 * s.Q4 * cos(s.a4));
         s.vy + (s.w * (r4.x - s.x)) - (p.R4 * s.Q4 * sin(s.a4))];  
    v4 = struct('x', v(1), 'y', v(2), 'z', 0);
    pv4 = sqrt(v4.x^2 + v4.y^2);


    % detekcia smyku na zaklade rychlosti (ak je vacsia ako nula, tak koleso smyka)
    sk = struct( ...
        'w1', (pv1 > 1e-4), ...
        'w2', (pv2 > 1e-4), ...                                     
        'w3', (pv3 > 1e-4), ...
        'w4', (pv4 > 1e-4));

    calc = 1;

    while calc

        if sk.w1
            if pv1 < 1e-4
                v1xn = 1;
                v1yn = 0;
            else
                v1xn = v1.x / pv1;
                v1yn = v1.y / pv1;
            end
        else
            v1xn = 0;
            v1yn = 0;
        end

        if sk.w2
            if pv2 < 1e-4
                v2xn = 1;
                v2yn = 0;
            else
                v2xn = v2.x / pv2;
                v2yn = v2.y / pv2;
            end
        else
            v2xn = 0;
            v2yn = 0;
        end

        if sk.w3
            if pv3 < 1e-4
                v3xn = 1;
                v3yn = 0;
            else
                v3xn = v3.x / pv3;
                v3yn = v3.y / pv3;
            end
        else
            v3xn = 0;
            v3yn = 0;
        end

        if sk.w4
            if pv4 < 1e-4
                v4xn = 1;
                v4yn = 0;
            else
                v4xn = v4.x / pv4;
                v4yn = v4.y / pv4;
            end
        else
            v4xn = 0;
            v4yn = 0;
        end


        isk = struct( ...
        'w1', (1 - sk.w1), ...
        'w2', (1 - sk.w2), ...                                     
        'w3', (1 - sk.w3), ...
        'w4', (1 - sk.w4));

        
        %           w'                                                          vx'             vy'             Nx              Ny              Tx1                     Ty1                     N1                      Q1'                             a1''            Tx2                     Ty2                     N2                      Q2'                             a2''            Tx3                     Ty3                     N3                      Q3'                             a3''            Tx4                     Ty4                     N4                      Q4'                             a4''            Tyx             Tyy  
        A = [       - p.mr * ((p.xT * sin(s.psi)) + (p.yT * cos(s.psi)))        p.mr            0               0               0               -1                      0                       0                       0                               0               -1                      0                       0                       0                               0               -1                      0                       0                       0                               0               -1                      0                       0                       0                               0;
                    - p.mr * ((p.yT * sin(s.psi)) - (p.xT * cos(s.psi)))        0               p.mr            0               0               0                       -1                      0                       0                               0               0                       -1                      0                       0                               0               0                       -1                      0                       0                               0               0                       -1                      0                       0                               0;
                    0                                                           0               0               -1              -1              0                       0                       1                       0                               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0;
                    0                                                           0               0               -1              1               0                       0                       0                       0                               0               0                       0                       1                       0                               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0;
                    0                                                           0               0               1               1               0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       0                       1                       0                               0               0                       0                       0                       0                               0;
                    0                                                           0               0               1               -1              0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       0                       1                       0                               0;
                    0                                                           0               0               0               0               0                       0                       1                       0                               0               0                       0                       1                       0                               0               0                       0                       1                       0                               0               0                       0                       1                       0                               0;      
                    0                                                           0               0               0               0               0                       -(r1.z - rT.z)          (r1.y - rT.y)           0                               0               0                       -(r2.z - rT.z)          (r2.y - rT.y)           0                               0               0                       -(r3.z - rT.z)          (r3.y - rT.y)           0                               0               0                       -(r4.z - rT.z)          (r4.y - rT.y)           0                               0;
                    0                                                           0               0               0               0               (r1.z - rT.z)           0                       -(r1.x - rT.x)          0                               0               (r2.z - rT.z)           0                       -(r2.x - rT.x)          0                               0               (r3.z - rT.z)           0                       -(r3.x - rT.x)          0                               0               (r4.z - rT.z)           0                       -(r4.x - rT.x)          0                               0;
                    (p.Jr)                                                      0               0               0               0               (r1.y - rT.y)           -(r1.x - rT.x)          0                       0                               0               (r2.y - rT.y)           -(r2.x - rT.x)          0                       0                               0               (r3.y - rT.y)           -(r3.x - rT.x)          0                       0                               0               (r4.y - rT.y)           -(r4.x - rT.x)          0                       0                               0;
                    0                                                           0               0               0               0               (p.R1 * cos(s.a1))      (p.R1 * sin(s.a1))      0                       p.Jw1                           0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0;
                    0                                                           0               0               0               0               -(p.e1 * sin(s.a1))     (p.e1 * cos(s.a1))      0                       0                               p.Jc1           0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0;
                    0                                                           0               0               0               0               0                       0                       0                       0                               0               (p.R2 * cos(s.a2))      (p.R2 * sin(s.a2))      0                       p.Jw2                           0               0                       0                       0                       0                               0               0                       0                       0                       0                               0;
                    0                                                           0               0               0               0               0                       0                       0                       0                               0               -(p.e2 * sin(s.a2))     (p.e2 * cos(s.a2))      0                       0                               p.Jc2           0                       0                       0                       0                               0               0                       0                       0                       0                               0;
                    0                                                           0               0               0               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               (p.R3 * cos(s.a3))      (p.R3 * sin(s.a3))      0                       p.Jw3                           0               0                       0                       0                       0                               0;
                    0                                                           0               0               0               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               -(p.e3 * sin(s.a3))     (p.e3 * cos(s.a3))      0                       0                               p.Jc3           0                       0                       0                       0                               0;
                    0                                                           0               0               0               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               (p.R4 * cos(s.a4))      (p.R4 * sin(s.a4))      0                       p.Jw4                           0;
                    0                                                           0               0               0               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               -(p.e4 * sin(s.a4))     (p.e4 * cos(s.a4))      0                       0                               p.Jc4;
                    0                                                           0               0               0               0               sk.w1                   0                       (sk.w1 * p.u1 * v1xn)   0                               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0;
                    0                                                           0               0               0               0               0                       sk.w1                   (sk.w1 * p.u1 * v1yn)   0                               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0;
                    0                                                           0               0               0               0               0                       0                       0                       0                               0               sk.w2                   0                       (sk.w2 * p.u2 * v2xn)   0                               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0;
                    0                                                           0               0               0               0               0                       0                       0                       0                               0               0                       sk.w2                   (sk.w2 * p.u2 * v2yn)   0                               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0;
                    0                                                           0               0               0               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               sk.w3                   0                       (sk.w3 * p.u3 * v3xn)   0                               0               0                       0                       0                       0                               0;
                    0                                                           0               0               0               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       sk.w3                   (sk.w3 * p.u3 * v3yn)   0                               0               0                       0                       0                       0                               0;
                    0                                                           0               0               0               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               sk.w4                   0                       (sk.w4 * p.u4 * v4xn)   0                               0;
                    0                                                           0               0               0               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       sk.w4                   (sk.w4 * p.u4 * v4yn)   0                               0;
                    -isk.w1 * (r1.y - s.y)                                      isk.w1          0               0               0               0                       0                       0                       -(isk.w1 * p.R1 * cos(s.a1))    0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0;
                    +isk.w1 * (r1.x - s.x)                                      0               isk.w1          0               0               0                       0                       0                       -(isk.w1 * p.R1 * sin(s.a1))    0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0;
                    -isk.w2 * (r2.y - s.y)                                      isk.w2          0               0               0               0                       0                       0                       0                               0               0                       0                       0                       -(isk.w2 * p.R2 * cos(s.a2))    0               0                       0                       0                       0                               0               0                       0                       0                       0                               0;
                    +isk.w2 * (r2.x - s.x)                                      0               isk.w2          0               0               0                       0                       0                       0                               0               0                       0                       0                       -(isk.w2 * p.R2 * sin(s.a2))    0               0                       0                       0                       0                               0               0                       0                       0                       0                               0;
                    -isk.w3 * (r3.y - s.y)                                      isk.w3          0               0               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       0                       0                       -(isk.w3 * p.R3 * cos(s.a3))    0               0                       0                       0                       0                               0;
                    +isk.w3 * (r3.x - s.x)                                      0               isk.w3          0               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       0                       0                       -(isk.w3 * p.R3 * sin(s.a3))    0               0                       0                       0                       0                               0;
                    -isk.w4 * (r4.y - s.y)                                      isk.w4          0               0               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       0                       0                       -(isk.w4 * p.R4 * cos(s.a4))    0;
                    +isk.w4 * (r4.x - s.x)                                      0               isk.w4          0               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       0                       0                       -(isk.w4 * p.R4 * sin(s.a4))    0;
        ];
    
    % oprava riadok 430 isk.w1
    
        b = [       p.mr * (s.w^2) * ((p.xT * cos(s.psi)) - (p.yT * sin(s.psi)));
                    p.mr * (s.w^2) * ((p.yT * cos(s.psi)) + (p.xT * sin(s.psi)));
                    (p.mr * p.g) / 4;
                    (p.mr * p.g) / 4;
                    (p.mr * p.g) / 4;
                    (p.mr * p.g) / 4;
                    (p.mr * p.g);
                    0;
                    0;
                    0;
                    M(1);
                    Mc(1);
                    M(2);
                    Mc(2);
                    M(3);
                    Mc(3);
                    M(4);
                    Mc(4);
                    0;
                    0;
                    0;
                    0;
                    0;
                    0;
                    0;
                    0;
                    ((s.w * dr1.y) - (p.R1 * s.Q1 * s.da1 * sin(s.a1))) * isk.w1;
                    (-(s.w * dr1.x) + (p.R1 * s.Q1 * s.da1 * cos(s.a1))) * isk.w1;
                    ((s.w * dr2.y) - (p.R2 * s.Q2 * s.da2 * sin(s.a2))) * isk.w2;
                    (-(s.w * dr2.x) + (p.R2 * s.Q2 * s.da2 * cos(s.a2))) * isk.w2;
                    ((s.w * dr3.y) - (p.R3 * s.Q3 * s.da3 * sin(s.a3))) * isk.w3;
                    (-(s.w * dr3.x) + (p.R3 * s.Q3 * s.da3 * cos(s.a3))) * isk.w3;
                    ((s.w * dr4.y) - (p.R4 * s.Q4 * s.da4 * sin(s.a4))) * isk.w4;
                    (-(s.w * dr4.x) + (p.R4 * s.Q4 * s.da4 * cos(s.a4))) * isk.w4;
                    
        ];

        % riesenie systemu
        x = A\b;
        x(26:29) = [sk.w1; sk.w2; sk.w3; sk.w4];
        x(30:37) = [v1.x; v1.y; v2.x; v2.y; v3.x; v3.y; v4.x; v4.y];
        calc = 0;

        % detekcia smyku kolies pomocou Amontonovho - Coulombovho zakona
        Tmax = 1.2 * x(8) * p.u1;
        T = sqrt(x(6)^2 + x(7)^2);
        if T > Tmax                    % ak je trecia sila vacsia, ako normalova sila, tak koleso zacne smykat (teda tato sila je menej ucinna)
            sk.w1 = 1;
            calc = 1;
        end
    
        Tmax = 1.2 * x(13) * p.u2;
        T = sqrt(x(11)^2 + x(12)^2);
        if T > Tmax                     % ak je trecia sila vacsia, ako normalova sila, tak koleso zacne smykat (teda tato sila je menej ucinna)
            sk.w2 = 1;
            calc = 1;
        end

        Tmax = 1.2 * x(18) * p.u3;
        T = sqrt(x(16)^2 + x(17)^2);
        if T > Tmax                  % ak je trecia sila vacsia, ako normalova sila, tak koleso zacne smykat (teda tato sila je menej ucinna)
            sk.w3 = 1;
            calc = 1;
        end

        Tmax = 1.2 * x(23) * p.u4;
        T = sqrt(x(21)^2 + x(22)^2);
        if T > Tmax                    % ak je trecia sila vacsia, ako normalova sila, tak koleso zacne smykat (teda tato sila je menej ucinna)
            sk.w4 = 1;
            calc = 1;
        end

    end

end

function r = get_r(x0, y0, psi, xL, yL, zL, e, a)

    n = [cos(a); sin(a); 0];                                        % uhol natocenia v globalnych suradniciach
    
    Rz = [cos(psi), -sin(psi),  0;
          sin(psi),  cos(psi),  0;
          0,         0,         1];                                 % rotacna matica :: lokalne suradnice do globalnych
    
    r0 = [x0; y0; 0];                                               % poloha stredu kinematiky
    rc = [xL; yL; zL];                                              % poloha dotykoveho bodu v lokalnych suradniciach

    r = r0 + (Rz*rc) - (e*n);                                       % globalna poloha dotykoveho bodu

end

function dr = get_dr(psi, xL, yL, e, a, da, w)

    n = [-sin(a)*da; cos(a)*da; 0];                                 % uhol natocenia v globalnych suradniciach (derivovana)
    
    Rz = [-sin(psi)*w, -cos(psi)*w,  0;
          cos(psi)*w,  -sin(psi)*w,  0;
          0,            0,           0];                            % rotacna matica :: lokalne suradnice do globalnych (derivovana)
    
    rc = [xL; yL; 0];                                               % poloha dotykoveho bodu v lokalnych suradniciach

    dr = Rz*rc - (e*n);                                             % derivacia globalnej polohy dotykoveho bodu

end