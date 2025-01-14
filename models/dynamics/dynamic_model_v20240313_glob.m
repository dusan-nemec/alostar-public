
clear;
close;
clc;


%% Get data logs

fName = 'log90-3.txt';                                                                      % file name
fID = fopen(fName, "r");                                                                    % file ID :: open file to read
lID = 0;                                                                                    % line ID :: init value

if (fID < 0)
    error("error opening file %s\n\n", fName);
end                                                                                         % detect error during opening file

array = zeros(1, 21);

while ~feof(fID)
    
    nline = fgetl(fID);                                                     % get line data from text message
    tmp = strsplit(nline);                                                  % devide data in the line based on tabulator

    if (lID == 0)
        lID = lID + 1;                                                      % increment line value
        colName = "" + tmp;                                                 % create string name
        continue;                                                           % go to the next cycle
    end

    if (lID == 1)
        arrBias = str2double(tmp);                                        % get current time stamp
    end

    array(lID, :) = str2double(tmp);
    array(lID, 1) = array(lID, 1) - arrBias(1);
    array(lID, 16:21) = array(lID, 16:21) - arrBias(16:21);
    lID = lID + 1;                                                          % increment line value

end

data = array2table(array);                                                                  % store logged data
data.Properties.VariableNames = colName;                                                    % variables name

fclose(fID);                                                                                % close file

data.dwz = [0; diff([data.wz])] ./ [1; diff([data.t])];
data.dQ1 = [0; diff([data.Q1])] ./ [1; diff([data.t])];
data.dQ2 = [0; diff([data.Q2])] ./ [1; diff([data.t])];
data.dQ3 = [0; diff([data.Q3])] ./ [1; diff([data.t])];
data.dQ4 = [0; diff([data.Q4])] ./ [1; diff([data.t])];

%% Structures / vectors definition

Mt = 0.0 * [1.0, 1.0, 1.0, 1.0];                                                            % traction motors torques
Ms = 0.0 * [1.0, 1.0, 1.0, 1.0];                                                            % steering motors torques
f = 0.75;
f1 = f;
f2 = f;
f3 = f;
f4 = f;

% parameters structure
p = struct(...
    ...
    'g', 9.80665, ...                                                                       % acceleration due to gravity [m/s^2]
    'mr', 95, ...                                                                           % mobile robot's weight [kg]
    'Jr', 5.375, ...                                                                        % mobile robot's moment of inertia
    'xT', 0*0.028, ...                                                                        % position of the centre of gravity from the kinematic centre :: x-component [m]
    'yT', 0.0, ...                                                                          % position of the centre of gravity from the kinematic centre :: y-component [m]
    'zT', 0.2, ...                                                                          % position of the centre of gravity from the kinematic centre :: z-component [m]
    ...
    'R1', 0.1625, ...                                                                       % wheel radius [m]
    'x1', 0.2975, ...                                                                       % position of the wheel centre from the kinematic centre :: x-component [m]
    'y1', 0.2850, ...                                                                       % position of the wheel centre from the kinematic centre :: y-component [m]
    'e1', 0.0, ...                                                                          % excentricity of the wheel [m]
    'Jc1', 1.0, ...                                                                         % castor's moment of inertia
    'Jw1', 0.05, ...                                                                        % wheel's moment of inertia
    'f1', f1, ...                                                                          % shear friction coefficient
    ...
    'R2', 0.1625, ...                                                                       % wheel radius [m]
    'x2', 0.2975, ...                                                                       % position of the wheel centre from the kinematic centre :: x-component [m]
    'y2', -0.2850, ...                                                                      % position of the wheel centre from the kinematic centre :: y-component [m]
    'e2', 0.0, ...                                                                          % excentricity of the wheel [m]
    'Jc2', 1.0, ...                                                                         % castor's moment of inertia
    'Jw2', 0.05, ...                                                                        % wheel's moment of inertia
    'f2', f2, ...                                                                          % shear friction coefficient
    ...
    'R3', 0.1625, ...                                                                       % wheel radius [m]
    'x3', -0.2975, ...                                                                      % position of the wheel centre from the kinematic centre :: x-component [m]
    'y3', -0.2850, ...                                                                      % position of the wheel centre from the kinematic centre :: y-component [m]
    'e3', 0.0, ...                                                                          % excentricity of the wheel [m]
    'Jc3', 1.0, ...                                                                         % castor's moment of inertia
    'Jw3', 0.05, ...                                                                        % wheel's moment of inertia
    'f3', f3, ...                                                                          % shear friction coefficient
    ...
    'R4', 0.1625, ...                                                                       % wheel radius [m]
    'x4', -0.2975, ...                                                                      % position of the wheel centre from the kinematic centre :: x-component [m]
    'y4', 0.2850, ...                                                                       % position of the wheel centre from the kinematic centre :: y-component [m]
    'e4', 0.0, ...                                                                          % excentricity of the wheel [m]
    'Jc4', 1.0, ...                                                                         % castor's moment of inertia
    'Jw4', 0.05, ...                                                                        % wheel's moment of inertia
    'f4', f4 ...                                                                           % shear friction coefficient
);

% data log structure
d = struct(...
    ...
    'x', 0.0, ...                                                                           % local coordinates [m]
    'y', 0.0, ...                                                                           % local coordinates [m]
    'psi', 0.0, ...                                                                         % local coordinates [rad]
    ...
    'dw', 0.0, ...                                                                          % angular acceleration [rad/s^2]
    'dw0', 0.0, ...                                                                         % estimated angular acceleration 
    'dvx', 0.0, ...                                                                         % acceleration :: x-component [m/s^2]
    'dvy', 0.0, ...                                                                         % acceleration :: y-component [m/s^2]
    ...
    'w', 0.0, ...                                                                           % angular speed [rad/s]         (source :: data log)
    'vx', 0.0, ...                                                                          % translational speed [mm/s]    (source :: dynamics model)
    'vy', 0.0, ...                                                                          % translational speed [mm/s]    (source :: dynamics model)
    ...
    'Q1', 0.0, ...                                                                          % wheel's angular speed [rad/s]
    'dQ1', 0.0, ...                                                                         % wheel's angular acceleration [rad/s^2]
    'a1', 0.0, ...                                                                          % steering wheel's orientation [rad]
    'da1', 0.0, ...                                                                         % steering wheel's speed [rad/s]
    ...
    'Q2', 0.0, ...                                                                          % wheel's angular speed [rad/s]
    'dQ2', 0.0, ...                                                                         % wheel's angular acceleration [rad/s^2]
    'a2', 0.0, ...                                                                          % steering wheel's orientation [rad]
    'da2', 0.0, ...                                                                         % steering wheel's speed [rad/s]
    ...
    'Q3', 0.0, ...                                                                          % wheel's angular speed [rad/s]
    'dQ3', 0.0, ...                                                                         % wheel's angular acceleration [rad/s^2]
    'a3', 0.0, ...                                                                          % steering wheel's orientation [rad]
    'da3', 0.0, ...                                                                         % steering wheel's speed [rad/s]
    ...
    'Q4', 0.0, ...                                                                          % wheel's angular speed [rad/s]
    'dQ4', 0.0, ...                                                                         % wheel's angular acceleration [rad/s^2]
    'a4', 0.0, ...                                                                          % steering wheel's orientation [rad]
    'da4', 0.0 ...                                                                          % steering wheel's speed [rad/s]
);

% state structure
s = struct(...
    ...
    'x', 0.0, ...                                                                           % mobile robot's global coordinates :: X [m]
    'y', 0.0, ...                                                                           % mobile robot's global coordinates :: Y [m]
    'psi', 0.0, ...                                                                         % mobile robot's global orientation :: PSI [rad]
    ...
    'vx', 0.0, ...                                                                          % [m/s]
    'vy', 0.0, ...                                                                          % [m/s]
    'w', 0.0, ...                                                                           % [rad/s]
    ...
    'Q1', 0.0, ...                                                                          % [rad/s]
    'dQ1', 0.0, ...                                                                         % [rad/s^2]
    'a1', 0.0, ...                                                                          % steering wheel's orientation [rad]
    'da1', 0.0, ...                                                                         % steering wheel's angular speed [rad/s]
    ...
    'Q2', 0.0, ...                                                                          % [rad/s]
    'dQ2', 0.0, ...                                                                         % [rad/s^2]
    'a2', 0.0, ...                                                                          % steering wheel's orientation [rad]
    'da2', 0.0, ...                                                                         % steering wheel's speed [rad/s]
    ...
    'Q3', 0.0, ...                                                                          % [rad/s]
    'dQ3', 0.0, ...                                                                         % [rad/s^2]
    'a3', 0.0, ...                                                                          % steering wheel's orientation [rad]
    'da3', 0.0, ...                                                                         % steering wheel's speed [rad/s]
    ...
    'Q4', 0.0, ...                                                                          % [rad/s]
    'dQ4', 0.0, ...                                                                         % [rad/s^2]
    'a4', 0.0, ...                                                                          % steering wheel's orientation [rad]
    'da4', 0.0 ...                                                                          % steering wheel's speed [rad/s]
);

%% Estimation + simulation

i = 1;
dt = 0.0001;
prescale = 10;
N = int32(data.t(end) / dt) + 1;
Nh = int32(N/prescale);
hs = repmat(s, Nh, 1);
hM = zeros(Nh, 5); % + time
hE = zeros(Nh, 28);
hx = [];
kh = 1;

Kp = 0.02;
Ki = 0.1;
Kd = 0.0;
Tf = 0.01;

ePID = zeros(1, 4);
iPID = zeros(1, 4);
dPID = zeros(1, 4);
Mmax = 30.0;

for k = 1:N
    
    warning('off', 'MATLAB:rankDeficientMatrix');
    
    t = double(k-1) * dt;

    if (i < numel(data.t) && t >= data.t(i))
%         
%         Mscale = 57.0;
%         Mt(1) = data.M1(i) / Mscale;
%         Mt(2) = data.M3(i) / Mscale;
%         Mt(3) = data.M4(i) / Mscale;
%         Mt(4) = data.M2(i) / Mscale;
%         
%         d.w = data.wz(i) * pi / 180;
%         d.dw = data.dwz(i) * pi / 180;
%         
%         d.Q1 = (data.Q1(i) * pi) / 180;
%         d.Q2 = (data.Q3(i) * pi) / 180;
%         d.Q3 = (data.Q4(i) * pi) / 180;
%         d.Q4 = (data.Q2(i) * pi) / 180;
%         
%         d.dQ1 = (data.dQ1(i) * pi) / 180;
%         d.dQ2 = (data.dQ3(i) * pi) / 180;
%         d.dQ3 = (data.dQ4(i) * pi) / 180;
%         d.dQ4 = (data.dQ2(i) * pi) / 180;

        i = i + 1;
    
    end
    
    if mod(k, 10) == 1
        % control wheels to meet the data
        diff_t = t - data.t(i);
        Qref = [    data.Q1(i) + data.dQ1(i) * diff_t;
                    data.Q3(i) + data.dQ3(i) * diff_t;
                    data.Q4(i) + data.dQ4(i) * diff_t;
                    data.Q2(i) + data.dQ2(i) * diff_t ] * pi / 180;

%         Qref = [1; 1; 1; 1];

        Q = [s.Q1; s.Q2; s.Q3; s.Q4];

        for ax = 1:4
            eQ = Qref(ax) - Q(ax);
            if Ki > 0
                iQ = iPID(ax) + eQ * dt;
                iQ = min(max(iQ, -1.0/Ki), 1.0/Ki);
            else
                iQ = 0;
            end

            if Kd > 0
                dQ = (dPID(ax) + eQ - ePID(ax)) / (Tf + dt);
            else
                dQ = 0;
            end
            
            % avoid stall condition
            if abs(eQ) < 0.1
                iQ = iQ * 0.95;
            end

            Mt(ax) = Mmax * min(max(Kp * eQ + Ki * iQ + Kd * dQ, -1.0), 1.0);

            ePID(ax) = eQ;
            iPID(ax) = iQ;
            dPID(ax) = dQ;
        end
        
        % avoid inter-axle tension
        if(Mt(1) * Mt(4) < -1.0)
            Mt(1) = Mt(1) + Mt(4);
            Mt(4) = 0;
        end
        
        if(Mt(2) * Mt(3) < -1.0)
            Mt(2) = Mt(2) + Mt(3);
            Mt(3) = 0;
        end
    end

    [x, p_new] = solve_system(s, p, Mt, Ms);
    x_prev = x;
    %p = p_new;

    s.w = s.w + x.dw*dt;
    s.vx = s.vx + x.dvx*dt;
    s.vy = s.vy + x.dvy*dt;

    s.Q1 = s.Q1 + x.dQ1*dt;
    s.Q2 = s.Q2 + x.dQ2*dt;
    s.Q3 = s.Q3 + x.dQ3*dt;
    s.Q4 = s.Q4 + x.dQ4*dt;

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

    if prescale == 1 || mod(k, prescale) == 1
        hs(kh) = s;
        hM(kh,1) = Mt(1);
        hM(kh,2) = Mt(2);
        hM(kh,3) = Mt(3);
        hM(kh,4) = Mt(4);
        hM(kh,5) = t;

        if isempty(hx)
            hx = repmat(x, Nh, 1);
        end

        hx(kh) = x;
        kh = kh + 1;
    end
end


%% Plots
tVal = (1:numel(hs))' * dt * prescale;

figure(1);
subplot(4, 1, 1);
plot(tVal, [hs.Q1], [data.t], [data.Q1]*pi/180);
xlabel('time [s]');
ylabel('Q1 [rad/s]');

subplot(4, 1, 2);
plot(tVal, [hs.Q2], [data.t], [data.Q3]*pi/180);
xlabel('time [s]');
ylabel('Q2 [rad/s]');

subplot(4, 1, 3);
plot(tVal, [hs.Q3], [data.t], [data.Q4]*pi/180);
xlabel('time [s]');
ylabel('Q3 [rad/s]');

subplot(4, 1, 4);
plot(tVal, [hs.Q4], [data.t], [data.Q2]*pi/180);
xlabel('time [s]');
ylabel('Q4 [rad/s]');

hxt = struct2table(hx);
hxt.T1 = sqrt([hxt.Tx1].^2 + [hxt.Ty1].^2);
hxt.T2 = sqrt([hxt.Tx2].^2 + [hxt.Ty2].^2);
hxt.T3 = sqrt([hxt.Tx3].^2 + [hxt.Ty3].^2);
hxt.T4 = sqrt([hxt.Tx4].^2 + [hxt.Ty4].^2);
legend('sim', 'meas');

figure(2);
subplot(4, 1, 1);
plot(tVal, hM(:,1), [data.t], [data.M1]/57.0);
xlabel('time [s]');
ylabel('M1 [N.m]');

subplot(4, 1, 2);
plot(tVal,  hM(:,2), [data.t], [data.M3]/57.0);
xlabel('time [s]');
ylabel('M2 [N.m]');

subplot(4, 1, 3);
plot(tVal,  hM(:,3), [data.t], [data.M4]/57.0);
xlabel('time [s]');
ylabel('M3 [N.m]');

subplot(4, 1, 4);
plot(tVal,  hM(:,4), [data.t], [data.M2]/57.0);
legend('sim', 'meas');
xlabel('time [s]');
ylabel('M4 [N.m]');

figure(3);
plot(tVal, [hs.w], [data.t], [data.wz] * pi/180);
legend('sim', 'meas');


%% Functions

function e = system_estimation(p, d, Mt, Ms)
     
    % calculate centre of gravity coodrinates
    rT = loc2glob(d, p.xT, p.yT, p.zT, 0, 0);
                             
    % calculate wheel's touch point coodrinates
    r1 = loc2glob(d, p.x1, p.y1, 0, p.e1, d.a1);
    r2 = loc2glob(d, p.x2, p.y2, 0, p.e2, d.a2);
    r3 = loc2glob(d, p.x3, p.y3, 0, p.e3, d.a3);
    r4 = loc2glob(d, p.x4, p.y4, 0, p.e4, d.a4);    
     
    % calculate wheel's touch point velocity
    v = [d.vx - (d.w * (r1.y - d.y)) - (p.R1 * d.Q1 * cos(d.a1));
         d.vy + (d.w * (r1.x - d.x)) - (p.R1 * d.Q1 * sin(d.a1))];    
    v1 = struct('x', v(1), 'y', v(2), 'z', 0);                                              % wheel's touch point velocity vector
    pv1 = sqrt(v1.x^2 + v1.y^2);                                                            % wheel's touch point velocity

    v = [d.vx - (d.w * (r2.y - d.y)) - (p.R2 * d.Q2 * cos(d.a2));
         d.vy + (d.w * (r2.x - d.x)) - (p.R2 * d.Q2 * sin(d.a2))];    
    v2 = struct('x', v(1), 'y', v(2), 'z', 0);                                              % wheel's touch point velocity vector
    pv2 = sqrt(v2.x^2 + v2.y^2);                                                            % wheel's touch point velocity

    v = [d.vx - (d.w * (r3.y - d.y)) - (p.R3 * d.Q3 * cos(d.a3));
         d.vy + (d.w * (r3.x - d.x)) - (p.R3 * d.Q3 * sin(d.a3))];    
    v3 = struct('x', v(1), 'y', v(2), 'z', 0);                                              % wheel's touch point velocity vector
    pv3 = sqrt(v3.x^2 + v3.y^2);                                                            % wheel's touch point velocity
    
    v = [d.vx - (d.w * (r4.y - d.y)) - (p.R4 * d.Q4 * cos(d.a4));
         d.vy + (d.w * (r4.x - d.x)) - (p.R4 * d.Q4 * sin(d.a4))];  
    v4 = struct('x', v(1), 'y', v(2), 'z', 0);                                              % wheel's touch point velocity vector
    pv4 = sqrt(v4.x^2 + v4.y^2);                                                            % wheel's touch point velocity

    sk = struct( ...
        'w1', (pv1 > 0.1), ...
        'w2', (pv2 > 0.1), ...                                     
        'w3', (pv3 > 0.1), ...
        'w4', (pv4 > 0.1));                                                                % wheels skid detection :: based on current velocity (higher than zero >> wheel skid)


    if sk.w1
        v1xn = v1.x / pv1;
        v1yn = v1.y / pv1;
    else
        v1xn = 0;
        v1yn = 0;
    end

    if sk.w2
        v2xn = v2.x / pv2;
        v2yn = v2.y / pv2;
    else
        v2xn = 0;
        v2yn = 0;
    end

    if sk.w3
        v3xn = v3.x / pv3;
        v3yn = v3.y / pv3;
    else
        v3xn = 0;
        v3yn = 0;
    end

    if sk.w4
        v4xn = v4.x / pv4;
        v4yn = v4.y / pv4;
    else
        v4xn = 0;
        v4yn = 0;
    end



    %           Tx1                     Ty1                     N1                      uN1                     Tx2                     Ty2                     N2                      uN2                     Tx3                     Ty3                     N3                      uN3                     Tx4                     Ty4                     N4                      uN4                                 Nx                  Ny
    A = [       1                       0                       0                       0                       1                       0                       0                       0                       1                       0                       0                       0                       1                       0                       0                       0                                   0                   0;
                0                       1                       0                       0                       0                       1                       0                       0                       0                       1                       0                       0                       0                       1                       0                       0                                   0                   0;
                0                       0                       1                       0                       1                       0                       1                       0                       0                       0                       1                       0                       0                       0                       1                       0                                   0                   0;
                0                       -(r1.z - rT.z)          (r1.y - rT.y)           0                       0                       -(r2.z - rT.z)          (r2.y - rT.y)           0                       0                       -(r3.z - rT.z)          (r3.y - rT.y)           0                       0                       -(r4.z - rT.z)          (r4.y - rT.y)           0                                   0                   0;
                (r1.z - rT.z)           0                       -(r1.x - rT.x)          0                       (r2.z - rT.z)           0                       -(r2.x - rT.x)          0                       (r3.z - rT.z)           0                       -(r3.x - rT.x)          0                       (r4.z - rT.z)           0                       -(r4.x - rT.x)          0                                   0                   0;
                -(r1.y - rT.y)          (r1.x - rT.x)           0                       0                       -(r2.y - rT.y)          (r2.x - rT.x)           0                       0                       -(r3.y - rT.y)          (r3.x - rT.x)           0                       0                       -(r4.y - rT.y)          (r4.x - rT.x)           0                       0                                   0                   0;
                -(p.R1 * cos(d.a1))     -(p.R1 * sin(d.a1))     0                       0                       0                       0                       0                       0                       0                       0                       0                       0                       0                       0                       0                       0                                   0                   0;
                %(p.e1 * sin(d.a1))      -(p.e1 * cos(d.a1))     0                       0                       0                       0                       0                       0                       0                       0                       0                       0                       0                       0                       0                       0                                   0                   0;
                0                       0                       0                       0                       -(p.R2 * cos(d.a2))     -(p.R2 * sin(d.a2))     0                       0                       0                       0                       0                       0                       0                       0                       0                       0                                   0                   0;
                %0                       0                       0                       0                       (p.e2 * sin(d.a2))      -(p.e2 * cos(d.a2))     0                       0                       0                       0                       0                       0                       0                       0                       0                       0                                   0                   0;
                0                       0                       0                       0                       0                       0                       0                       0                       -(p.R3 * cos(d.a3))     -(p.R3 * sin(d.a3))     0                       0                       0                       0                       0                       0                                   0                   0;
                %0                       0                       0                       0                       0                       0                       0                       0                       (p.e3 * sin(d.a3))      -(p.e3 * cos(d.a3))     0                       0                       0                       0                       0                       0                                   0                   0;
                0                       0                       0                       0                       0                       0                       0                       0                       0                       0                       0                       0                       -(p.R4 * cos(d.a4))     -(p.R4 * sin(d.a4))     0                       0                                   0                   0;
                %0                       0                       0                       0                       0                       0                       0                       0                       0                       0                       0                       0                       (p.e4 * sin(d.a4))      -(p.e4 * cos(d.a4))     0                       0                                   0                   0;
                sk.w1                   0                       0                       (sk.w1 * v1xn)          0                       0                       0                       0                       0                       0                       0                       0                       0                       0                       0                       0                                   0                   0;
                0                       sk.w1                   0                       (sk.w1 * v1yn)          0                       0                       0                       0                       0                       0                       0                       0                       0                       0                       0                       0                                   0                   0;
                0                       0                       0                       0                       sk.w2                   0                       0                       (sk.w2 * v2xn)          0                       0                       0                       0                       0                       0                       0                       0                                   0                   0;
                0                       0                       0                       0                       0                       sk.w2                   0                       (sk.w2 * v2yn)          0                       0                       0                       0                       0                       0                       0                       0                                   0                   0;
                0                       0                       0                       0                       0                       0                       0                       0                       sk.w3                   0                       0                       (sk.w3 * v3xn)          0                       0                       0                       0                                   0                   0;
                0                       0                       0                       0                       0                       0                       0                       0                       0                       sk.w3                   0                       (sk.w3 * v3yn)          0                       0                       0                       0                                   0                   0;
                0                       0                       0                       0                       0                       0                       0                       0                       0                       0                       0                       0                       sk.w4                   0                       0                       (sk.w4 * v4xn)                      0                   0;
                0                       0                       0                       0                       0                       0                       0                       0                       0                       0                       0                       0                       0                       sk.w4                   0                       (sk.w4 * v4yn)                      0                   0;
                0                       0                       1                       0                       0                       0                       0                       0                       0                       0                       0                       0                       0                       0                       0                       0                                   -1                  -1;
                0                       0                       0                       0                       0                       0                       1                       0                       0                       0                       0                       0                       0                       0                       0                       0                                   -1                  1;
                0                       0                       0                       0                       0                       0                       0                       0                       0                       0                       1                       0                       0                       0                       0                       0                                   1                   1;
                0                       0                       0                       0                       0                       0                       0                       0                       0                       0                       0                       0                       0                       0                       1                       0                                   1                   -1;

    ];

    b = [       p.mr * (d.dvx - d.dw * ((p.xT * sin(d.psi)) + (p.yT * cos(d.psi))) - (d.w * d.w) * ((p.xT * cos(d.psi)) - (p.yT * sin(d.psi))));
                p.mr * (d.dvy - d.dw * ((p.yT * sin(d.psi)) - (p.xT * cos(d.psi))) - (d.w * d.w) * ((p.yT * cos(d.psi)) + (p.xT * sin(d.psi))));
                p.mr * p.g;
                0;
                0;
                p.Jr * d.dw;
                (p.Jw1 * d.dQ1) - Mt(1);
                %- Ms(1);
                (p.Jw2 * d.dQ2) - Mt(2);
                %- Ms(2);
                (p.Jw3 * d.dQ3) - Mt(3);
                %- Ms(3);
                (p.Jw4 * d.dQ4) - Mt(4);
                %- Ms(4);
                0;
                0;
                0;
                0;
                0;
                0;
                0;
                0;
                (p.mr * p.g) / 4;
                (p.mr * p.g) / 4;
                (p.mr * p.g) / 4;
                (p.mr * p.g) / 4;
    ];

    % solve system
    e = A\b;
    e(21) = sk.w1;
    e(22) = sk.w2;
    e(23) = sk.w3;
    e(24) = sk.w4;
    
%     if(sum(e(21:24)) > 0)
%         disp('skid');
%     end

end

function [x, p_new] = solve_system(s, p, Mt, Ms)

    % calculate centre of gravity coodrinates
    rT = loc2glob(s, p.xT, p.yT, p.zT, 0, 0);
    
    % calculate wheel's touch point coodrinates
    r1 = loc2glob(s, p.x1, p.y1, 0, p.e1, s.a1);
    r2 = loc2glob(s, p.x2, p.y2, 0, p.e2, s.a2);
    r3 = loc2glob(s, p.x3, p.y3, 0, p.e3, s.a3);
    r4 = loc2glob(s, p.x4, p.y4, 0, p.e4, s.a4);
    
 
    % calculate wheel's touch point velocity
    v = [s.vx - s.w * (r1.y - s.y) - p.R1 * s.Q1 * cos(s.a1);
         s.vy + s.w * (r1.x - s.x) - p.R1 * s.Q1 * sin(s.a1)];    
    v1 = struct('x', v(1), 'y', v(2), 'z', 0);                                              % wheel's touch point velocity vector
    pv1 = sqrt(v1.x^2 + v1.y^2);                                                            % wheel's touch point velocity

    v = [s.vx - s.w * (r2.y - s.y) - p.R2 * s.Q2 * cos(s.a2);
         s.vy + s.w * (r2.x - s.x) - p.R2 * s.Q2 * sin(s.a2)];    
    v2 = struct('x', v(1), 'y', v(2), 'z', 0);                                              % wheel's touch point velocity vector
    pv2 = sqrt(v2.x^2 + v2.y^2);                                                            % wheel's touch point velocity

    v = [s.vx - s.w * (r3.y - s.y) - p.R3 * s.Q3 * cos(s.a3);
         s.vy + s.w * (r3.x - s.x) - p.R3 * s.Q3 * sin(s.a3)];    
    v3 = struct('x', v(1), 'y', v(2), 'z', 0);                                              % wheel's touch point velocity vector
    pv3 = sqrt(v3.x^2 + v3.y^2);                                                            % wheel's touch point velocity
    
    v = [s.vx - s.w * (r4.y - s.y) - p.R4 * s.Q4 * cos(s.a4);
         s.vy + s.w * (r4.x - s.x) - p.R4 * s.Q4 * sin(s.a4)];  
    v4 = struct('x', v(1), 'y', v(2), 'z', 0);                                              % wheel's touch point velocity vector
    pv4 = sqrt(v4.x^2 + v4.y^2);                                                            % wheel's touch point velocity

    sk = struct( ...
        'w1', (pv1 > 1e-4), ...
        'w2', (pv2 > 1e-4), ...                                     
        'w3', (pv3 > 1e-4), ...
        'w4', (pv4 > 1e-4));                                                                % wheels skid detection :: based on current velocity (higher than zero >> wheel skid)

    calc = 1;                                                                               % 1: enable calculation

    while calc
        
        if sk.w1
            if pv1 < 1e-5
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
            if pv2 < 1e-5
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
            if pv3 < 1e-5
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
            if pv4 < 1e-5
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
    
      
        cols = {     'dw',                                                      'dvx',          'dvy',          'Nx',           'Ny',           'Tx1',                  'Ty1',                  'N1',                   'dQ1',                          'd2a1',         'Tx2',                  'Ty2',                  'N2',                   'dQ2',                          'd2a2',         'Tx3',                  'Ty3',                  'N3',                   'dQ3',                          'd2a3',         'Tx4',                  'Ty4',                  'N4',                   'dQ4',                          'd2a4' ...
                    's1', 's2', 's3', 's4', 'v1x', 'v1y', 'v2x', 'v2y', 'v3x', 'v3y', 'v4x', 'v4y'};  
        A = [       - p.mr * (p.xT * sin(s.psi) + p.yT * cos(s.psi))            p.mr            0               0               0               -1                      0                       0                       0                               0               -1                      0                       0                       0                               0               -1                      0                       0                       0                               0               -1                      0                       0                       0                               0;
                    - p.mr * (p.yT * sin(s.psi) - p.xT * cos(s.psi))            0               p.mr            0               0               0                       -1                      0                       0                               0               0                       -1                      0                       0                               0               0                       -1                      0                       0                               0               0                       -1                      0                       0                               0;
                    0                                                           0               0               -1              -1              0                       0                       1                       0                               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       0                       0                       0                               -7;
                    0                                                           0               0               -1              1               0                       0                       0                       0                               0               0                       0                       1                       0                               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0;
                    0                                                           0               0               1               1               0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       0                       1                       0                               0               0                       0                       0                       0                               0;
                    0                                                           0               0               1               -1              0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       0                       1                       0                               0;
                    0                                                           0               0               0               0               0                       0                       1                       0                               0               0                       0                       1                       0                               0               0                       0                       1                       0                               0               0                       0                       1                       0                               0;      
                    0                                                           0               0               0               0               0                       -(r1.z - rT.z)          (r1.y - rT.y)           0                               0               0                       -(r2.z - rT.z)          (r2.y - rT.y)           0                               0               0                       -(r3.z - rT.z)          (r3.y - rT.y)           0                               0               0                       -(r4.z - rT.z)          (r4.y - rT.y)           0                               0;
                    0                                                           0               0               0               0               (r1.z - rT.z)           0                       -(r1.x - rT.x)          0                               0               (r2.z - rT.z)           0                       -(r2.x - rT.x)          0                               0               (r3.z - rT.z)           0                       -(r3.x - rT.x)          0                               0               (r4.z - rT.z)           0                       -(r4.x - rT.x)          0                               0;
                    (p.Jr)                                                      0               0               0               0               (r1.y - rT.y)           -(r1.x - rT.x)          0                       0                               0               (r2.y - rT.y)           -(r2.x - rT.x)          0                       0                               0               (r3.y - rT.y)           -(r3.x - rT.x)          0                       0                               0               (r4.y - rT.y)           -(r4.x - rT.x)          0                       0                               0;
                    0                                                           0               0               0               0               (p.R1 * cos(s.a1))      (p.R1 * sin(s.a1))      0                       p.Jw1                           0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0;
                    %0                                                           0               0               0               0               -(p.e1 * sin(s.a1))     (p.e1 * cos(s.a1))      0                       0                               p.Jc1           0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0;
                    0                                                           0               0               0               0               0                       0                       0                       0                               0               (p.R2 * cos(s.a2))      (p.R2 * sin(s.a2))      0                       p.Jw2                           0               0                       0                       0                       0                               0               0                       0                       0                       0                               0;
                    %0                                                           0               0               0               0               0                       0                       0                       0                               0               -(p.e2 * sin(s.a2))     (p.e2 * cos(s.a2))      0                       0                               p.Jc2           0                       0                       0                       0                               0               0                       0                       0                       0                               0;
                    0                                                           0               0               0               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               (p.R3 * cos(s.a3))      (p.R3 * sin(s.a3))      0                       p.Jw3                           0               0                       0                       0                       0                               0;
                    %0                                                           0               0               0               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               -(p.e3 * sin(s.a3))     (p.e3 * cos(s.a3))      0                       0                               p.Jc3           0                       0                       0                       0                               0;
                    0                                                           0               0               0               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               (p.R4 * cos(s.a4))      (p.R4 * sin(s.a4))      0                       p.Jw4                           0;
                    %0                                                           0               0               0               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               -(p.e4 * sin(s.a4))     (p.e4 * cos(s.a4))      0                       0                               p.Jc4;
                    0                                                           0               0               0               0               sk.w1                   0                       (sk.w1 * p.f1 * v1xn)   0                               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0;
                    0                                                           0               0               0               0               0                       sk.w1                   (sk.w1 * p.f1 * v1yn)   0                               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0;
                    0                                                           0               0               0               0               0                       0                       0                       0                               0               sk.w2                   0                       (sk.w2 * p.f2 * v2xn)   0                               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0;
                    0                                                           0               0               0               0               0                       0                       0                       0                               0               0                       sk.w2                   (sk.w2 * p.f2 * v2yn)   0                               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0;
                    0                                                           0               0               0               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               sk.w3                   0                       (sk.w3 * p.f3 * v3xn)   0                               0               0                       0                       0                       0                               0;
                    0                                                           0               0               0               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       sk.w3                   (sk.w3 * p.f3 * v3yn)   0                               0               0                       0                       0                       0                               0;
                    0                                                           0               0               0               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               sk.w4                   0                       (sk.w4 * p.f4 * v4xn)   0                               0;
                    0                                                           0               0               0               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       sk.w4                   (sk.w4 * p.f4 * v4yn)   0                               0;
%                     -isk.w1 * (r1.y - s.y)                                      isk.w1          0               0               0               0                       0                       0                       -(isk.w1 * p.R1 * cos(s.a1))    0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0;
%                     +isk.w1 * (r1.x - s.x)                                      0               isk.w1          0               0               0                       0                       0                       -(isk.w1 * p.R1 * sin(s.a1))    0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0;
%                     -isk.w2 * (r2.y - s.y)                                      isk.w2          0               0               0               0                       0                       0                       0                               0               0                       0                       0                       -(isk.w2 * p.R2 * cos(s.a2))    0               0                       0                       0                       0                               0               0                       0                       0                       0                               0;
%                     +isk.w2 * (r2.x - s.x)                                      0               isk.w2          0               0               0                       0                       0                       0                               0               0                       0                       0                       -(isk.w2 * p.R2 * sin(s.a2))    0               0                       0                       0                       0                               0               0                       0                       0                       0                               0;
%                     -isk.w3 * (r3.y - s.y)                                      isk.w3          0               0               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       0                       0                       -(isk.w3 * p.R3 * cos(s.a3))    0               0                       0                       0                       0                               0;
%                     +isk.w3 * (r3.x - s.x)                                      0               isk.w3          0               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       0                       0                       -(isk.w3 * p.R3 * sin(s.a3))    0               0                       0                       0                       0                               0;
%                     -isk.w4 * (r4.y - s.y)                                      isk.w4          0               0               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       0                       0                       -(isk.w4 * p.R4 * cos(s.a4))    0;
%                     +isk.w4 * (r4.x - s.x)                                      0               isk.w4          0               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       0                       0                       0                               0               0                       0                       0                       -(isk.w4 * p.R4 * sin(s.a4))    0;
        ];
    
        b = [       p.mr * s.w^2 * (p.xT * cos(s.psi) - p.yT * sin(s.psi));
                    p.mr * s.w^2 * (p.yT * cos(s.psi) + p.xT * sin(s.psi));
                    (p.mr * p.g) / 4;
                    (p.mr * p.g) / 4;
                    (p.mr * p.g) / 4;
                    (p.mr * p.g) / 4;
                    (p.mr * p.g);
                    0;
                    0;
                    0;
                    Mt(1);
                    %Ms(1);
                    Mt(2);
                    %Ms(2);
                    Mt(3);
                    %Ms(3);
                    Mt(4);
                    %Ms(4);
                    0;
                    0;
                    0;
                    0;
                    0;
                    0;
                    0;
                    0;
%                     ((s.w * dr1.y) - (p.R1 * s.Q1 * s.da1 * sin(s.a1))) * isk.w1;
%                     (-(s.w * dr1.x) + (p.R1 * s.Q1 * s.da1 * cos(s.a1))) * isk.w1;
%                     ((s.w * dr2.y) - (p.R2 * s.Q2 * s.da2 * sin(s.a2))) * isk.w2;
%                     (-(s.w * dr2.x) + (p.R2 * s.Q2 * s.da2 * cos(s.a2))) * isk.w2;
%                     ((s.w * dr3.y) - (p.R3 * s.Q3 * s.da3 * sin(s.a3))) * isk.w3;
%                     (-(s.w * dr3.x) + (p.R3 * s.Q3 * s.da3 * cos(s.a3))) * isk.w3;
%                     ((s.w * dr4.y) - (p.R4 * s.Q4 * s.da4 * sin(s.a4))) * isk.w4;
%                     (-(s.w * dr4.x) + (p.R4 * s.Q4 * s.da4 * cos(s.a4))) * isk.w4;
        ];

        
        % solve system
        x = A\b;                                                                            % output vector
        x(26:29) = [sk.w1; sk.w2; sk.w3; sk.w4];                                            % output vector
        x(30:37) = [v1.x; v1.y; v2.x; v2.y; v3.x; v3.y; v4.x; v4.y];                        % output vector
        calc = 0;                                                                           % 0: disable calculation

        x = cell2struct(num2cell(x), cols);
%         % wheels skid detection
%         Tmax = max(1.2 * x(8) * p.f1, 0);
%         T = sqrt(x(6)^2 + x(7)^2);
%         if (T > Tmax)                                                                       % if the friction force is greater than the normal force, the wheel starts to skid (i.e. this force is less effective)
%             sk.w1 = 1;                                                                      % wheel skids
%             calc = 1;                                                                       % repeat calculation with wheel skid update
%         end
% 
%         Tmax = max(1.2 * x(13) * p.f2, 0);
%         T = sqrt(x(11)^2 + x(12)^2);
%         if (T > Tmax)                                                                       % if the friction force is greater than the normal force, the wheel starts to skid (i.e. this force is less effective)
%             sk.w2 = 1;                                                                      % wheel skids
%             calc = 1;                                                                       % repeat calculation with wheel skid update
%         end
% 
%         Tmax = max(1.2 * x(18) * p.f3, 0);
%         T = sqrt(x(16)^2 + x(17)^2);
%         if (T > Tmax)                                                                       % if the friction force is greater than the normal force, the wheel starts to skid (i.e. this force is less effective)
%             sk.w3 = 1;                                                                      % wheel skids
%             calc = 1;                                                                       % repeat calculation with wheel skid update
%         end
% 
%         Tmax = max(1.2 * x(23) * p.f4, 0);
%         T = sqrt(x(21)^2 + x(22)^2);
%         if (T > Tmax)                                                                       % if the friction force is greater than the normal force, the wheel starts to skid (i.e. this force is less effective)
%             sk.w4 = 1;                                                                      % wheel skids
%             calc = 1;                                                                       % repeat calculation with wheel skid update
%         end

        % aposteriori friction coefficient estimation
        T = sqrt(x.Tx1^2 +x.Ty1^2);
        N = x.N1;
        
        if N > 0.01
            if T/N > 1.1 * p.f1
                p.f1 = T/N;
                sk.w1 = 1;  
                calc = 1;
            end
        end
        
        T = sqrt(x.Tx2^2 + x.Ty2^2);
        N = x.N2;
        if N > 0.01
            if T/N > 1.1 * p.f2
                p.f2 = T/N;
                sk.w2 = 1;  
                calc = 1;
            end
        end
        
        T = sqrt(x.Tx3^2 + x.Ty3^2);
        N = x.N3;
        if N > 0.01
            if T/N > 1.1 * p.f3
                p.f3 = T/N;
                sk.w3 = 1;  
                calc = 1;
            end
        end
        
        T = sqrt(x.Tx4^2 + x.Ty4^2);
        N = x.N4;
        if N > 0.01
            if T/N > 1.1 * p.f4
                p.f4 = T/N;
                sk.w4 = 1;  
                calc = 1;
            end
        end

    end
    
    p_new = p;
end

function r = loc2glob(s, xL, yL, zL, e, a)
    
    n = [cos(a); sin(a); 0];                                                                % angle vector

    Rz = [cos(s.psi), -sin(s.psi),  0;
          sin(s.psi),  cos(s.psi),  0;
          0,         0,         1];                                                         % rotational matrix :: transform local coordinates to global coordinates

    r0 = [s.x; s.y; 0];                                                                       % global coordinates of the kinematics centre
    rc = [xL; yL; zL];                                                                      % local coordinates of the touch point 

    r = r0 + (Rz*rc) - (e*n);                                                               % global coordinates of the touch point
    r = struct('x', r(1), 'y', r(2), 'z', r(3));
end






























