
clc;
clear;
close all;

%% startup :: init variables

dt = 0.01000;                                                           % simulation time stamp [s]

spdStruct = struct( ...
    'max', 0.0, ...                                                     % [m/s] or [rad/s]
    'acc', 0.0, ...                                                     % [m/s^2] or [rad/s^2]
    'dec', 0.0 ...                                                      % [m/s^2] or [rad/s^2]
    );                                                                  % speed vector component params

pivotStruct = struct( ...
    'X', 0.0, ...                                                       % position from kinematic centre [m]
    'Y', 0.0, ...                                                       % cartesian position from kinematic centre [m]
    'R', 0.0, ...                                                       % wheel radius [m]
    'enT', 0, ...                                                       % 0: inactive traction, 1: active traction
    'enS', 0 ...                                                        % 0: inactive steering, 1: active steering
    );                                                                  % pivot parameters

pSpd = repmat(spdStruct,3,1);                                           % initialize matrix
pPivot = repmat(pivotStruct,1,2);                                       % initialize matrix

%% init :: set parameters
eMin = 100;
eVal = 10000;                                                           % [%]
eStep = 100;                                                          % [%]
eMode = [0, ...                                                         % (0,x): common, (1,x): differential, (2,x): pivot1, (3,x): pivot2
         4];                                                            % (x,0): circular wheel, (x,1): eliptical wheel, (x,2): X distance, (x,3): Y distance, (x, 4) encoder resolution

pEnc = [10000, 10000; ...                                               % encoder resolution :: pulses per revolution [-]
        1,     1];                                                      % 0: disable encoder, 1: enable encoder

pPivot(1).Y = 0.235;                                                    % [m]
pPivot(1).R = 0.110;                                                    % [m]
pPivot(1).enT = 1;                                                      % 1: activate

pPivot(2).Y = -0.235;                                                   % [m]
pPivot(2).R = 0.110;                                                    % [m]
pPivot(2).enT = 1;                                                      % 1: activate

pSpd(1).max = 0.250;                                                    % [m/s]
pSpd(1).acc = 0.250;                                                    % [m/s^2]
pSpd(1).dec = 0.250;                                                    % [m/s^2]

pSpd(3).max = 0.285;                                                    % [rad/s]
pSpd(3).acc = 0.285;                                                    % [rad/s^2]
pSpd(3).dec = 0.285;                                                    % [rad/s^2]

%% generate speed platform :: based on trajectory

%               O       T
trajectory = [  1       30;                                             % go straight
                3       6.5198;                                         % rotate pi/2 :: CCW
                1       17.5;                                           % go straight
                3       6.5198;                                         % rotate pi/2 :: CCW
                1       30;                                             % go straight
                3       6.5198;                                         % rotate pi/2 :: CCW
                1       17.5;                                           % go straight
                4       12.020;                                         % rotate pi :: CW
                1       17.5;                                           % go straight
                4       6.50045;                                        % rotate pi/2 :: CCW
                1       30;                                             % go straight
                4       6.50045;                                        % rotate pi/2 :: CCW
                1       17.5;                                           % go straight
                4       6.50045;                                        % rotate pi/2 :: CCW
                1       30;                                             % go straight
                3       12.020;                                         % rotate pi :: CW

];                                                                      % O: operation (1:fwd, 2: bwd, 3: CCW, 4: CW), T: operation time [s]

v = SpdPlatform(pSpd, trajectory, dt);                                  % target speed vector

%% runtime :: simulation

id = 1;
nmr = numel(0:eStep:eVal);
error = zeros(nmr, 4);

for i=eMin:eStep:eVal
    
    [err, estimatedXYA, trueXYA, dist, Q, Q_true] = ...
    Simulation(v, pPivot, pEnc, eMode, i, dt);                          % call simulation model        

    error(id,1) = sum(err(:,1)) / numel(err(:,1));
    id = id + 1;

%     index = num2str(i);
%     nameVal = "Error between real and estimated trajectory :: params error: " + index + " [%]"; 
%     figure('Name', nameVal)
%     plot(dist(:,2), err(:,1));
%     xlabel('WMR travelled distance [m]');
%     ylabel('Error value [%]');
    
end

figure('Name','Real WMR trajectory');
plot(trueXYA(:,1), trueXYA(:,2));
xlabel('X [m]');
ylabel('Y [m]');

figure('Name','Estimated WMR trajectory');
plot(estimatedXYA(:,1), estimatedXYA(:,2));
xlabel('X [m]');
ylabel('Y [m]');

figure('Name','Error');
x = 0:numel(error(:,1))-1;
x = x * eStep;
loglog(x', error(:,1));
xlabel('Encoder ticks per turn');
ylabel('Mean localization error [m]');

%% simulation functions

function [eVal, pXYA, pXYA_true, dist, Q, Q_true] = Simulation(v, p, pEnc, eM, e, dt)

    N = size(v, 1);
    wa = zeros(1,2);                                                    % initialize matrix
    inc = zeros(1,2);                                                   % initialize matrix
    prevInc = zeros(1,2);                                               % initialize matrix
    prevPos = zeros(1,3);                                               % initialize matrix
    prevPos_true = zeros(1,3);                                          % initialize matrix
    eVal = zeros(N,1);                                      % initialize matrix
    pXYA = zeros(N,3);                                      % initialize matrix
    pXYA_true = zeros(N,3);                                 % initialize matrix
    Q = zeros(N, 2);
    Q_true = zeros(N, 2);

    pDist = 0;                                                          % initialize value
    pDist_true = 0;                                                     % initialize value
    dQ_odo = zeros(1,2);
    dist = zeros(N,2);                                      % initialize matrix

    for i=1:1:N
        
        dx = v(i,1);                                                    % [m/s]
        dy = v(i,2);                                                    % [m/s]
        dw = v(i,3);                                                    % [rad/s]

        % required speed
        [A, dQ] = InvKinematic(dx, dy, dw, p);                          % inverse kinematic :: non-holonomic WMR

        % simulate wheel rotation
        for j=1:1:2
            wa(j) = wa(j) + dQ(j)*dt;
            if (pEnc(2,j) == 1 && e > 0 && eM(2) == 4)                                         % encoder is present
                inc(j) = EncoderData(e, wa(j));                 % get current inc value
                a = (inc(j)-prevInc(j)) * (2*pi/e);
                dQ_odo(j) = a/dt;
                prevInc(j) = inc(j);                                    % store value
            else                                                        % without encoder
                dQ_odo(j) = dQ(j);
            end
        end
        
        Q(i,:) = dQ_odo;
        Q_true(i,:) = dQ;

        % true value
        pTrue = EditParams(p, eM, e, wa);                               % edit pivot parameters  
        [s, pDist_true] = DirKinematic(A, dQ, pTrue, ...
                          prevPos_true, pDist_true, dt);                % direct kinematic :: non-holonomic WMR
        
        prevPos_true = s;                                               % store position
        pXYA_true(i,1) = s(1,1);                                        % current position :: [m]
        pXYA_true(i,2) = s(1,2);                                        % current position :: [m]
        pXYA_true(i,3) = s(1,3);                                        % current position :: [rad]


        % estimated value       
        [s, pDist] = DirKinematic(A, dQ_odo, p, ...
                     prevPos, pDist, dt);                               % direct kinematic :: non-holonomic WMR
        
        prevPos = s;                                                    % store position
        pXYA(i,1) = s(1,1);                                             % modified position :: [m]
        pXYA(i,2) = s(1,2);                                             % modified position :: [m]
        pXYA(i,3) = s(1,3);                                             % modified position :: [rad]

        eVal(i,1) = GetError(pXYA_true(i,:), pXYA(i,:));                
        eVal(i,2) = pXYA(i,3) - pXYA_true(i,3);

        dist(i,1) = pDist;
        dist(i,2) = pDist_true;
    end
    
end


% parameters error model
function mVal = ModifyValue(eMode, i, aVal, eVal)

    switch eMode

        case 0
            mVal = aVal + (aVal * eVal);

        case 1
            if (i == 1)
                mVal = aVal + (aVal * eVal);
            else
                mVal = aVal - (aVal * eVal);
            end

        case 2
            if (i == 1)
                mVal = aVal + (aVal * eVal);
            else
                mVal = aVal;
            end

        case 3
            if (i == 2)
                mVal = aVal + (aVal * eVal);
            else
                mVal = aVal;
            end

        otherwise
            mVal = aVal;

    end

end

function mp = EditParams(p, eMode, eVal, angle)

    mp = p;
    e = (eVal/100);
    
    for i=1:1:2

        switch eMode(2)

            case 0
                mp(i).R = ModifyValue(eMode(1), i, p(i).R, e);
                               
            case 1
                a = p(i).R*(1 + e) * cos(angle(i));
                b = p(i).R*(1 - e) * sin(angle(i));
                mp(i).R = sqrt(a^2 + b^2);

            case 2
                mp(i).X = ModifyValue(eMode(1), i, p(i).X, e);

            case 3
                mp(i).Y = ModifyValue(eMode(1), i, p(i).Y, e);

        end

    end

end

function incVal = EncoderData(res, angle)
    
    stepVal = 2*pi/res;                                     % get angular step
    incVal = floor(angle/stepVal);                          % get number of pulses
    
end

function e = GetError(p1, p2)

    x = (p2(1,1) - p1(1,1))^2;
    y = (p2(1,2) - p1(1,2))^2;
    e = sqrt(x+y);

end


% kinematic models
function [A, dQ] = InvKinematic(dx, dy, dw, p)

    A = zeros(1,2);                                         % default values
    dQ = zeros(1,2);                                        % default values

    for i=1:1:2
        
        if (p(i).enS == 1)
            A(i) = atan2((dy + dw*p(i).X), ...
                         (dx - dw*p(i).Y));                 % target steering angle :: [rad]
        end

        if (p(i).enT == 1)
            dQ(i) = (1/p(i).R) ...
                  * ((cos(A(i))*(dx - dw*p(i).Y)) ...
                  + (sin(A(i))*(dy + dw*p(i).X)));          % target traction speed :: [rad/s]
        end

    end

end

function [s, dist] = DirKinematic(A, dQ, p, prevPos, prevDist, dt)
    
    J = [1, 0, -p(1).Y;
         0, 1, p(1).X;
         1, 0, -p(2).Y;
         0, 1, p(2).X];                                     % mobile robot matrix

    JT = transpose(J);                                      % transposed mobile robot matrix

    dq = [dQ(1)*p(1).R*cos(A(1));
          dQ(1)*p(1).R*sin(A(1));
          dQ(2)*p(2).R*cos(A(2));
          dQ(2)*p(2).R*sin(A(2))];                          % joint workspace

    dp = ((JT*J)\JT) * dq;                                  % pseudo-inversion :: to reach cartesian speed vector
    
    dx = dp(1);                                             % [m/s]
    dy = dp(2);                                             % [m/s]
    dw = dp(3);                                             % [m/s]

    s(3) = wrapToPi(prevPos(3) + (dw*dt));                  % [rad]
    
    s(1) = prevPos(1) ... 
         + (((dx*cos(s(3))) + (dy*sin(s(3)))) * dt);        % [m]
    
    s(2) = prevPos(2) ...
         + (((dx*sin(s(3))) + (dy*cos(s(3)))) * dt);        % [m]

    dist = prevDist + (dx * dt);

end


% generate speed platform
function v = Ramp(p, aSpd, tSpd, dt)
    
    tInc = (p.acc * dt);
    tDec = (p.dec * dt);

    if (abs(tSpd) > p.max)
        tSpd = p.max * (tSpd/abs(tSpd));
    end

    if (tSpd > aSpd)
        
        if (aSpd >= 0.0)         
            aSpd = aSpd + tInc;
            if (aSpd > tSpd)
                aSpd = tSpd;
            end
        end

        if (aSpd < 0.0)         
            aSpd = aSpd + tDec;
            if (aSpd > tSpd)
                aSpd = tSpd;
            end
        end

    elseif (tSpd < aSpd)
        
        if (aSpd >= 0.0)         
            aSpd = aSpd - tDec;
            if (aSpd < tSpd)
                aSpd = tSpd;
            end
        end

        if (aSpd < 0.0)         
            aSpd = aSpd - tInc;
            if (aSpd < tSpd)
                aSpd = tSpd;
            end
        end

    end

    v = aSpd;

end

function v = SpdPlatform(pSpd, traj, dt)

    id = 1;                                                 % current stored data id
    v = zeros(1,3);
    opNmbr = numel(traj(:,1));                              % number of operations in desired trajectory

    for i=1:1:opNmbr                                        % generate speed vector for each operation
        
        opTime = traj(i,2);                                 % operation time

        for j=0:dt:(opTime - dt)
            
            spd = OperationSpd(pSpd, traj(i,1), ...
                traj(i,2), j, dt);                          % generate target speed vector value

            if (id == 1)
                adX = 0.0;                                  % init val
                adY = 0.0;                                  % init val
                adW = 0.0;                                  % init val
            else
                adX = v((id-1),1);                          % previous val
                adY = v((id-1),2);                          % previous val
                adW = v((id-1),3);                          % previous val
            end

            v(id,1) = Ramp(pSpd(1), adX, spd(1), dt);       % ramped speed value [m/s]
            v(id,2) = Ramp(pSpd(2), adY, spd(2), dt);       % ramped speed value [m/s]
            v(id,3) = Ramp(pSpd(3), adW, spd(3), dt);       % ramped speed value [rad/s]
            id = id + 1;                                    % increment each cycle

        end

    end

end

function v = OperationSpd(pSpd, opId, opTime, actTime, dt)
    
    v = zeros(1,3);
    
    if (opId == 1) ...
       && (actTime < (opTime - 1 - dt))
        v(1) = pSpd(1).max;                                 % maximum target speed :: x-component :: fwd
    end

    if (opId == 2) ...
       && (actTime < (opTime - 1 - dt))
        v(1) = -pSpd(1).max;                                % maximum target speed :: x-component :: bwd
    end

    if (opId == 3) ...
       && (actTime < (opTime - 1 - dt))
        v(3) = pSpd(3).max;                                 % maximum target speed :: w-component :: ccw
    end

    if (opId == 4) ...
       && (actTime < (opTime - 1 - dt))
        v(3) = -pSpd(3).max;                                % maximum target speed :: w-component :: cw
    end

end

