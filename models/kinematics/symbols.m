%% 
syms s1 s2 s3 c1 c2 c3 real
syms d1 d2 d3 real

% Z-Y-X convention (yaw-pitch-roll)
R1 = [c1 -s1 0; s1 c1 0; 0 0 1];
R2 = [c2 0 s2; 0 1 0; -s2 0 c2];
R3 = [1 0 0; 0 c3 -s3; 0 s3 c3];

Rzyx = R1 * R2 * R3;
wzyx =  R3' * [d3 0 0]' + R3' * R2' * [0 d2 0]' + R3' * R2' * R1' * [0 0 d1]';
Tzyx =  R3' * [1 0 0; 0 0 0; 0 0 0] + R3' * R2' * [0 0 0; 0 1 0; 0 0 0] + R3' * R2' * R1' * [0 0 0; 0 0 0; 0 0 1]


% Z-X-Z convention
R1 = [c1 -s1 0; s1 c1 0; 0 0 1];
R2 = [1 0 0; 0 c2 -s2; 0 s2 c2];
R3 = [c3 -s3 0; s3 c3 0; 0 0 1];
Rzxz = R1 * R2 * R3
wzxz = R3' * [0 0 d3]' + R3' * R2' * [d2 0 0]' + R3' * R2' * R1' * [0 0 d1]'
Tzxz =  R3' * [0 0 0; 0 0 0; 0 0 1] + R3' * R2' * [0 1 0; 0 0 0; 0 0 0] + R3' * R2' * R1' * [0 0 0; 0 0 0; 1 0 0]


%%
syms sx sy sz cx cy cz

Rx = [1 0  0;
      0 cx -sx;
      0 sx cx];
  
Ry = [cy 0 sy;
      0  1  0;
      -sy 0 cy];
  
Rz = [cz -sz 0;
      sz cz 0;
      0  0  1];
  
Rz * Ry * Rx


