body = [ 1.7312e-04, -1.0074e-05, -1.5604e-04,  2.5301e-01,  1.0340e+00, -9.7732e+00];
    
wheels = [
    [-2.8353e-06, -9.2689e-05,  5.8401e-05,  8.6311e+00,  4.6242e+00,7.4795e-01];
    [-1.7196e-04, -7.6027e-05,  1.0422e-04,  9.8110e+00,  9.1212e-02, 1.4341e-01]
];

% wheels_bias = [
%     [-2.8353e-06, -9.2689e-05,  5.8401e-05,  3.4140e-02,  1.9879e-02, -2.8346e-01];
%     [-1.7196e-04, -7.6027e-05,  1.0422e-04,  5.9175e-02, -8.4940e-04, 1.1748e+00]
% ];



sx = sin(body_roll);
cx = cos(body_roll);
sy = sin(body_pitch);
cy = cos(body_pitch);

Rx = [1 0  0;
      0 cx -sx;
      0 sx cx];
  
Ry = [cy 0 sy;
      0  1  0;
      -sy 0 cy];
  
body2world = Ry * Rx;
  
acc_glob = body2world * body_acc';


fi = sg .* atan2(-sg .* wheels(:, 4), -sg .* wheels(:,5)) + body_pitch;
fi_deg = fi * 180 / pi;

gravity = [0; 0; -9.80665];

acc_wh_glob = zeros(3, 2);
wheel2body = {};
wheels_bias = zeros(2, 6);

for k = 1:2
    s2 = sin(sg(k)*pi/2);
    c2 = cos(sg(k)*pi/2);
    s3 = sin(sg(k)*fi(k));
    c3 = cos(sg(k)*fi(k));

    R2 = [1 0  0;
          0 c2 -s2;
          0 s2 c2];
    R3 = [c3 -s3 0;
          s3 c3 0;
          0  0  1];
      
    wheel2body{k} = R2 * R3;
    gravity_wh = wheel2body{k}' * body2world' * gravity;
    wheels_bias(k,:) = wheels(k,:) - [zeros(1, 3), gravity_wh'];
    wh_cal = wheels(k,:) - wheels_bias(k,:);
    acc_cal = wh_cal(:,4:6);
    acc_wh_glob(:,k) = body2world * wheel2body{k} * acc_cal';
end

acc_wh_glob

% syms s1 c1 s2 c2 s3 c3
% 
% R1 = [c1 -s1 0;
%       s1  c1 0;
%       0   0 1];
%   
% R2 = [1 0  0;
%       0 c2 -s2;
%       0 s2 c2];
% R3 = [c3 -s3 0;
%       s3 c3 0;
%       0  0  1];
% 
% R1 * R2 * R3



