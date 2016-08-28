clear all; close all;

% filename1 = 'merged.txt';
% track_data = importdata(filename1)
load('train_v1_labels_gt.mat');
left_hand_driver = lhd;
right_hand_driver = rhd;

%% Kalman Filter for hand detection
%dataset
r = 5
theta = 0:(2*pi)/360:2*pi;
x = r.*cos(theta);
y = r.*sin(theta);
px = x + randn(1,size(theta,2))./100;
py = y + randn(1,size(theta,2))./10;
px = left_hand_driver(:,1) + left_hand_driver(:,3)./2;
py = left_hand_driver(:,2) + left_hand_driver(:,4)./2;

figure;
for i = 1:numel(px)
hold on;
plot(px(i),py(i), '-ko');
%pause(.1);
end
title('original');
axis([-100 700 0 450])

delta_t = 4/15;
A = [1 0 delta_t 0; 0 1 0 delta_t; 0 0 1 0; 0 0 0 1;];
H = [1 0 0 0; 0 1 0 0];

omega_q = 1;
omega_r = 1;
f = 0.1845;
accel_thresh = 1.2e5;

Q = omega_q.*eye(4);
R = omega_r.*[f f/40; f/40 f/4];
%initialization
vel_tx(1) = 0;
vel_tx(1) = 0;
x_t = [px(1) py(1) 0 0];
P_t_minus = Q;
x_hat_t_minus = x_t';

accel_savex = 0;
accel_savey = 0;


%state vector
for i = 2:numel(px)
%     vel_tx(2) = (px(i) - px(i-1))/(1/15);
%     vel_ty(2) = (py(i) - py(i-1))/(1/15);
%     accel_tx = (vel_tx(2) - vel_tx(1))/(1/15);
%     accel_savex(end+1) = accel_tx;
%     accel_ty = (vel_ty(2) - vel_ty(1))/(1/15);
%     accel_savey(end+1) = accel_ty;
    vel_tx(1) = 1%vel_tx(2);
    vel_ty(1) = 1%vel_ty(2);
    
    x_t = [px(i) py(i) vel_tx(1) vel_ty(1)];
    
    x_t = x_t.';
%     if (abs(accel_tx) > accel_thresh) || (abs(accel_ty) > accel_thresh) 
%         omega_q = 0.75/accel_thresh;
%         omega_r = 7.5/(accel_thresh)^2;
%     else
%         omega_q = 0.25/accel_thresh;
%         omega_r = 7.5/accel_thresh;
%     end
    
%     Q = omega_q.*eye(4);
%     R = omega_r.*[f f/40; f/40 f/4];
    
    P_t = Q;
    z_t = H*x_t;
    

    K_f = P_t_minus*(H.')*((H*P_t_minus*(H.')+R)^-1);
    x_hat_t = x_hat_t_minus + K_f*(z_t - H*x_hat_t_minus);
    P_t = (eye(4) - K_f*H)*P_t_minus;
    
    
    x_hat_t_minus = A*x_hat_t;
    P_t_minus = A*P_t*(A.') + Q;
    stateSaves(i,:) = x_hat_t_minus; 
end

figure;
for i = 1:numel(px)
    hold on;
plot(stateSaves(i,1),stateSaves(i,2), '-rx');
plot(px(i),py(i), '-ko');

%pause(.1);

end
title('predicted');
axis([-100 700 0 450])

figure;
for i = 1:numel(px)
    hold on;
plot(stateSaves(i,1),stateSaves(i,2), '-rx');
plot(px(i),py(i), '-ko');    

end
title('(Traditional) Kalman Filter Estimates and Ground Truths');
xlabel('X');
ylabel('Y');
legend('KF Estimates', 'Ground Truths')
grid on;
set(gca, 'Ydir', 'reverse');
