clear all; close all;

filename1 = 'merged.txt';
track_data = importdata(filename1)

left_hand_driver = track_data(1:3:end,:);
right_hand_driver = track_data(2:3:end,:);
right_hand_pass = track_data(3:3:end,:);

%% Kalman Filter for hand detection
%dataset

pxl = left_hand_driver(:,1) + left_hand_driver(:,3)./2;
pyl = left_hand_driver(:,2) + left_hand_driver(:,4)./2;


pxr = right_hand_driver(:,1) + right_hand_driver(:,3)./2;
pyr = right_hand_driver(:,2) + right_hand_driver(:,4)./2;


figure;
for i = 1:numel(pxl)
hold on;
plot(pxl(i),pyl(i), '-ko');
%pause(.1);
end
title('original left');
axis([-100 700 0 450])

figure;
for i = 1:numel(pxr)
hold on;
plot(pxr(i),pyr(i), '-ko');
%pause(.1);
end
title('original right');
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
vel_txl(1) = 0;
vel_txl(1) = 0;
x_tl = [pxl(1) pyl(1) 0 0];
P_tl_minus = Q;
x_hat_tl_minus = x_tl';


vel_txr(1) = 0;
vel_txr(1) = 0;
x_tr = [pxr(1) pyr(1) 0 0];
P_tr_minus = Q;
x_hat_tr_minus = x_tr';



%state vector
for i = 2:numel(pxl)

    vel_txl(1) = 1;
    vel_tyl(1) = 1;
    
    x_tl = [pxl(i) pyl(i) vel_txl(1) vel_tyl(1)];
    
    x_tl = x_tl.';
    
    P_tl = Q;
    z_tl = H*x_tl;
    

    K_fl = P_tl_minus*(H.')*((H*P_tl_minus*(H.')+R)^-1);
    x_hat_tl = x_hat_tl_minus + K_fl*(z_tl - H*x_hat_tl_minus);
    P_tl = (eye(4) - K_fl*H)*P_tl_minus;
    
    
    x_hat_tl_minus = A*x_hat_tl;
    P_tl_minus = A*P_tl*(A.') + Q;
    stateSavesl(i,:) = x_hat_tl_minus; 
    
    vel_txr(1) = 1;
    vel_tyr(1) = 1;
    
    x_tr = [pxr(i) pyr(i) vel_txr(1) vel_tyr(1)];
    
    x_tr = x_tr.';
    
    P_tr = Q;
    z_tr = H*x_tr;
    

    K_fr = P_tr_minus*(H.')*((H*P_tr_minus*(H.')+R)^-1);
    x_hat_tr = x_hat_tr_minus + K_fr*(z_tr - H*x_hat_tr_minus);
    P_tr = (eye(4) - K_fr*H)*P_tr_minus;
    
    
    x_hat_tr_minus = A*x_hat_tr;
    P_tr_minus = A*P_tr*(A.') + Q;
    stateSavesr(i,:) = x_hat_tr_minus;







end

figure;
for i = 1:numel(pxl)
    hold on;
plot(stateSavesl(i,1),stateSavesl(i,2), '-rx');
plot(pxl(i),pyl(i), '-ko');
end
title('predicted left');
axis([-100 700 0 450])

figure;
for i = 1:numel(pxl)
    hold on;
plot(stateSavesr(i,1),stateSavesr(i,2), '-rx');
plot(pxr(i),pyr(i), '-ko');
end
title('predicted right');
axis([-100 700 0 450])



figure;
for i = 1:numel(pxl)
    hold on;
plot(stateSavesl(i,1),stateSavesl(i,2), '-rx');
plot(stateSavesr(i,1),stateSavesr(i,2), '-gx');


pause(.025)
plot(pxl(i),pyl(i), '-ko'); 
plot(pxr(i),pyr(i), '-bo');    


end
title('predicted and actual');

