clear all; close all;


%filename1 = 'merged.txt';
cd test_labels

filename1 = 'test_v1_labels';
load(filename1);
cd ..

left_hand_driver = lhd;
right_hand_driver = rhd;
left_hand_pass = lhp;
right_hand_pass = rhp;

%% Kalman Filter for hand detection
%dataset

pxld = left_hand_driver(:,1) + left_hand_driver(:,3)./2;
pyld = left_hand_driver(:,2) + left_hand_driver(:,4)./2;


pxrd = right_hand_driver(:,1) + right_hand_driver(:,3)./2;
pyrd = right_hand_driver(:,2) + right_hand_driver(:,4)./2;

pxrp = left_hand_pass(:,1) + left_hand_pass(:,3)./2;
pyrp = left_hand_pass(:,2) + left_hand_pass(:,4)./2;

pxlp = right_hand_pass(:,1) + right_hand_pass(:,3)./2;
pylp = right_hand_pass(:,2) + right_hand_pass(:,4)./2;

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
%LD
vel_txl(1) = 0;
vel_txl(1) = 0;
x_tl = [pxld(1) pyld(1) 0 0];
P_tl_minus = Q;
x_hat_tl_minus = x_tl';

%RD
vel_txr(1) = 0;
vel_txr(1) = 0;
x_tr = [pxrd(1) pyrd(1) 0 0];
P_tr_minus = Q;
x_hat_tr_minus = x_tr';

%LP
vel_txlp(1) = 0;
vel_txlp(1) = 0;
x_tlp = [pxlp(1) pylp(1) 0 0];
P_tl_minusp = Q;
x_hat_tl_minusp = x_tlp';

%RP
vel_txrp(1) = 0;
vel_txrp(1) = 0;
x_trp = [pxrp(1) pyrp(1) 0 0];
P_tr_minusp = Q;
x_hat_tr_minusp = x_trp';

%%
%state vector
for i = 2:numel(pxld)
%Account for inconsistent data by minimizing 2-norm
    var = ['pxld'; 'pxrd'; 'pxlp'; 'pxrp'];
    Mapping = [];
    
    %pxld
    n11 = abs(pxrp(i) - eval(sprintf('%s(i-1)',var(1,:))));
    n21 = abs(pxrp(i) - eval(sprintf('%s(i-1)',var(2,:))));
    n31 = abs(pxrp(i) - eval(sprintf('%s(i-1)',var(3,:))));
    n41 = abs(pxrp(i) - eval(sprintf('%s(i-1)',var(4,:))));
    [M11, I11] = min([n11 n21 n31 n41]);
    Mapping = [Mapping; sprintf('%s',var(I11(1),:))];
    var(I11(1),:) = [];
    
    %pxrd
    n12 = norm(pxlp(i) - eval(sprintf('%s(i-1)',var(1,:))));
    n22 = norm(pxlp(i) - eval(sprintf('%s(i-1)',var(2,:))));
    n32 = norm(pxlp(i) - eval(sprintf('%s(i-1)',var(3,:))));
    [M12, I12] = min([n12 n22 n32]);
    Mapping = [Mapping; sprintf('%s',var(I12(1),:))];
    var(I12(1),:) = [];

    %pxlp
    n13 = norm(pxrd(i) - eval(sprintf('%s(i-1)',var(1,:))));
    n23 = norm(pxrd(i) - eval(sprintf('%s(i-1)',var(2,:))));
    [M13, I13] = min([n13 n23]);
    Mapping = [Mapping; sprintf('%s',var(I13(1),:))];
    var(I13(1),:) = [];
  
    Mapping = [Mapping; sprintf('%s',var((1),:))];
    Mapping = flipud(Mapping);
    
    %Mapping:
    %[pxld(i)->(1) pxrd->(2) pxlp->(3) pxrp->(4)]
%1st case
    if strcmp(Mapping(1,:),'pxld')
        pxldtemp = pxld(i);
        pyldtemp = pyld(i);
        if strcmp(Mapping(2,:),'pxrd')
            pxrdtemp = pxrd(i);
            pyrdtemp = pyrd(i);
            if strcmp(Mapping(3,:),'pxlp')
                pxlptemp = pxlp(i);
                pylptemp = pylp(i);
                
                pxrptemp = pxrp(i);
                pyrptemp = pyrp(i);
            else
                pxrptemp = pxlp(i);
                pyrptemp = pylp(i);
                
                pxlptemp = pxrp(i);
                pylptemp = pyrp(i);
            end
        %[pxld(i)->(1) pxrd->(2) pxlp->(3) pxrp->(4)]
        elseif strcmp(Mapping(2,:),'pxlp')
            pxlptemp = pxrd(i);
            pylptemp = pyrd(i);
            if strcmp(Mapping(3,:),'pxrd')
                pxrdtemp = pxlp(i);
                pyrdtemp = pylp(i);
                
                pxrptemp = pxrp(i);
                pyrptemp = pyrp(i);
            else
                pxrptemp = pxlp(i);
                pyrptemp = pylp(i);
                
                pxrdtemp = pxrp(i);
                pyrdtemp = pyrp(i);
            end
        %[pxld(i)->(1) pxrd->(2) pxlp->(3) pxrp->(4)]
        else
            pxrptemp = pxrd(i);
            pyrptemp = pyrd(i);
            if strcmp(Mapping(3,:),'pxrd')
                pxrdtemp = pxlp(i);
                pyrdtemp = pylp(i);
                
                pxlptemp = pxrp(i);
                pylptemp = pyrp(i);
            else
                pxlptemp = pxlp(i);
                pylptemp = pylp(i);
                
                pxrdtemp = pxrp(i);
                pyrdtemp = pyrp(i);
            end
        end
%2nd Case
    %[pxld(i)->pxrd pxrd->(2) pxlp->(3) pxrp->(4)]
    elseif strcmp(Mapping(1,:),'pxrd')
        pxrdtemp = pxld(i);
        pyrdtemp = pyld(i);
        %[pxld(i)->pxrd pxrd->pxld pxlp->(3) pxrp->(4)]
        if strcmp(Mapping(2,:),'pxld')
            pxldtemp = pxrd(i);
            pyldtemp = pyrd(i);
            if strcmp(Mapping(3,:),'pxlp')
                pxlptemp = pxlp(i);
                pylptemp = pylp(i);
                
                pxrptemp = pxrp(i);
                pyrptemp = pyrp(i);
            else
                pxrptemp = pxlp(i);
                pyrptemp = pylp(i);
                
                pxlptemp = pxrp(i);
                pylptemp = pyrp(i);
            end
        %[pxld(i)->pxrd pxrd->pxlp pxlp->(3) pxrp->(4)]
        elseif strcmp(Mapping(2,:),'pxlp')
            pxlptemp = pxrd(i);
            pylptemp = pyrd(i);
            if strcmp(Mapping(3,:),'pxld')
                pxldtemp = pxlp(i);
                pyldtemp = pylp(i);
                
                pxrptemp = pxrp(i);
                pyrptemp = pyrp(i);
            else
                pxrptemp = pxlp(i);
                pyrptemp = pylp(i);
                
                pxldtemp = pxrp(i);
                pyldtemp = pyrp(i);
            end
        %[pxld(i)->pxrd pxrd->pxrp pxlp->(3) pxrp->(4)]
        else
            pxrptemp = pxrd(i);
            pyrptemp = pyrd(i);
            if strcmp(Mapping(3,:),'pxld')
                pxldtemp = pxlp(i);
                pyldtemp = pylp(i);
                
                pxlptemp = pxrp(i);
                pylptemp = pyrp(i);
            else
                pxlptemp = pxlp(i);
                pylptemp = pylp(i);
                
                pxldtemp = pxrp(i);
                pyldtemp = pyrp(i);
            end
        end
%3rd Case
    %[pxld(i)->pxlp pxrd->(2) pxlp->(3) pxrp->(4)]
    elseif strcmp(Mapping(1,:),'pxlp')
        pxlptemp = pxld(i);
        pylptemp = pyld(i);
        %[pxld(i)->pxlp pxrd->pxld pxlp->(3) pxrp->(4)]
        if strcmp(Mapping(2,:),'pxld')
            pxldtemp = pxrd(i);
            pyldtemp = pyrd(i);
            if strcmp(Mapping(3,:),'pxrd')
                pxrdtemp = pxlp(i);
                pyrdtemp = pylp(i);
                
                pxrptemp = pxrp(i);
                pyrptemp = pyrp(i);
            else
                pxrptemp = pxlp(i);
                pyrptemp = pylp(i);
                
                pxrdtemp = pxrp(i);
                pyrdtemp = pyrp(i);
            end
        %[pxld(i)->pxlp pxrd->pxrd pxlp->(3) pxrp->(4)]
        elseif strcmp(Mapping(2,:),'pxrd')
            pxrdtemp = pxrd(i);
            pyrdtemp = pyrd(i);
            if strcmp(Mapping(3,:),'pxld')
                pxldtemp = pxlp(i);
                pyldtemp = pylp(i);
                
                pxrptemp = pxrp(i);
                pyrptemp = pyrp(i);
            else
                pxrptemp = pxlp(i);
                pyrptemp = pylp(i);
                
                pxldtemp = pxrp(i);
                pyldtemp = pyrp(i);
            end
        %[pxld(i)->pxlp pxrd->pxrp pxlp->(3) pxrp->(4)]
        else
            pxrptemp = pxrd(i);
            pyrptemp = pyrd(i);
            if strcmp(Mapping(3,:),'pxld')
                pxldtemp = pxlp(i);
                pyldtemp = pylp(i);
                
                pxrdtemp = pxrp(i);
                pyrdtemp = pyrp(i);
            else
                pxrdtemp = pxlp(i);
                pyrdtemp = pylp(i);
                
                pxldtemp = pxrp(i);
                pyldtemp = pyrp(i);
            end
        end
%4th Case
    %[pxld(i)->pxrp pxrd->(2) pxlp->(3) pxrp->(4)]
    else
        pxrptemp = pxld(i);
        pyrptemp = pyld(i);
        %[pxld(i)->pxrp pxrd->pxld pxlp->(3) pxrp->(4)]
        if strcmp(Mapping(2,:),'pxld')
            pxldtemp = pxrd(i);
            pyldtemp = pyrd(i);
            if strcmp(Mapping(3,:),'pxrd')
                pxrdtemp = pxlp(i);
                pyrdtemp = pylp(i);
                
                pxlptemp = pxrp(i);
                pylptemp = pyrp(i);
            else
                pxlptemp = pxlp(i);
                pylptemp = pylp(i);
                
                pxrdtemp = pxrp(i);
                pyrdtemp = pyrp(i);
            end
        %[pxld(i)->pxrp pxrd->pxrd pxlp->(3) pxrp->(4)]
        elseif strcmp(Mapping(2,:),'pxrd')
            pxrdtemp = pxrd(i);
            pyrdtemp = pyrd(i);
            if strcmp(Mapping(3,:),'pxld')
                pxldtemp = pxlp(i);
                pyldtemp = pylp(i);
                
                pxlptemp = pxrp(i);
                pylptemp = pyrp(i);
            else
                pxlptemp = pxlp(i);
                pylptemp = pylp(i);
                
                pxldtemp = pxrp(i);
                pyldtemp = pyrp(i);
            end
        %[pxld(i)->pxrp pxrd->pxlp pxlp->(3) pxrp->(4)]
        else
            pxlptemp = pxrd(i);
            pylptemp = pyrd(i);
            if strcmp(Mapping(3,:),'pxld')
                pxldtemp = pxlp(i);
                pyldtemp = pylp(i);
                
                pxrdtemp = pxrp(i);
                pyrdtemp = pyrp(i);
            else
                pxrdtemp = pxlp(i);
                pyrdtemp = pylp(i);
                
                pxldtemp = pxrp(i);
                pyldtemp = pyrp(i);
            end
        end
    end
    
%%%%%%%%%%%%%%
    %%%%%
    pxld(i) = pxldtemp;
    pyld(i) = pyldtemp;
    
    
    pxrd(i) = pxrdtemp;
    pyrd(i) = pyrdtemp;
    
    pxlp(i) = pxlptemp;
    pylp(i) = pylptemp;
    
    pxrp(i) = pxrptemp;
    pyrp(i) = pyrptemp;
    
   
%%%%%%%%%%%%%Left Hand Driver%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    vel_txl(1) = 1;
    vel_tyl(1) = 1;
    
    x_tl = [pxld(i) pyld(i) vel_txl(1) vel_tyl(1)];
    
    x_tl = x_tl.';
    
    P_tl = Q;
    z_tl = H*x_tl;
    

    K_fl = P_tl_minus*(H.')*((H*P_tl_minus*(H.')+R)^-1);
    x_hat_tl = x_hat_tl_minus + K_fl*(z_tl - H*x_hat_tl_minus);
    P_tl = (eye(4) - K_fl*H)*P_tl_minus;
    
    
    x_hat_tl_minus = A*x_hat_tl;
    P_tl_minus = A*P_tl*(A.') + Q;
    stateSavesl(i,:) = x_hat_tl_minus;
    
%%%%%%%%%%%%%Right Hand Driver%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    vel_txr(1) = 1;
    vel_tyr(1) = 1;
    
    x_tr = [pxrd(i) pyrd(i) vel_txr(1) vel_tyr(1)];
    
    x_tr = x_tr.';
    
    P_tr = Q;
    z_tr = H*x_tr;
    

    K_fr = P_tr_minus*(H.')*((H*P_tr_minus*(H.')+R)^-1);
    x_hat_tr = x_hat_tr_minus + K_fr*(z_tr - H*x_hat_tr_minus);
    P_tr = (eye(4) - K_fr*H)*P_tr_minus;
    
    
    x_hat_tr_minus = A*x_hat_tr;
    P_tr_minus = A*P_tr*(A.') + Q;
    stateSavesr(i,:) = x_hat_tr_minus;
    
%%%%%%%%%%%%%Left Hand Passenger%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    vel_txlp(1) = 1;
    vel_tylp(1) = 1;
    
    x_tlp = [pxlp(i) pylp(i) vel_txlp(1) vel_tylp(1)];
    
    x_tlp = x_tlp.';
    
    P_tlp = Q;
    z_tlp = H*x_tlp;
    

    K_flp = P_tl_minusp*(H.')*((H*P_tl_minusp*(H.')+R)^-1);
    x_hat_tlp = x_hat_tl_minusp + K_flp*(z_tlp - H*x_hat_tl_minusp);
    P_tlp = (eye(4) - K_flp*H)*P_tl_minusp;
    
    
    x_hat_tl_minusp = A*x_hat_tlp;
    P_tl_minusp = A*P_tlp*(A.') + Q;
    stateSaveslp(i,:) = x_hat_tl_minusp; 
    
%%%%%%%%%%%%%Right Hand Passenger%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    vel_txrp(1) = 1;
    vel_tyrp(1) = 1;
    
    x_trp = [pxrp(i) pyrp(i) vel_txrp(1) vel_tyrp(1)];
    
    x_trp = x_trp.';
    
    P_trp = Q;
    z_trp = H*x_trp;
    

    K_frp = P_tr_minusp*(H.')*((H*P_tr_minusp*(H.')+R)^-1);
    x_hat_trp = x_hat_tr_minusp + K_frp*(z_trp - H*x_hat_tr_minusp);
    P_trp = (eye(4) - K_frp*H)*P_tr_minusp;
    
    
    x_hat_tr_minusp = A*x_hat_trp;
    P_tr_minusp = A*P_trp*(A.') + Q;
    stateSavesrp(i,:) = x_hat_tr_minusp;

    %Check if hands are on wheel
    wheel = [30 270 276 76];
    hands = numHandsonWheel(wheel, stateSavesr(i,:), stateSavesl(i,:), stateSaveslp(i,:), stateSavesrp(i,:));
    
    handTracking(i,:) = hands;
    
    disp(i)
end
%%
% figure;
% axis([0 700 0 450])
% title('Left Hand KF Estimates and ACF Detections in Real Time');
% xlabel('X');
% ylabel('Y');
% grid on;
% set(gca,'Ydir','reverse');
% for i = 1:numel(pxl)
%     hold on;
% plot(stateSavesl(i,1),stateSavesl(i,2), '-rx');
% %pause(.1)
% 
% plot(pxl(i),pyl(i), '-ko');
% %legend('KF Estimates','ACF Detections')
% end
%%%%%%%%%%%%%%%%Ground Truth Plots%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure;
for i = 1:numel(pxld)
hold on;
plot(pxld(i),pyld(i), '-ko');
%pause(.1);
end
title('Original Left Driver');
set(gca,'Ydir','reverse');
axis([-100 700 0 450])

figure;
for i = 1:numel(pxrd)
hold on;
plot(pxrd(i),pyrd(i), '-ko');
%pause(.1);
end
title('Original Right Driver');
set(gca,'Ydir','reverse');
axis([-100 700 0 450])

figure;
for i = 1:numel(pxld)
hold on;
plot(pxlp(i),pylp(i), '-ko');
%pause(.1);
end
title('Originial Left Passenger');
set(gca,'Ydir','reverse');
axis([-100 700 0 450])

figure;
for i = 1:numel(pxld)
hold on;
plot(pxrp(i),pyrp(i), '-ko');
%pause(.1);
end
title('Originial Right Passenger');
set(gca,'Ydir','reverse');
axis([-100 700 0 450])

%%%%%%%%%%%%%%%%Normal Kalman Filter Plots%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% figure;
% axis([0 700 0 450])
% title('Right Hand KF Estimates and ACF Detections in Real Time');
% xlabel('X');
% ylabel('Y');
% grid on;
% set(gca,'Ydir','reverse');
%     hold on;
% scatter(stateSavesr(:,1),stateSavesr(:,2), 'rx');
% scatter(pxrd(:),pyrd(:), 'ko');
% legend('KF Estimates','ACF Detections')
% hold on;
% 
% 
% figure;
% axis([0 700 0 450])
% title('Left Hand KF Estimates and ACF Detections in Real Time');
% xlabel('X');
% ylabel('Y');
% grid on;
% set(gca,'Ydir','reverse');
% hold on;
% scatter(stateSavesl(:,1),stateSavesl(:,2), 'gx');
% scatter(pxld(:),pyld(:), 'bo');    
% legend('KF Estimates','ACF Detections')
% hold on;

%%%%%%%%%%%%%%%%Real Time Kalman Filter Plots%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
image = ones(100,200)*255;
figure;
imagesc(450,40,image,[0 255]);
axis([0 700 0 450])
title('Left Hand KF Estimates and ACF Detections in Real Time');
xlabel('X');
ylabel('Y');
grid on;
set(gca,'Ydir','reverse');
for i = 1:numel(pxld)
    hold on;
    colormap gray;
plot(stateSavesr(i,1),stateSavesr(i,2), '-rx');
if i ~= 1
    image = Numbers(image, handTracking(i-1),-500,0,255);
    imagesc(450,40,image,[0 255]);
end
image = Numbers(image, handTracking(i),-500,0,0);
imagesc(450,40,image,[0 255]);

pause(.025)
plot(pxrd(i),pyrd(i), '-ko');
legend('KF Estimates','ACF Detections')
end

image = ones(100,200)*255;
figure;
imagesc(450,40,image,[0 255]);
axis([-100 700 0 450])
title('Right Hand KF Estimates and ACF Detections in Real Time');
xlabel('X');
ylabel('Y');
grid on;
set(gca,'Ydir','reverse');
for i = 1:numel(pxld)
    hold on;
    colormap gray;
plot(stateSavesl(i,1),stateSavesl(i,2), '-rx');
plot(stateSavesl(i,1),stateSavesl(i,2), '-gx');
if i ~= 1
    image = Numbers(image, handTracking(i-1),-500,0,255);
    imagesc(450,40,image,[0 255]);
end
image = Numbers(image, handTracking(i),-500,0,0);
imagesc(450,40,image,[0 255]);

pause(.025)
plot(pxld(i),pyld(i), '-ko'); 
plot(pxld(i),pyld(i), '-bo');    

legend('KF Estimates','ACF Detections')
end
yj
save('TestSetResults.mat','stateSavesl','stateSavesr','stateSaveslp','stateSavesrp','pxld','pyld','pxrd','pyrd','pxlp','pylp','pxrp','pyrp','handTracking');
