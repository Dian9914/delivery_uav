pos_x = csvread('pos_x.csv',1);
pos_y = csvread('pos_y.csv',1);
pos_z = csvread('pos_z.csv',1);
%cov_x = csvread('cov_x.csv',1);


i_x=1; i_y=1; i_z=1;
% encontrar donde esta el problema con los datos
while pos_x(i_x,4) ~= 0
    i_x=i_x+1;
end

while pos_y(i_y,4) ~= 0
    i_y=i_y+1;
end

while pos_x(i_z,4) ~= 0
    i_z=i_z+1;
end

ground_truth_x = [pos_x(:,1) pos_x(:,2)];
ground_truth_y = [pos_y(:,1) pos_y(:,2)];
ground_truth_z = [pos_z(:,1) pos_z(:,2)];

KF_pose_x = [pos_x(1:i_x-1,3) pos_x(1:i_x-1,4)];
KF_pose_y = [pos_y(1:i_y-1,3) pos_y(1:i_y-1,4)];
KF_pose_z = [pos_z(1:i_z-1,3) pos_z(1:i_z-1,4)];

UAL_pose_x = [pos_x(1:i_x-3,5) pos_x(1:i_x-3,6)];
UAL_pose_y = [pos_y(1:i_y-2,5) pos_y(1:i_y-2,6)];
UAL_pose_z = [pos_z(1:i_z-1,5) pos_z(1:i_z-1,6)];


figure(1)
plot(ground_truth_x(:,1),ground_truth_x(:,2));
title("Posición X del UAV");
grid; xlabel("Tiempo [s]"); ylabel("Coordenada [m]");

hold on 
plot(KF_pose_x(:,1),KF_pose_x(:,2))
plot(UAL_pose_x(:,1),UAL_pose_x(:,2))
hold off
legend("Posición real","Posición KF", "Posición UAL");
%xlim([0 100]);


figure(2)
plot(ground_truth_y(:,1),ground_truth_y(:,2));
title("Posición Y del UAV");
grid; xlabel("Tiempo [s]"); ylabel("Coordenada [m]");

hold on 
plot(KF_pose_y(:,1),KF_pose_y(:,2))
plot(UAL_pose_y(:,1),UAL_pose_y(:,2))
hold off
legend("Posición real","Posición KF", "Posición UAL");
%xlim([0 100]);


return

figure(3)
plot(ground_truth_z(:,1),ground_truth_z(:,2));
title("Posición Z del UAV");
grid; xlabel("Tiempo [s]"); ylabel("Coordenada [m]");

hold on 
plot(KF_pose_z(:,1),KF_pose_z(:,2))
plot(UAL_pose_z(:,1),UAL_pose_z(:,2))
hold off
legend("Posición real","Posición KF", "Posición UAL");
%xlim([0 100]);



figure(4)
plot(cov_x(:,1),cov_x(:,2));
title("Covarianza del filtro (sigma) en X");
grid; xlabel("Tiempo [s]"); ylabel("Coordenada [m]");



return
% figure(1)
% plot(pos_x(:,1),[pos_x(:,2) pos_x(:,3) pos_x(:,4)]);
% title("Posición X del UAV")
% grid; xlabel("Tiempo [s]"); ylabel("Coordenada [m]");
% legend("Posición real","Posición KF", "Posición UAL");

figure(1)
plot(pos_x(:,1),pos_x(:,2))
grid; xlabel("Tiempo [s]"); ylabel("Coordenada [m]");

hold on 
plot(pos_x(:,3),pos_x(:,4))
plot(pos_x(:,5),pos_x(:,6))
hold off
legend("Posición real","Posición KF", "Posición UAL");


return
figure(2)
plot(pos_y(:,1),[pos_y(:,2) pos_y(:,3) pos_y(:,4)]);
title("Posición X del UAV")
grid; xlabel("Tiempo [s]"); ylabel("Coordenada [m]");
legend("Posición real","Posición KF", "Posición UAL");

