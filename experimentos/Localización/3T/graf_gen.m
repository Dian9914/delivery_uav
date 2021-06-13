pos_x = csvread('pos_x.csv',1);
pos_y = csvread('pos_y.csv',1);


i=1;

% encontrar donde esta el problema con los datos
while pos_x(i,4) ~= 0
    i=i+1;
end
ground_truth = [pos_x(:,1) pos_x(:,2)];
KF_pose = [pos_x(1:i-1,3) pos_x(1:i-1,4)];
UAL_pose = [pos_x(1:i-2,5) pos_x(1:i-2,6)];

figure(1)
plot(ground_truth(:,1),ground_truth(:,2));
grid; xlabel("Tiempo [s]"); ylabel("Coordenada [m]");

hold on 
plot(KF_pose(:,1),KF_pose(:,2))
plot(UAL_pose(:,1),UAL_pose(:,2))
hold off
legend("Posición real","Posición KF", "Posición UAL");




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

