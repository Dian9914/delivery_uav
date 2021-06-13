pos_x = csvread('pos_x.csv',1);
pos_y = csvread('pos_y.csv',1);


figure(1)
plot(pos_x(:,1),[pos_x(:,2) pos_x(:,3) pos_x(:,4)]);
title("Posición X del UAV")
grid; xlabel("Tiempo [s]"); ylabel("Coordenada [m]");
legend("Posición real","Posición KF", "Posición UAL");

return
figure(2)
plot(pos_y(:,1),[pos_y(:,2) pos_y(:,3) pos_y(:,4)]);
title("Posición X del UAV")
grid; xlabel("Tiempo [s]"); ylabel("Coordenada [m]");
legend("Posición real","Posición KF", "Posición UAL");

