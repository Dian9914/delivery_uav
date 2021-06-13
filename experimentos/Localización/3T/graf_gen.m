pos_x = csvread('pos_x.csv',1);
pos_y = csvread('pos_y.csv',1);


i=1;

% % encontrar donde esta el problema con los datos
% while pos_x(i,3) ~= 0
%     i=i+1;
% end
% 
% % crear una nueva pos_x donde se tome el dato 0 para
% pos_x_new = [];
% 
% size_i = i;
% for j=1:size_i
%     pos_x_new(i,:) = [pos_x(i,1) pos_x(i,2) pos_x (
% 
% end


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

