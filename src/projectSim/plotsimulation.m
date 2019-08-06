close all;
figure(1)
subplot 211
plot(t,q(:,1),'linewidth',2);
hold on;
plot(t,q_d(1),'r-','linewidth',4);
title('joint1 position control');
xlabel('time/sec');ylabel('pos/rad');
subplot 212
plot(t,q(:,2),'linewidth',2);
hold on;
plot(t,q_d(2),'r-','linewidth',4);
title('joint2 position control');
xlabel('time/sec');ylabel('pos/rad');

figure(2)
subplot 211
plot(t,q_d(1)-q(:,1),'Linewidth',2);
hold on;
plot(t,0,'r','Linewidth',3);
title('joint1  position error');
xlabel('time/sec');ylabel('pos/rad');
subplot 212
plot(t,q_d(2)-q(:,2),'Linewidth',2);
hold on;
plot(t,0,'r','Linewidth',3);
title('joint2  position error');
xlabel('time/sec');ylabel('pos/rad');