% position_data_run1 = out.position_data
% correction_data_run1 = out.correction_data

% figure;
% subplot(1,2,1);
% plot(position_data_run1.Time, position_data_run1.Data);
% hold on;
% plot(position_data_run2.Time, position_data_run2.Data);
% plot(position_data_run3.Time, position_data_run3.Data);
% plot(position_data_run4.Time, position_data_run4.Data);
% plot(position_data_run5.Time, position_data_run5.Data);
% 
% grid on;
% xlabel('Time (s)');
% ylabel('Line Position');
% legend('Run 1','Run 2','Run 3','Run 4','Run 5');
% 
% subplot(1,2,2);
% plot(correction_data_run1.Time, correction_data_run1.Data);
% hold on;
% plot(correction_data_run2.Time, correction_data_run2.Data);
% plot(correction_data_run3.Time, correction_data_run3.Data);
% plot(correction_data_run4.Time, correction_data_run4.Data);
% plot(correction_data_run5.Time, correction_data_run5.Data);
% 
% grid on;
% xlabel('Time (s)');
% ylabel('Correction');
% legend('Run 1','Run 2','Run 3','Run 4','Run 5');

u = double(correction_data_run1.Data(:));
y = double(position_data_run1.Data(:));
t = correction_data_run1.Time(:);

idx = t <= 2;

u = u(idx);
y = y(idx);
t = t(idx);

Ts = mean(diff(t));
data = iddata(y, u, Ts);

sys1 = tfest(data, 1, 0);
sys2 = tfest(data, 2, 0);
sys3 = tfest(data, 2, 1);

disp(' ')
disp('1st Order - 0 Zeros:')
sys1;

disp(' ')
disp('2nd Order - 0 Zeros:')
sys2;

disp(' ')
disp('2nd Order - 1 Zero:')
sys3;

figure;
compare(data, sys1, sys2, sys3);

legend('Measured Output', ...
       '1st Order - 0 Zeros', ...
       '2nd Order - 0 Zeros', ...
       '2nd Order - 1 Zero');

grid on;

figure;
subplot(2,1,1)
plot(t,u,'LineWidth',1.2)
xlabel('Time (s)')
ylabel('Input')
xlim([0 2])
grid on
title('Input')

subplot(2,1,2)
plot(t,y,'LineWidth',1.2)
xlabel('Time (s)')
ylabel('Position')
xlim([0 2])
grid on
title('Measured Position')