clear
load linear_tray_top_reachable_x_y;
low = linear_tray_reachable_x_y;
load linear_tray_top_approachable_x_y;
high = linear_tray_top_approachable_x_y;
figure(1)
plot(linear_tray_top_reachable_x_y(:,1),linear_tray_top_reachable_x_y(:,2),'*',linear_tray_top_approachable_x_y(:,1),linear_tray_top_approachable_x_y(:,2),'kx')
