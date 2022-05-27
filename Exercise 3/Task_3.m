figure;
hold on;
plot(angular_position); % Plot angular position w.r.t. time
plot(angular_velocity); % Plot angular velocity w.r.t. time
xlabel('Time (s)');
legend('Angular Position (rad)', 'Angular Velocity (rad/s)');

figure;
plot(angular_position.data, angular_velocity.data); % Plot angular position w.r.t. angular velocity
xlabel('Angular Position (rad)');
ylabel('Angular Velocity (rad/s)');