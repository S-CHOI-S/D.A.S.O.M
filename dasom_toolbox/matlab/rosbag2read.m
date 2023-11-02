%%
clear all; close all; clc;
bag = rosbag("/home/choisol/dasom_ws/src/bag/_2023-11-02-17-23-52.bag");
% scp paletrone3@192.168.1.10:~/catkin_ws/src/FAC_MAV_paletrone/FAC_MAV/bag/_2023-11-02-17-23-52.bag ~/dasom_ws/src/bag

%%
topic_name = '/pos_d';
topic_name2 = '/joint_states';
msg_type = 'geometry_msgs/Vector3';
msg_type2 = 'sensor_msgs/JointState';

msgs = readMessages(select(bag, 'Topic', topic_name, 'MessageType', msg_type));
msgs2 = readMessages(select(bag, 'Topic', topic_name2, 'MessageType', msg_type2));
timestamps = cellfun(@(msg) msg.Header.Stamp.Sec + msg.Header.Stamp.Nsec*1e-9, msgs2);

linear_x_values = cellfun(@(msg) msg.X, msgs);
linear_y_values = cellfun(@(msg) msg.Y, msgs);
linear_z_values = cellfun(@(msg) msg.Z, msgs);

angular_x_values = cellfun(@(msg) msg.Linear.X, msgs);
angular_y_values = cellfun(@(msg) msg.Linear.Y, msgs);
angular_z_values = cellfun(@(msg) msg.Linear.Z, msgs);

plot_data = timeseries([linear_x_values, linear_y_values, linear_z_values], timestamps);

time_origin = plot_data.Time(1);

plot_msgs_size = size(msgs)

plot_zeros = zeros(plot_msgs_size(1),6);

%%
%
for i=1:plot_msgs_size(1)
    pose(i,1)=msgs{i,1}.Linear.X;
    pose(i,2)=msgs{i,1}.Linear.Y;
    pose(i,3)=msgs{i,1}.Linear.Z;
    pose(i,4)=msgs{i,1}.Angular.X;
    pose(i,5)=msgs{i,1}.Angular.Y;
    pose(i,6)=msgs{i,1}.Angular.Z;
end

%%
figure
title('Joint2 w & w/o DOB')
%subplot(2,3,1)
%plot(plot_data.Time-time_origin,plot_data.Data(:,1),'-.','LineWidth',2.0);
hold on
plot(plot_data.Time-time_origin,plot_data.Data(:,1),"LineWidth",2)
hold on
plot(plot_data.Time-time_origin,plot_data.Data(:,2),"LineWidth",2)
hold on
plot(plot_data.Time-time_origin,plot_data.Data(:,3),"LineWidth",2)
hold on
plot(plot_data.Time-time_origin,plot_data.Data(:,4),"LineWidth",2)
hold on
plot(plot_data.Time-time_origin,plot_data.Data(:,5),"LineWidth",2)
hold on
plot(plot_data.Time-time_origin,plot_data.Data(:,6),"LineWidth",2)
legend('reference angle','desired angle','measured angle')
xlabel('time[s]')
ylabel('angle[rad]')
%ylim([1 1.85]);
xlim([1 22]);

%%
signal = plot_data.Data(:,6);

% fourier transform
N = length(signal);
X = fft(signal);

% convert data to frequency data
magnitude = abs(X);

% visualization
plot(plot_data.Time-time_origin, magnitude)
xlabel('주파수 (Hz)')
ylabel('크기')
title('주파수 영역 데이터')