clear all; close all; clc;
bag = rosbag("C:\Users\hami0\OneDrive\2023-06-20-09-19-16.bag");
% bag = rosbag("C:\Users\hami0\OneDrive\1__dobXPosition_P300_D100.bag")
%%
bag_plot_data = select(bag,'Topic','/first_publisher','Time',[bag.StartTime+15,bag.StartTime + 60]);
% bag_plot_data = select(bag,'Topic','/measured_dynamixel_position')
bag_plot2_data = select(bag,'Topic','/second_publisher','Time',[bag.StartTime+15,bag.StartTime + 60]);
%%
plot_data = timeseries(bag_plot_data);
plot2_data = timeseries(bag_plot2_data);
%%
%
plot_msgs = readMessages(bag_plot_data,"DataFormat","struct");
plot2_msgs = readMessages(bag_plot2_data,"DataFormat","struct");

%
time_origin = plot_data.Time(1);
time_origin2 = plot2_data.Time(1);

%
plot_msgs_size = size(plot_msgs)
plot2_msgs_size = size(plot2_msgs)

%%
plot_zeros = zeros(plot_msgs_size(1),6);
plot2_zeros = zeros(plot2_msgs_size(1),6);

%%
%
for i=1:plot_msgs_size(1)
    pose(i,1)=plot_msgs{i,1}.Linear.X;
    pose(i,2)=plot_msgs{i,1}.Linear.Y;
    pose(i,3)=plot_msgs{i,1}.Linear.Z;
    pose(i,4)=plot_msgs{i,1}.Angular.X;
    pose(i,5)=plot_msgs{i,1}.Angular.Y;
    pose(i,6)=plot_msgs{i,1}.Angular.Z;
end

%
for i=1:plot2_msgs_size(1)
    pose2(i,1)=plot2_msgs{i,1}.Linear.X;
    pose2(i,2)=plot2_msgs{i,1}.Linear.Y;
    pose2(i,3)=plot2_msgs{i,1}.Linear.Z;
    pose2(i,4)=plot2_msgs{i,1}.Angular.X;
    pose2(i,5)=plot2_msgs{i,1}.Angular.Y;
    pose2(i,6)=plot2_msgs{i,1}.Angular.Z;
end

%%
figure
title('Joint2 w & w/o DOB')
%subplot(2,3,1)
%plot(plot_data.Time-time_origin,plot_data.Data(:,1),'-.','LineWidth',2.0);
hold on
%plot(plot_data.Time-time_origin,plot_data.Data(:,1),"LineWidth",2)
hold on
%plot(plot_data.Time-time_origin,plot_data.Data(:,2),"LineWidth",2)
hold on
%plot(plot_data.Time-time_origin,plot_data.Data(:,3),"LineWidth",2)
hold on
plot(plot_data.Time-time_origin,plot_data.Data(:,4),"LineWidth",2)
hold on
plot(plot_data.Time-time_origin,plot_data.Data(:,5),"LineWidth",2)
hold on
plot(plot_data.Time-time_origin,plot_data.Data(:,6),"LineWidth",2)
legend('reference angle','desired angle','measured angle')
xlabel('time[s]')
ylabel('angle[rad]')
ylim([1 1.85]);
xlim([1 12]);
%%
figure
title('X position response w/o DOB')
hold on
plot(plot2_data.Time-time_origin2,plot2_data.Data(:,1),"LineWidth",2)
hold on
%plot(plot2_data.Time-time_origin2,plot2_data.Data(:,2),"LineWidth",2)
hold on
%plot(plot2_data.Time-time_origin2,plot2_data.Data(:,3),"LineWidth",2)
hold on
plot(plot2_data.Time-time_origin2,plot2_data.Data(:,4),"LineWidth",2)
hold on
%plot(plot2_data.Time-time_origin2,plot2_data.Data(:,5),"LineWidth",2)
hold on
%plot(plot2_data.Time-time_origin2,plot2_data.Data(:,6),"LineWidth",2)
legend('reference position','measured position')
xlabel('time[s]')
ylabel('Position[m]')
ylim([-0.08 0.18]);
xlim([0 10]);
%%
figure
title('Joint2 servoangle')
plot(plot_data.Time-time_origin,plot_data.Data(:,4),'-.','LineWidth',2.0);
hold on
plot(plot_data.Time-time_origin,plot_data.Data(:,6),"LineWidth",2)
legend('reference angle','measured angle')
xlabel('time[s]')
ylabel('angle[rad]')
title('Joint2 servoangle')
ylim([1 1.85]);
xlim([1 12]);