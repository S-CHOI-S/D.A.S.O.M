clear all; close all; clc;
bag=rosbag("/home/choisol/2023-11-10-01-17-31.bag");
%%
bag_optitrack_data           =select(bag,'Topic','/dasomPalletrone/world');
%%
optitrack_data               =timeseries(bag_optitrack_data);
%%
optitrack_msgs              =readMessages(bag_optitrack_data);

time_origin                  =optitrack_data.Time(1);

optitrack_msgs_size         =size(optitrack_msgs);

optitrack                   =zeros(optitrack_msgs_size(1),11);

%% FAC_MAV

figure
hold on
%plot(angle_data.Time-time_origin,angle_data.Data(:,1),'-.','LineWidth',2.0); %stamp.secs
%plot(angle_data.Time-time_origin,angle_data.Data(:,2),'LineWidth',2.0); %stamp.nsecs
%plot(angle_data.Time-time_origin,angle_data.Data(:,3),'LineWidth',2.0); %sec
plot(optitrack_data.Time-time_origin,optitrack_data.Data(:,4),'LineWidth',2.0); %position.X
plot(optitrack_data.Time-time_origin,optitrack_data.Data(:,5),'LineWidth',2.0); %position.Y
plot(optitrack_data.Time-time_origin,optitrack_data.Data(:,6),'LineWidth',2.0); %position.Z
plot(optitrack_data.Time-time_origin,optitrack_data.Data(:,7),'LineWidth',2.0); %orientation.X
plot(optitrack_data.Time-time_origin,optitrack_data.Data(:,8),'LineWidth',2.0); %orientation.Y
plot(optitrack_data.Time-time_origin,optitrack_data.Data(:,9),'LineWidth',2.0); %orientation.Z
plot(optitrack_data.Time-time_origin,optitrack_data.Data(:,10),'LineWidth',2.0);%orientation.W
title('Optitrack')
legend('position X','position Y','position Z','orientation X','orientation Y','orientation Z','orientation W');