clear all;close all;clc;

syms g  %gravitational acceleration

%Joint angle
syms theta_1 theta_2 theta_3 theta_4 theta_5 theta_6
q=[theta_1;theta_2;theta_3;theta_4;theta_5;theta_6];

%Length of link
l1=sym('l_1');
l2=sym('l_2');
l3=sym('l_3');
l4=sym('l_4');
l5=sym('l_5');
l6=sym('l_6');
l7=sym('l_7');
%%
l1 = 0.05465;
l2 = 0.1585;
l3 = 0.099;
l4 = 0.04233;
l5 = 0.06975;
l6 = 0.04;
l7 = 0.01;
%%
theta_1=0;
theta_2=120;
theta_3=-30;
theta_4=0;
theta_5=0;
theta_6=0;
%
theta_1_des=theta_1;
theta_2_des=theta_2;
theta_3_des=theta_3;
theta_4_des=theta_4;
theta_5_des=theta_5;
theta_6_des=theta_6;

%%
L1=[  cosd(theta_1)  -sind(theta_1)         0               0;
      sind(theta_1)   cosd(theta_1)         0               0;
            0               0               1              l1;
            0               0               0               1         ];

L2=[        1               0               0               0;
            0         cosd(theta_2)  -sind(theta_2)  l2*cosd(theta_2);
            0         sind(theta_2)   cosd(theta_2)  l2*sind(theta_2);
            0               0               0               1         ];

L3=[        1               0               0               0;
            0         cosd(theta_3)  -sind(theta_3)  l3*cosd(theta_3);
            0         sind(theta_3)   cosd(theta_3)  l3*sind(theta_3);
            0               0               0               1         ];

L4=[  cosd(theta_4)         0         sind(theta_4) -l4*sind(theta_4);
            0               1               0               0;
     -sind(theta_4)         0         cosd(theta_4) -l4*cosd(theta_4);
            0               0               0               1         ];

L5=[        1               0               0               0;
            0               1               0              l5;
            0               0               1               0;
            0               0               0               1         ];

L6=[  cosd(theta_5)  -sind(theta_5)         0               0;
      sind(theta_5)   cosd(theta_5)         0               0;
            0               0               1              l6;
            0               0               0               1         ];

L7=[        1               0               0               0;
            0         cosd(theta_6)  -sind(theta_6)  l7*cosd(theta_6);
            0         sind(theta_6)   cosd(theta_6)  l7*sind(theta_6);
            0               0               0               1         ];

% L1=[  cos(theta_1)  -sin(theta_1)         0               0;
%       sin(theta_1)   cos(theta_1)         0               0;
%             0               0               1              l1;
%             0               0               0               1         ];
% 
% L2=[        1               0               0               0;
%             0         cos(theta_2)  -sin(theta_2)  l2*cos(theta_2);
%             0         sin(theta_2)   cos(theta_2)  l2*sin(theta_2);
%             0               0               0               1         ];
% 
% L3=[        1               0               0               0;
%             0         cos(theta_3)  -sin(theta_3)  l3*cos(theta_3);
%             0         sin(theta_3)   cos(theta_3)  l3*sin(theta_3);
%             0               0               0               1         ];
% 
% L4=[  cos(theta_4)         0         sin(theta_4) -l4*sin(theta_4);
%             0               1               0               0;
%      -sin(theta_4)         0         cos(theta_4) -l4*cos(theta_4);
%             0               0               0               1         ];
% 
% L5=[        1               0               0               0;
%             0               1               0              l5;
%             0               0               1               0;
%             0               0               0               1         ];
% 
% L6=[  cos(theta_5)  -sin(theta_5)         0               0;
%       sin(theta_5)   cos(theta_5)         0               0;
%             0               0               1              l6;
%             0               0               0               1         ];
% 
% L7=[        1               0               0               0;
%             0         cos(theta_6)  -sin(theta_6)  l7*cos(theta_6);
%             0         sin(theta_6)   cos(theta_6)  l7*sin(theta_6);
%             0               0               0               1         ];

%%
P1=L1;
P2=P1*L2;
P3=P2*L3;
P4=P3*L4;
P5=P4*L5;
P6=P5*L6;
P7=P6*L7

P37=L4*L5*L6*L7

EE_pos=P7(1:3,4)

EE_orient=P7(1:3,1:3)
EE_orient_r11=EE_orient(1,1);
EE_orient_r13=EE_orient(1,3);
EE_orient_r23=EE_orient(2,3);
EE_orient_r12=EE_orient(1,2);
EE_orient_r21=EE_orient(2,1);
EE_orient_r22=EE_orient(2,2);
EE_orient_r31=EE_orient(3,1);

alpha = atan2(EE_orient_r21,EE_orient_r11);

EE_orient_roll=atan2(sin(alpha)*EE_orient_r13-cos(alpha)*EE_orient_r23,-sin(alpha)*EE_orient_r12+cos(alpha)*EE_orient_r22)

EE_orient_pitch=atan2(-EE_orient_r31,cos(alpha)*EE_orient_r11+sin(alpha)*EE_orient_r21)	% limit pitch direction

EE_orient_yaw=alpha

xw=EE_pos(1,1)-l7*EE_orient(1,2);
yw=EE_pos(2,1)-l7*EE_orient(2,2);
zw=EE_pos(3,1)-l7*EE_orient(3,2);

Wrist_center=[P6(1,4) P6(2,4) P6(3,4)]
Calc_Wrist_center=[xw yw zw]

%%
% 1,2,3 servomotor's angle
xw=0.035740;
yw=0;
zw=0.25;

r=sqrt(xw^2+yw^2+(zw-l1)^2);

th1=atan2(yw,xw)-pi/2;

if th1 <= -pi
    th1 = th1 + pi;
    xw = -xw
    yw = -yw
end

cth2=[(l2^2)+(r^2)-(l3+l5)^2]/[2*(l2)*r];
th2=atan2(zw-l1,sqrt(xw^2+yw^2))+atan2(sqrt(1-(cth2)^2),cth2); % sign

cth3=[(l2^2)+(l3+l5)^2-(r^2)]/[2*l2*(l3+l5)];
th3= -(pi-atan2(sqrt(1-cth3^2),cth3)); % sign

%Des_theta123=[theta_1_des theta_2_des theta_3_des]
[th1 th2 th3] 
Calc_theta123=[rad2deg(th1) rad2deg(th2) rad2deg(th3)]

%%
%syms roll pitch yaw
theta_1 = th1
theta_2 = th2
theta_3 = th3
roll=pi/2;
pitch=0;
yaw=0;

L1=[  cos(theta_1)  -sin(theta_1)         0               0;
      sin(theta_1)   cos(theta_1)         0               0;
            0               0               1              l1;
            0               0               0               1         ];

L2=[        1               0               0               0;
            0         cos(theta_2)  -sin(theta_2)  l2*cos(theta_2);
            0         sin(theta_2)   cos(theta_2)  l2*sin(theta_2);
            0               0               0               1         ];

L3=[        1               0               0               0;
            0         cos(theta_3)  -sin(theta_3)  l3*cos(theta_3);
            0         sin(theta_3)   cos(theta_3)  l3*sin(theta_3);
            0               0               0               1         ];

P1=L1;
P2=P1*L2;
P3=P2*L3;

Ry = [ cos(pitch)      0      sin(pitch);
           0           1          0     ;
      -sin(pitch)      0      cos(pitch)];
Rz = [  cos(yaw)   -sin(yaw)      0;
        sin(yaw)    cos(yaw)      0;
           0           0          1     ];
Rx = [     1           0          0;
           0       cos(roll)  -sin(roll);
           0       sin(roll)   cos(roll)];

RR=Rz*Ry*Rx

RR37=transpose(P3(1:3,1:3)) * RR

r11=RR37(1,1);
r21=RR37(2,1);
r22=RR37(2,2);
r23=RR37(2,3);
r31=RR37(3,1);
r32=RR37(3,2);
r33=RR37(3,3);
%Des_theta456=[theta_4_des theta_5_des theta_6_des]

pitch=atan2(-r31,r11); % theta_4
yaw=atan2(r21*cos(pitch),r11); % theta_5
roll=atan2(-r23,r22); % theta_6

pitch_d=rad2deg(pitch);
yaw_d=rad2deg(yaw);
roll_d=rad2deg(roll);

[pitch yaw roll]
Cala_theta456=[pitch_d yaw_d roll_d]

%%
RR37
P37(1:3,1:3)