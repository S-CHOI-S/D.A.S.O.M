clear all;close all;clc;

syms g  %gravitational acceleration

%Joint angle
syms theta_1 theta_2 theta_3 theta_4 theta_5 theta_6
q=[theta_1;theta_2;theta_3;theta_4;theta_5;theta_6];

%Length of link
% l1=sym('l_1')
% l2=sym('l_2');
% l3=sym('l_3');
% l4=sym('l_4');
% l5=sym('l_5');
% l6=sym('l_6');
% l7=sym('l_7');

syms l1 l2 l3 l4 l5 l6 l7

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

L4=[  cos(theta_4)         0         sin(theta_4) -l4*sin(theta_4);
            0               1               0               0;
     -sin(theta_4)         0         cos(theta_4) -l4*cos(theta_4);
            0               0               0               1         ];

L5=[        1               0               0               0;
            0               1               0              l5;
            0               0               1               0;
            0               0               0               1         ];

L6=[  cos(theta_5)  -sin(theta_5)         0               0;
      sin(theta_5)   cos(theta_5)         0               0;
            0               0               1              l6;
            0               0               0               1         ];

L7=[        1               0               0               0;
            0         cos(theta_6)  -sin(theta_6)  l7*cos(theta_6);
            0         sin(theta_6)   cos(theta_6)  l7*sin(theta_6);
            0               0               0               1         ];

%%
%Joint's SE(3)
P1=L1;
P2=P1*L2;
P3=P2*L3;
P4=P3*L4;
P5=P4*L5;
P6=P5*L6;
P7=simplify(P6*L7)

EE_pos=P7(1:3,4)

%Jacobian
J_v=simplify(jacobian(EE_pos,q))
J_omega=simplify([P1(1:3,3) P2(1:3,1) P3(1:3,1) P4(1:3,2) P6(1:3,3) P7(1:3,1)])
J=simplify([J_v;J_omega])