clear all; close all; clc;

%syms th1 th2
th1 = 45 % deg
th2 = 180 % deg

RotZ=[   cosd(th1)    -sind(th1)      0;
         sind(th1)     cosd(th1)      0;
            0             0           1         ];

RotX=[      1             0           0;
            0          cosd(th2)  -sind(th2);
            0          sind(th2)   cosd(th2)    ];

RotD2M = RotZ * RotX

Mposition = [1 1 1];
Mposition = transpose(Mposition)

M2Dposition = RotD2M * Mposition