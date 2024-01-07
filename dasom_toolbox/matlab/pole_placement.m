A = [-1,-2;1,0];
B = [2;0];
C = [0,1];
D = 0;
sys = ss(A,B,C,D);

Pol = pole(sys)

figure(1)
step(sys)
hold on;

p = [-1, -2]; % 선택한 실수 극점

K = place(A,B,p); % 극점 배치를 사용해 gain 행렬 K 찾기
Acl = A-B*K;
syscl = ss(Acl,B,C,D); % closed-loop의 전달함수 구하기
Pcl = pole(syscl) % closed-loop의 극점 확인
% Pcl = 2x1
%   -2.0000
%   -1.0000

figure(1)
step(syscl) % closed-loop system의 step response 확인

p = [-2, -3];
K2 = place(A,B,p);
syscl2 = ss(A-B*K2,B,C,D);
figure(1);
step(syscl2);

stepinfo(syscl)
stepinfo(syscl2)

%%
% 분자 다항식
numerator = [1];

% 분모 다항식
denominator = [1 5 6 4 1];

% 전달 함수 정의
sys = tf(numerator, denominator);

% 전달 함수를 state-space 모델로 변환
sys_ss = ss(sys);

% 원하는 극점 정의
desired_poles = [-1 -2 -3 -4];

% 극점 배치를 위한 피드백 행렬 계산
K = place(sys_ss.A, sys_ss.B, desired_poles);

% 계산된 피드백 행렬 출력
disp('Calculated Feedback Matrix (K):');
disp(K);

% 피드백 시스템 생성
sys_feedback = ss(sys_ss.A - sys_ss.B * K, sys_ss.B, sys_ss.C, sys_ss.D);

% 피드백 시스템의 극점 확인
poles_feedback = pole(sys_feedback);
disp('Calculated Closed-loop Poles:');
disp(poles_feedback);

% 안정성 판단
if all(real(poles_feedback) < 0)
    disp('The system is stable.');
else
    disp('The system is not stable.');
end
