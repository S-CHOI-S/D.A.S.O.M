% 데이터 준비 (예: 사인파 데이터)
fs = 1000; % 샘플링 주파수 (Hz)
t = 0:1/fs:1-1/fs; % 시간 벡터 (1초 동안)
signal = sin(2*pi*50*t) + 0.5*sin(2*pi*120*t);

% 푸리에 변환 수행
N = length(signal); % 데이터 포인트 수
f = (0:N-1)*(fs/N); % 주파수 벡터
X = fft(signal);

% 복소수형태의 주파수 성분 데이터를 절댓값으로 변환
magnitude = abs(X);

% 주파수 성분 시각화
plot(f, magnitude)
xlabel('주파수 (Hz)')
ylabel('크기')
title('주파수 영역 데이터')
