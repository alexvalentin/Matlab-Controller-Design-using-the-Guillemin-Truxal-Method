
%% Laboratory 3 - Control Engineering I

%% CONTROLLER DESIGN USING THE GUILLEMIN-TRUXAL METHOD
%% GOALS

% Study the method of controller design using the Guillemin-Truxal approach

% Understand the output variation y(t) for different performance sets 

% Interpretation of the effects of imposed performances using simple
% control structures

% Understabd the method of controller design using graphical representation

%% Problem
% Consider the fixed parts: Kf = 2, Tf = 2 sec, the performance sets are
% [...] . Compute the controllers using the Guilemin-Truxal method, if
% necessary simplify the controller structure and check the performance of
% the closed loop system using Matlab.

%% (a)

% Hf = Kf/ (s* (Tf*s+1))

Hf = tf(2, [2 1 0])
sigma = 0.15 %overshoot
ts = 35; %settling time

zeta = abs(log(sigma)/ sqrt(pi^2 + (log(sigma))^2)) %damping ratio
wn = 4/(ts*zeta) %natural frequency

cv = wn/2/zeta % velocity coefficient

deltawb = wn*sqrt(1-2*zeta^2 + sqrt(2-4*zeta^2+4*zeta^4))%bandwidth

H0 = tf(wn^2, [1 2*zeta*wn wn^2])

Hr = minreal(H0/Hf/(1-H0)); %the controller
step(H0)

t=0:0.1:50;
figure, lsim(H0, t, t) % Simulate time response of dynamic systems
figure, bode(H0) % at -3dB magnitude, we read the freq. = deltawb

%% (b)

Hf = tf(2, [2 1 0])
sigma = 0.15
ts=6

zeta = abs(log(sigma)/ sqrt(pi^2 + (log(sigma))^2))
wn = 4/(ts*zeta)

cv = wn/2/zeta % cv>=1

deltawb = wn*sqrt(1-2*zeta^2 + sqrt(2-4*zeta^2+4*zeta^4)) % <=2 rad/sec

H0 = tf(wn^2, [1 2*zeta*wn wn^2]) % the closed loop of the system

Hr = minreal(H0/Hf/(1-H0)); %the controller
step(H0)

t=0:0.1:50;
figure, lsim(H0, t, t) % Simulate time response of dynamic systems
figure, bode(H0) % at -3dB magnitude, we read the freq. = deltawb

%% (c)

Hf = tf(2, [2 1 0])
sigma = 0.1
ts=3

zeta = abs(log(sigma)/ sqrt(pi^2 + (log(sigma))^2))
wn = 4/(ts*zeta)

cv = wn/2/zeta % cv> = 1.5

deltawb = wn*sqrt(1-2*zeta^2 + sqrt(2-4*zeta^2+4*zeta^4)) % (false) <1.2 rad/sec

H0 = tf(wn^2, [1 2*zeta*wn wn^2]) % the closed loop of the system / second order 

Hr = minreal(H0/Hf/(1-H0)); %the controller
step(H0)

t=0:0.1:50;
figure, lsim(H0, t, t) %Simulate time response of dynamic systems
figure, bode(H0) % at -3dB magnitude, we read the freq. = deltawb

