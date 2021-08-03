clear all;
close all;
clc;

%% Domain

T = 2*pi;        % function period
N = 1024;        % number of samples

dt = T/(N-1);
t = -T/2:dt:T/2; % time moments vector

Max = 20;        % upper sum limit

%% Function creation

rise = (1:1:256)/256;
fall = fliplr(rise);

% hat function
f = [zeros(1,256), rise, fall, zeros(1,256)];

% pulse function
%f = [zeros(1,256), ones(1,512), zeros(1,256)];

% function plotting
figure(1);
plot(t,f);
grid on;
title('f(t)');
xlabel('time, [s]');
ylabel('f(t)');
axis([-3.5 3.5 -0.1 1.1]);

%% Complex Fourier Series

figure(2);
hold on;
grid on;
title('Fourier Series of f(t)');
xlabel('time, [s]');
ylabel('SzF(t)');

C0 = complex(1/T*sum(f.*dt));
SzF = C0;

for k=1:Max
    
    Cnegative(k) = 1/T*sum(f.*exp(-1i*2*pi/T*(-k)*t).*dt);
    Cpositive(k) = 1/T*sum(f.*exp(-1i*2*pi/T*k*t).*dt);
    
    SzF = SzF + Cnegative(k).*exp(1i*2*pi/T*(-k)*t) + Cpositive(k).*exp(1i*2*pi/T*k*t);
    plot(t, real(SzF));
    pause(0.1);
    
end

figure(3);
plot(t,f);
grid on;
hold on;
plot(t,real(SzF));
axis([-3.5 3.5 -0.1 1.1]);
title('Fourier Series of f(t) and f(t)');
xlabel('time [t]');
ylabel('f(t), SzF(t)');

%% Amplitudes and phases of harmonics

Amppositive = sqrt(real(Cpositive).^2+imag(Cpositive).^2);
Ampnegative = sqrt(real(Cnegative).^2+imag(Cnegative).^2);

Amp = Amppositive + Ampnegative;


Phasepositive = atan(imag(Cpositive)./real(Cpositive));
Phasenegative = atan(imag(Cnegative)./real(Cnegative));

Phase = Phasenegative;
% or
% Phase = - Phasepositive;

omega = [2*pi/T*(1:1:Max)];

figure(4);
stem(omega, Amp);
title('Amplitude of sinusiodal function versus frequency');
xlabel('omega [rad/s]');
ylabel('Amplitude');
grid on;

figure(5);
stem(omega, Phase);
title('Phase of sinusiodal function versus frequency');
xlabel('omega [rad/s]');
ylabel('Phase [rad]');
grid on;

%% Function reconstruction using harmonics

SzF = real(C0);

for k=1:Max
    
    SzF = SzF + Amp(k)*sin(2*pi/T*k*t + Phase(k)+pi/2);
    
end

figure(6);
plot(t,f);
grid on;
title('Sum of harmonics and f(t)');
xlabel('time, [s]');
ylabel('f(t), Szf(t)');
axis([-3.5 3.5 -0.1 1.1]);
hold on;
plot(t,SzF)
