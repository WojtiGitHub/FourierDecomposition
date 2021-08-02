clear all;
close all;
clc;

%% Dziedzina

T = 2*pi;        % okres funkcji
N = 1024;        % liczba próbek

dt = T/(N-1);
t = -T/2:dt:T/2; % wektor czasu

Max = 20;        % Gróna granica sumy

%% Tworzenie funkcji ala trójkątnej

rise = (1:1:256)/256;
fall = fliplr(rise);

% Funkcja hat
f = [zeros(1,256), rise, fall, zeros(1,256)];

% Funkcja prostokątna
%f = [zeros(1,256), ones(1,512), zeros(1,256)];


% wyrysowanie funkcji
figure(1);
plot(t,f);
grid on;
title('Funkcja hat lub prostokątna');
xlabel('czas, [s]');
ylabel('f(t)');
axis([-3.5 3.5 -0.1 1.1]);

%% Rzeczywisty Szereg Fouriera

figure(2);
hold on;
grid on;
title('Suma kolejnych harmonicznych');
xlabel('czas, [s]');
ylabel('SzF(t)');

A0 = 2/T*sum(f.*dt);
SzF = A0/2;

for k=1:Max
    
    A(k) = 2/T*sum(f.*cos(2*pi/T*k*t).*dt);
    B(k) = 2/T*sum(f.*sin(2*pi/T*k*t).*dt);
    SzF = SzF + A(k)*cos(2*pi/T*k*t) + B(k)*sin(2*pi/T*k*t);
    
    plot(t,SzF);
    pause(0.1);
    
end

figure(3);
plot(t,f);
grid on;
hold on;
plot(t,SzF);
axis([-3.5 3.5 -0.1 1.1]);
title('Suma Szeregu Fourier-a, a f(t)');
xlabel('czas [t]');
ylabel('f(t) oraz suma szeregu Fourier-a');
%% Amplitudy i fazy funkcji sinusoidalnych

Amp = sqrt(A.^2+B.^2);
Phase = atan(B./A);
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

%% Rekonstrukcja z fal sinusoidalnych

SzF = A0/2;

for k=1:Max
    
    SzF = SzF + Amp(k)*sin(2*pi/T*k*t + Phase(k)+pi/2);
    
end

figure(6);
plot(t,f);
grid on;
title('Suma fal sinusoidalnych a f(t)');
xlabel('czas, [s]');
ylabel('f(t)');
axis([-3.5 3.5 -0.1 1.1]);
hold on;
plot(t,SzF);
