clear all;
close all;
clc;

%% Dziedzina

T = 2*pi;        % okres funkcji
N = 1024;        % liczba próbek

dt = T/(N-1);
t = -T/2:dt:T/2; % wektor czasu

Max = 20;        % górna granica sumy

%% Tworzenie funkcji

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

%% Zespolony Szereg Fourier-a

figure(2);
hold on;
grid on;
title('Suma kolejnych harmonicznych');
xlabel('czas, [s]');
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
plot(t,SzF);
axis([-3.5 3.5 -0.1 1.1]);
title('Suma Szeregu Fourier-a, a f(t)');
xlabel('czas [t]');
ylabel('f(t) oraz suma szeregu Fourier-a');

%% Amplitudy i fazy funkcji sinusoidalnych

Amppositive = sqrt(real(Cpositive).^2+imag(Cpositive).^2);
Ampnegative = sqrt(real(Cnegative).^2+imag(Cnegative).^2);

Amp = Amppositive + Ampnegative;


Phasepositive = atan(imag(Cpositive)./real(Cpositive));
Phasenegative = atan(imag(Cnegative)./real(Cnegative));

Phase = Phasenegative;
% lub
% Phase = - Phasepositive;

omega = [2*pi/T*(1:1:Max)];

figure(4);
stem(omega, Amp);
title('Amplituda dla kolejnych harmonicznych');
xlabel('omega [rad/s]');
ylabel('Amplitude');
grid on;

figure(5);
stem(omega, Phase);
title('Przesunięcie fazowe dla kolejnych harmonicznych');
xlabel('omega [rad/s]');
ylabel('Phase [rad]');
grid on;

%% Rekonstrukcja z fal sinusoidalnych

SzF = real(C0);

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
