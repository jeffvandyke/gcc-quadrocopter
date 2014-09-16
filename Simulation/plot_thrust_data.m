% plots of the data for motors.
% read in from excel with:
% fly_all = xlsread('book1.xlsx', 'A1:E13')

fw = 'fontwidth';
fs = 'fontsize';
lw = 'LineWidth';
ms = 'MarkerSize';
mec = 'MarkerEdgeColor';
mfc = 'MarkerFaceColor';

sig_freq = 490; %hz
copter_weight = 1200; %grams

thrust_per_motor = copter_weight/4; %grams
thrust_per_motor = thrust_per_motor*9.80665e-3; %Newtons

load('test11_1.mat');
load('test10_8.mat');
load('test10_5.mat');
load('test10_0.mat');

pw11_1 = (pwm11_1/255)*1e3/sig_freq;
pw10_8 = (pwm10_8/255)*1e3/sig_freq;
pw10_5 = (pwm10_5/255)*1e3/sig_freq;
pw10_0 = (pwm10_0/255)*1e3/sig_freq;

thrustNewton11_1 = 9.80665e-3*thrustTOT11_1;
thrustNewton10_8 = 9.80665e-3*thrustTOT10_8;
thrustNewton10_5 = 9.80665e-3*thrustTOT10_5;
thrustNewton10_0 = 9.80665e-3*thrustTOT10_0;

%% Pulse-Width vs. Current
figure, hold on
plot(pw11_1,current11_1,'ms:',ms,5,mfc,'m')
plot(pw10_8,current10_8,'rs:',ms,5,mfc,'r')
plot(pw10_5,current10_5,'bs:',ms,5,mfc,'b')
plot(pw10_0,current10_0,'cs:',ms,5,mfc,'c')
    grid on
    xlabel('Pulse Width (ms)',fs,14)
    ylabel('Current (Amps)',fs,14)
    title('Pulse-Width vs. Current',fs,16)
    legend('11.1V','10.8V','10.5V','10.0V','Location','SouthEast')
    
%% Pulse-Width vs. Total Thrust
figure, hold on
plot(pw11_1,thrustNewton11_1,'ms:',ms,5,mfc,'m')
plot(pw10_8,thrustNewton10_8,'rs:',ms,5,mfc,'r')
plot(pw10_5,thrustNewton10_5,'bs:',ms,5,mfc,'b')
plot(pw10_0,thrustNewton10_0,'cs:',ms,5,mfc,'c')
plot(linspace(1.2,1.8,2),[thrust_per_motor,thrust_per_motor],'k')
    grid on
    xlabel('Pulse Width (ms)',fs,14)
    ylabel('Thrust Total (Newtons)',fs,14)
    title('Pulse-Width vs. Thrust',fs,16)
    legend('11.1V','10.8V','10.5V','10.0V','Min Force','Location','SouthEast')
    
    
%% Current vs. Thrust
figure, hold on
plot(current11_1,thrustNewton11_1,'ms:',ms,5,mfc,'m')
plot(current10_8,thrustNewton10_8,'rs:',ms,5,mfc,'r')
plot(current10_5,thrustNewton10_5,'bs:',ms,5,mfc,'b')
plot(current10_0,thrustNewton10_0,'cs:',ms,5,mfc,'c')
plot(linspace(0,6,2),[thrust_per_motor,thrust_per_motor],'k')
    grid on
    ylabel('Thrust (N)',fs,14)
    xlabel('Current (Amps)',fs,14)
    title('Current vs. Thrust',fs,16)
    legend('11.1V','10.8V','10.5V','10.0V','Min Thrust','Location','SouthEast')
    