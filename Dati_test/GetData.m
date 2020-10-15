clear all
close all
clc

data_position=importdata('/home/asusrobot/Documents/Federico-De_Simone/Dati_test/Controllo_Posizione/Data_position_Joint6.txt');
data_error=importdata('/home/asusrobot/Documents/Federico-De_Simone/Dati_test/Controllo_Posizione/Data_error.txt');
data_velocities=importdata('/home/asusrobot/Documents/Federico-De_Simone/Dati_test/Controllo_Posizione/Data_velocities.txt');

position=data_position(:,1);

time_x=data_position(:,2);
time_x=time_x./1000000;
error=data_error(:,1);

velocity_command = data_velocities(:,1);
velocity_measured = data_velocities(:,2);



subplot(211), plot(time_x, position, '.r'), ylabel('position [degrees]'), xlabel('Time [ms]'), title('Position 100->0 with k=0.7')
hold on

subplot(212), plot(time_x, error, '.g'), ylabel('error [degrees]'), xlabel('Time [ms]'), title('-0.001<error<0.001')

figure(2)
plot(time_x, velocity_command, '.r'), ylabel('velocity [degrees/s]'), xlabel('Time [ms]'), title('Velocity')
hold on
plot(time_x, velocity_measured, '.r'), ylabel('velocity measured [degrees/s]'), xlabel('Time [ms]'), title('Velocity')



