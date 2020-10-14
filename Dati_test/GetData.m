clear all
close all
clc

data_position=importdata('/home/asusrobot/Documents/Federico-De_Simone/Data_position_Joint6.txt');
data_error=importdata('/home/asusrobot/Documents/Federico-De_Simone/Data_error.txt');

position=data_position(:,1);
y_d=position(1);
y_d=y_d * ones(length(position),1);
position(1)=[];
time_x=data_position(:,2);
time_x(1)=[];
time_x=time_x./1000000;
error=data_error(:,1);

subplot(211), plot(time_x, position, '.r'), ylabel('position [degrees]'), xlabel('Time [ns]'), title('Position 180 with k=0.1')
hold on
plot(y_d, '-b')
subplot(212), plot(time_x, error, '.g'), ylabel('error [degrees]'), xlabel('Time [ns]'), title('-0.001<error<0.001')