clc
clear
close all
fclose(instrfindall);

a = arduino('/dev/cu.usbmodem11101', 'Uno', 'Libraries', 'Servo'); % Check COM#
s = servo(a,'D13', 'MinPulseDuration', 1e-3, 'MaxPulseDuration', 2e-3); %D13 is Steering Control
v = servo(a,'D12', 'MinPulseDuration', 1e-3, 'MaxPulseDuration', 2e-3);

%% Define computer-specific variables
% Modify these values to be those of your first computer:
ipA = '198.21.229.206';   portA = 9090;
% Modify these values to be those of your second computer:
ipB = '198.21.170.15';  portB = 9091;


udpB = udp(ipA,portA,'LocalPort',portB);
fopen(udpB);
ii = 0;
data = 0;
%%
while(1)
    ii = ii+1;
    if udpB.BytesAvailable > 0
        data = fread(udpB, udpB.BytesAvailable)
        flushinput(udpB)
    end
    if data == 5 && mod(ii,5) == 0
        pause(0.01)
        writePosition(v,0.47);
    elseif data == 2
        writePosition(v,0.55);
    elseif data == 1 && mod(ii,10) == 0
        pause(0.01)
        writePosition(v,0.47);
    end
end





