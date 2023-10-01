cam = webcam('HD Webcam C615');
cam.Resolution = '1920x1080';
while (1)
    pic = snapshot(cam);
    pic = imresize(pic,0.5);
    imshow(pic);
end

% Set up
a = arduino('/dev/cu.usbmodem11101', 'Uno', 'Libraries', 'Servo'); % Check COM#
v = servo(a,'D12', 'MinPulseDuration', 1e-3, 'MaxPulseDuration', 2e-3);
% Hold ESC button down until light turns red then release
%% After LED blinks once
writePosition(v, 1);
%% After blinks twice
writePosition(v, 0);
%% Steady blinking
writePosition(v, 0.5);
pause(5)


    % Code to aid in finding minimum speed
H = uicontrol('Style', 'slider','SliderStep',[1/1000,1/500],'Min',-.5,'Max',.5,'Position',[50 50 400 40]); % Slider GUI to help find min. values
j = 0;
while (ishandle(H))
    i = get(H,'value');
    if i~=j
        speed = 0.5 + i
        writePosition(v, speed); % Writes value to pin
    end
    j = i;
    pause(.25)
end
writePosition(v, 0.5); % Sets speed to neutral after loop exit

