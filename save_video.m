cam = webcam('HD Webcam C615');
cam.Resolution = '1920x1080';
i = 393;
cd frames
while (1)
    pic = snapshot(cam);
    pic = imresize(pic,0.5);
    imshow(pic);
    % converting integer to string
    Sx = num2str(i);
    i = i + 1;
    % concatenating 2 strings
    Strc = strcat(Sx, '.jpg');
    
    % exporting the frames
    if ~mod(i,2)
        imwrite(pic, Strc);
    else
        continue
    end    
end

