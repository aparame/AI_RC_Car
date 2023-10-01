clc
clear
close all

a= arduino();
s = servo(a,'D13', 'MinPulseDuration', 1e-3, 'MaxPulseDuration', 2e-3); %D13 is Steering Control
v = servo(a,'D12', 'MinPulseDuration', 1e-3, 'MaxPulseDuration', 2e-3);
k1 = 1;
k2 = 1;
cam = webcam('HD Webcam C615'); % change it if not working
cam.Resolution = '1920x1080';
load('cameraParams.mat')

%% Controller
while (1)
    pic = snapshot(cam);
    pic = rgb2hsv(pic);
    shape = size(pic);
    gray_pic = rgb2gray(pic);
    edge_pic = edge(gray_pic, 'canny',([0.2 0.3]));

    % region masking by selecting a polygon
    % specify row coordinates of polygon
    a = [shape(2)*0.2, shape(2)*0.8, shape(2), 0];

    % specify column coordinates of polygon
    b = [shape(1)*0.5, shape(1)*0.5, shape(1), shape(1)];
    bw = roipoly(pic, a, b);
    BW = (edge_pic(:,:,1)&bw);
%     figure(1)
%     imshow(BW)

    % hough line detection ------------------------------------------------
    [H,T,R] = hough(BW,'RhoResolution',0.75,'Theta',-90:0.1:89.9);
    P = houghpeaks(H,50,'threshold',0);
    lines = houghlines(BW,T,R,P,'FillGap',4,'MinLength',5);
%     imagesc(pic);
%     hold on;
% 
%     for i = 1:length(lines)
%         plot([lines(i).point1(1),lines(i).point2(1)],[lines(i).point1(2),lines(i).point2(2)],'LineWidth',2,'Color','red');
%     end
    anglethres=0.05; % separate left/right by orientation threshold
    leftlines=[];
    rightlines=[]; % Two group of lines
    for k = 1:length(lines)

        x1 = lines(k).point1(1);
        x2 = lines(k).point2(1);
        y1 = lines(k).point1(2);
        y2 = lines(k).point2(2);
        Length = sqrt((x1-x2)^2+(y1-y2)^2);
        if (x2>=shape(2)/2) && ((y2-y1)/(x2-x1)>anglethres && Length>50)
            rightlines = [rightlines;x1,y1;x2,y2];
        elseif (x2<=shape(2)/2) && ((y2-y1)/(x2-x1)<(-1*anglethres) && Length>50)
            leftlines = [leftlines;x1,y1;x2,y2];
        end


        if isempty(rightlines)
            if (x2>=shape(2)/2) || ((y2-y1)/(x2-x1)>anglethres)
                rightlines=[rightlines;x1,y1;x2,y2];
            end
        end

        if isempty(leftlines)
            if (x2<=shape(2)/2) || ((y2-y1)/(x2-x1)<(-1*anglethres))
                leftlines=[leftlines;x1,y1;x2,y2];
            end
        end
    end
        draw_y = [shape(1)*0.6, shape(1)];  % two row coordinates
        PL=polyfit(leftlines(:,2),leftlines(:,1),1);
        draw_lx=polyval(PL,draw_y); %two col coordinates of left line
        PR=polyfit(rightlines(:,2),rightlines(:,1),1);
        draw_rx=polyval(PR,draw_y); %two col coordinates of right line
        figure(2)
        imagesc(pic);
        hold on;
        plot(draw_lx,draw_y,'LineWidth',5,'Color','red');
        hold on;
        plot(draw_rx,draw_y,'LineWidth',5,'Color','red');
        hold off

    % Calculating reference Angle
        ul = draw_lx;
        vl = draw_y;
        kl = [];
        ur = draw_rx;
        vr = draw_y;
        kr = [];
    
        % this code lets you convert image pixel into real world coordinates.
        for i = 1:length(ul)
            R = [ul(i);vl(i);1] \ cameraParams.IntrinsicMatrix;
            kl = cat(1,kl,R);
        end
        for i = 1:length(ur)
            R = [ur(i);vr(i);1] \ cameraParams.IntrinsicMatrix;
            kr = cat(1,kr,R);
        end
    

    Code below is for getting the middle line from two lanes.

        Mid = [(kl(1,1) + kr(1,1))/2 , (kl(1,2) + kr(1,2))/2; (kl(2,1) + kr(2,1))/2 , (kl(2,2) + kr(2,2))/2 ];
        plot(Mid(:,1), Mid(:,2));
        hold off;

    %   Actuation -
        ii = 0
        while (1)
            if mod(ii,5) == 0
                writePosition(v,0.34);
            else
                writePosition(v,0.55);
            end
            ii = ii + 1
    
        end
    Stanley Controller Design
    
        dep_Angle = atand((Mid(end,2) - Mid(1,2))/(Mid(end,1) - Mid(1,1)))
        dep_Distance = Mid(1,1) + ((Mid(end,1) - Mid(1,1)) / (Mid(end,2) - Mid(1,2))) * Mid(1,2)
    
        steeringAngle1 = -k1 * dep_Angle - k2 * dep_Distance % We get delta or steering angle from this line.
        steeringAngle = 0.5 + (0.5/90) * steeringAngle1
    
        % this line was written to adjust the
        % steering input such that it remains between 0 and 1.
        disp(steeringAngle)

    % steering = 1 is right, steering 0 is left, 0.5 is mid. working properly.
    the following line makes sure that steering input always remains
    between 0 and 1, even if the value of the steeing input goes below
    zero and above 1.

        if steeringAngle < 0
            steeringAngle = 0
        elseif steeringAngle > 1
            steeringAngle = 1
        end
    Around sharp corners the vehicle cannot detect inner line, then it
    throws an error that an array is empty. This code deals with that
    issue when such array is empty.
        if isempty(rightlines)
            steeringAngle = 1
            writePosition(s,steeringAngle);
            pause(0.01);
        elseif isempty(leftlines)
            steeringAngle = 0
            writePosition(s,steeringAngle);
            pause(0.01);
        else
            writePosition(s,steeringAngle);
            pause(0.01);
        end
        temp = steeringAngle
end