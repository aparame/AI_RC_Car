clc
clear
close all

% a= arduino();
% s = servo(a,'D13', 'MinPulseDuration', 1e-3, 'MaxPulseDuration', 2e-3); %D13 is Steering Control
% v = servo(a,'D12', 'MinPulseDuration', 1e-3, 'MaxPulseDuration', 2e-3);
k1 = 1;
k2 = 0.005;
cam = webcam('HD Webcam C615'); % change it if not working
cam.Resolution = '1920x1080';
load('cameraParams.mat')
intpar=(cameraParams.IntrinsicMatrix)';
focal=cameraParams.FocalLength(1);
Fu=intpar(1,1);
Fv=intpar(2,2);
u0=intpar(1,3);
v0=intpar(2,3);

Yc = 380;
Cpos =0;
ii = 1;
leftlines=zeros(4,2);
rightlines=zeros(4,2); % Two group of lines

%% Controller
while (1)
    pic = snapshot(cam);
    pic = imresize(pic,0.25);


    shape = size(pic);
    rlim1=90;glim1=140;blim1=190; % Low Limits
    rlim2=160;glim2=200;blim2=255; % High Limits

    % Color Selection for lane detection
    [x,y] = find(pic(:,:,1)<rlim1|pic(:,:,2)<glim1|pic(:,:,3)<blim1|...
        pic(:,:,1)>rlim2|pic(:,:,2)>glim2|pic(:,:,3)>blim2);
    [x2,y2] = find(rlim2>pic(:,:,1)>rlim1 | glim2>pic(:,:,2)>glim1...
        | blim2>pic(:,:,3)>blim1);
    nl_pic = pic;
    for i =1:length(x)
        nl_pic(x(i), y(i),:)=[0, 0, 0];
        % make all colors below & above lim as 0
    end

    for i =1:length(x2)
        nl_pic(x2(i), y2(i),:)=[255, 255, 255];
        % make all colors between lim as 255
    end
    gray_pic=rgb2gray(pic);
    edge_pic = edge(gray_pic,'canny',[0.25 0.3]);

    % region masking by selecting a polygon
    % specify row coordinates of polygon
    a = [0, shape(2), shape(2), 0];

    % specify column coordinates of polygon
    b = [shape(1)*0.45, shape(1)*0.45, shape(1), shape(1)];
    bw = roipoly(pic, a, b);
    BW=(nl_pic(:,:,1)&bw | edge_pic(:,:,1)&bw | nl_pic (:,:,2)&bw...
        | nl_pic (:,:,3)&bw);
    %     figure(1)
    %     imshow(BW)
    % Clean it up.
    % Fill holes.
    BW = imfill(BW, 'holes');
    % Get rid of small blobs.
    BW = bwareaopen(BW, 100);
    % Smooth border
    BW = imclose(BW, true(9));
%     figure(2)
%     imshow(BW)
    %     % Display the original color image.
    %     subplot(2, 2, 3);
    %     imshow(binaryImage);
    %     axis on;
    %     title('Cleaned Binary Image');
    %
    %
    % hough line detection ------------------------------------------------
    [H,T,R] = hough(BW);
    P = houghpeaks(H,100,'threshold',1);
    lines = houghlines(BW,T,R,P,'FillGap',4,'MinLength',2);
%     imagesc(pic);
%     hold on;
    %

    anglethres=0.000002; %separate left/right by orientation threshold
    leftlines=[];rightlines=[]; %Two group of lines
    for k = 1:length(lines)
        x1=lines(k).point1(1);y1=lines(k).point1(2);
        x2=lines(k).point2(1);y2=lines(k).point2(2);
        if (x2>=2*shape(2)/3) && ((y2-y1)/(x2-x1)>anglethres)
            rightlines=[rightlines;x1,y1;x2,y2];
        elseif (x2<=shape(2)/3) && ((y2-y1)/(x2-x1)<(-1*anglethres))
            leftlines=[leftlines;x1,y1;x2,y2];
        end
    end
    draw_y=[shape(1)*0.6,shape(1)]; %two row coordinates

    if not(isempty(leftlines))
        PL=polyfit(leftlines(:,2),leftlines(:,1),1);
        draw_lx=polyval(PL,draw_y); %two col coordinates of left line
    else
        draw_lx=[draw_rx(1)-300;draw_rx(2)-310];
    end
    if not(isempty(rightlines))
        PR=polyfit(rightlines(:,2),rightlines(:,1),1);
        draw_rx=polyval(PR,draw_y); %two col coordinates of right line
    else
        draw_rx=[draw_lx(1)+300;draw_lx(2)+310];
    end

    centre_top = draw_lx(2) + (draw_rx(2) - draw_lx(2))/2;
    centre_bottom = draw_lx(1) + (draw_rx(1) - draw_lx(1))/2;

    figure(2)
    imagesc(pic);
    hold on;
    plot(draw_lx,draw_y,'LineWidth',5,'Color','red');
    hold on
    plot([centre_bottom centre_top],draw_y,'LineWidth',3,'LineStyle','--','Color','black')
    hold on
    plot(draw_rx,draw_y,'LineWidth',5,'Color','red');
%     hold on
    % Calculating reference Angle
    %     ul = draw_lx;
    %     vl = draw_y;
    %     kl = [];
    %     ur = draw_rx;
    %     vr = draw_y;
    %     kr = [];
    % Calculating reference Angle
    % Extracted Lane Marker Points
    Zcl=(Fv*Yc)./(draw_y-v0);
    Xcl=Zcl.*(draw_lx-u0)/Fu;
    Xlf=Xcl(1);
    Xln=Xcl(2);
    Zlf=Zcl(1);
    Zln=Zcl(2);

    Zcr=(Fv*Yc)./(draw_lx-v0);
    Xcr=Zcr.*(draw_y-u0)/Fu;
    Xrf=Xcr(1);
    Xrn=Xcr(2);
    Zrf=Zcr(1);
    Zrn=Zcr(2);

    % CenterLine
    Xcf=0.5*(Xrf+Xlf);
    Zcf=0.5*(Zrf+Zlf);
    Xcn=0.5*(Xrn+Xln);
    Zcn=0.5*(Zrn+Zln);
   

    %     % this code lets you convert image pixel into real world coordinates.
    %     for i = 1:length(ul)
    %         R = [ul(i);vl(i);1] \ cameraParams.IntrinsicMatrix;
    %         kl = cat(1,kl,R);
    %     end
    %     for i = 1:length(ur)
    %         R = [ur(i);vr(i);1] \ cameraParams.IntrinsicMatrix;
    %         kr = cat(1,kr,R);
    %     end%     %Code below is for getting the middle line from two lanes.
    %     %
    %     Mid = [(kl(1,1) + kr(1,1))/2 , (kl(1,2) + kr(1,2))/2; (kl(2,1) + kr(2,1))/2 , (kl(2,2) + kr(2,2))/2 ];
    %     %     %         plot(Mid(:,1), Mid(:,2));
    %     %     hold off;
    %     % Stanley Controller Design
    %
    %     dep_Angle = atand((Mid(end,2) - Mid(1,2))/(Mid(end,1) - Mid(1,1)));
    %     dep_Distance = Mid(1,1) + ((Mid(end,1) - Mid(1,1)) / (Mid(end,2) - Mid(1,2))) * Mid(1,2);
    %
    %     if mod(ii,3) == 0
    %         writePosition(v,0.46);
    %     else
    %         writePosition(v,0.55);
    %     end
    %
    %     phi=-k1*dep_Angle+atan2(k2*dep_Distance,0.47);   % Steering Angle
    %
    %
    %
    %     writePosition(s,phi);

    %
    %     % this line was written to adjust the
    %     % steering input such that it remains between 0 and 1.
    %     disp(phi)
    %
    %     % steering = 1 is right, steering 0 is left, 0.5 is mid. working properly.
    %     %     the following line makes sure that steering input always remains
    %     %     between 0 and 1, even if the value of the steeing input goes below
    %     %     zero and above 1.
    %
    %     if steeringAngle < 0
    %         steeringAngle = 0
    %     elseif steeringAngle > 1
    %         steeringAngle = 1
    %     end
    %     %     Around sharp corners the vehicle cannot detect inner line, then it
    %     %     throws an error that an array is empty. This code deals with that
    %     %     issue when such array is empty.
    %     if isempty(rightlines) && mod(ii,2) == 0
    %         steeringAngle = 1
    %         writePosition(s,steeringAngle);
    %         pause(0.001);
    %     elseif isempty(leftlines) && mod(ii,2) == 0
    %         steeringAngle = 0
    %         writePosition(s,steeringAngle);
    %         pause(0.001);
    %     else
    %         writePosition(s,steeringAngle);
    %         pause(0.001);
    %     end
    %     %     temp = steeringAngle
    ii = ii + 1;
end