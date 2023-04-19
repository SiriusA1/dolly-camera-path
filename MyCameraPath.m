%% clear workspace
clc
clear all
close all

% load variables: BackgroundPointCloudRGB,ForegroundPointCloudRGB,K,crop_region,filter_size)
load data.mat

data3DC = {BackgroundPointCloudRGB,ForegroundPointCloudRGB};
R       = eye(3);

%% Find camera position
M_e = K*[R zeros(3,1)];
foregroundObj = [ForegroundPointCloudRGB(1,:);ForegroundPointCloudRGB(2,:);ForegroundPointCloudRGB(3,:);ones(1,size(ForegroundPointCloudRGB(1,:),2))];
foregroundImage = M_e * foregroundObj;

imgW_i = 400/4;
imgH_i = 640/4;
imgW_e = range(foregroundImage(1,:) ./ foregroundImage(3,:));
imgH_e = range(foregroundImage(2,:) ./ foregroundImage(3,:));

foregroundW = range(ForegroundPointCloudRGB(1,:));
foregroundH = range(ForegroundPointCloudRGB(2,:));
foregroundD_e = mean(ForegroundPointCloudRGB(3,:));

foregroundD_i = foregroundD_e * imgW_e / imgW_i;
start_pos = [0; 0; 0; 1];
theta = linspace(180,0,26); theta(end)=[];
%move_x = linspace(-4, 4, 25);
%move_z_temp = linspace(2, 0, 13);
%move_z = horzcat(linspace(0, 2, 13), move_z_temp(2:end));
move_x = 4*cosd(theta);
move_z = 2*sind(theta);
move = [move_x;zeros(1, 25);move_z;zeros(1, 25)];

foregroundWavg = mean(ForegroundPointCloudRGB(1,:));
hypotenuse = sqrt( (-4-foregroundWavg)^2 + (0-foregroundD_e)^2 );
angle = cos( (0 - foregroundD_e) / hypotenuse);

rotate_y_1_temp = linspace(1, cos(angle), 13);
rotate_y_1 = horzcat(linspace(cos(-angle), 1, 13), rotate_y_1_temp(2:end));
rotate_y_2_temp = linspace(0, -sin(angle), 13);
rotate_y_2 = horzcat(linspace(-sin(-angle), 0, 13), rotate_y_2_temp(2:end));
rotate_y_3_temp = linspace(0, sin(angle), 13);
rotate_y_3 = horzcat(linspace(sin(-angle), 0, 13), rotate_y_3_temp(2:end));

%% Creating images and video
video = VideoWriter('mycamerapath.avi');
video.FrameRate = 5;
open(video);

% create an image sequence
for step = 1 : 25
    tic;
    fname = sprintf('output%03d.png', step);
    fprintf('\nGenerating %s', fname);
    %K(1,1) = imgW_i / foregroundW * (foregroundD_i);
    %K(2,2) = imgH_i / foregroundH * (foregroundD_i);
    R = [rotate_y_1(:, step), 0, rotate_y_2(:, step);...
        0, 1, 0;...
        rotate_y_3(:, step), 0, rotate_y_1(:, step);...
        0, 0, 0];
    extr = [R start_pos + move(:, step)];
    intr = [K [0;0;0]];
    M = intr*extr;
    im = PointCloud2Image(M, data3DC, crop_region, filter_size);
    imwrite(im, fname);
    writeVideo(video, im);
    toc;
end

close(video);