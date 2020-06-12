clear all; close all; clc

% 随便生成一点数据
theta_yaw_current=(rand-0.5)*pi/3; theta_pitch_current=(rand-0.5)*pi/3; % 当前电机角度，设居中时为零度
z_target=rand+0.5; raidus=rand; x_target=z_target*raidus*cos(rand*2*pi); y_target=sqrt((z_target*raidus)^2-x_target^2); % 相机坐标系下装甲板位置

x_s_camera=0; y_s_camera=-0.5; z_s_camera=0.25; % 世界坐标系下相机位置
x_s_yaw=0;y_s_yaw=0;z_s_yaw=0; % 世界坐标系下yaw电机位置
x_s_pitch=-0.25;y_s_pitch=0;z_s_pitch=0.25; % 世界坐标系下pitch电机位置

T_s_yaw_origin=[eye(3), [x_s_yaw; y_s_yaw; z_s_yaw]; 0, 0, 0, 1]; % 零度时世界坐标系到yaw电机
T_s_pitch_origin=[[0, 0, -1; -1, 0, 0; 0, 1, 0], [x_s_pitch; y_s_pitch; z_s_pitch]; 0, 0, 0, 1]; % 零度时世界坐标系到pitch电机
T_s_camera_origin=[[1, 0, 0; 0, 0, -1; 0, 1, 0], [x_s_camera; y_s_camera; z_s_camera]; 0, 0, 0, 1]; % 零度时世界坐标系到相机

S_yaw_yaw=[0; 0; 1; 0; 0; 0]; % yaw坐标系下yaw电机twist
S_yaw_s=Adjoint(T_s_yaw_origin)*S_yaw_yaw; % 世界坐标系下yaw电机twist
S_pitch_pitch=[0; 0; 1; 0; 0; 0]; % pitch坐标系下pitch电机twist
S_pitch_s=Adjoint(T_s_pitch_origin)*S_pitch_pitch; % 世界坐标系下pitch电机twist

CrossMatrix_yaw_s=VecTose3(S_yaw_s);
CrossMatrix_pitch_s=VecTose3(S_pitch_s);

T_s_yaw_current=expm(CrossMatrix_yaw_s*theta_yaw_current)*T_s_yaw_origin; % 当前世界坐标系到yaw电机
T_s_pitch_current=expm(CrossMatrix_yaw_s*theta_yaw_current)*expm(CrossMatrix_pitch_s*theta_pitch_current)*T_s_pitch_origin; % 当前世界坐标系到pitch电机
T_s_camera_current=expm(CrossMatrix_yaw_s*theta_yaw_current)*expm(CrossMatrix_pitch_s*theta_pitch_current)*T_s_camera_origin; % 当前世界坐标系到相机
T_camera_target_current=[eye(3), [x_target; y_target; z_target]; 0, 0, 0, 1]; % 当前相机到装甲片
T_s_target_current=T_s_camera_current*T_camera_target_current;

% 画初始位置（需要Robotics Toolbox）
figure('Name', 'Before', 'NumberTitle', 'off')
hold on
trplot(T_s_yaw_current, 'c', 'rgb', 'frame', 'Yaw', 'length', 0.25)
trplot(T_s_pitch_current, 'c', 'rgb', 'frame', 'Pitch', 'length', 0.25)
trplot(T_s_camera_current, 'c', 'rgb', 'frame', 'Camera', 'length', 0.25)
trplot(T_s_target_current, 'c', 'rgb', 'frame', 'Target', 'length', 0.25)
view([1, 1, 1])
pbaspect([1, 1, 1])

theta_yaw_destiny=real(-(log(((cos(theta_yaw_current) + sin(theta_yaw_current)*1i)*(x_target*2i + cos(theta_pitch_current) + 2*z_target*cos(theta_pitch_current) - 2*y_target*sin(theta_pitch_current))*1i)/((cos(theta_yaw_current) - sin(theta_yaw_current)*1i)*(2*x_target + cos(theta_pitch_current)*1i + z_target*cos(theta_pitch_current)*2i - y_target*sin(theta_pitch_current)*2i)))*1i)/2);
theta_pitch_destiny=real(-(log(-(cos(theta_yaw_current)*cos(theta_pitch_current) + cos(theta_pitch_current)*sin(theta_yaw_current)*1i + sin(theta_pitch_current)*(((cos(theta_yaw_current) + sin(theta_yaw_current)*1i)*(x_target*2i + cos(theta_pitch_current) + 2*z_target*cos(theta_pitch_current) - 2*y_target*sin(theta_pitch_current))*1i)/((cos(theta_yaw_current) - sin(theta_yaw_current)*1i)*(2*x_target + cos(theta_pitch_current)*1i + z_target*cos(theta_pitch_current)*2i - y_target*sin(theta_pitch_current)*2i)))^(1/2)*2i + x_target*cos(theta_yaw_current)*2i - 2*x_target*sin(theta_yaw_current) + 2*z_target*cos(theta_yaw_current)*cos(theta_pitch_current) + y_target*cos(theta_pitch_current)*(((cos(theta_yaw_current) + sin(theta_yaw_current)*1i)*(x_target*2i + cos(theta_pitch_current) + 2*z_target*cos(theta_pitch_current) - 2*y_target*sin(theta_pitch_current))*1i)/((cos(theta_yaw_current) - sin(theta_yaw_current)*1i)*(2*x_target + cos(theta_pitch_current)*1i + z_target*cos(theta_pitch_current)*2i - y_target*sin(theta_pitch_current)*2i)))^(1/2)*4i - 2*y_target*cos(theta_yaw_current)*sin(theta_pitch_current) + z_target*cos(theta_pitch_current)*sin(theta_yaw_current)*2i + z_target*sin(theta_pitch_current)*(((cos(theta_yaw_current) + sin(theta_yaw_current)*1i)*(x_target*2i + cos(theta_pitch_current) + 2*z_target*cos(theta_pitch_current) - 2*y_target*sin(theta_pitch_current))*1i)/((cos(theta_yaw_current) - sin(theta_yaw_current)*1i)*(2*x_target + cos(theta_pitch_current)*1i + z_target*cos(theta_pitch_current)*2i - y_target*sin(theta_pitch_current)*2i)))^(1/2)*4i - y_target*sin(theta_yaw_current)*sin(theta_pitch_current)*2i - (x_target*cos(theta_yaw_current)*(cos(2*theta_yaw_current)*1i - sin(2*theta_yaw_current))*(4*x_target*cos(theta_pitch_current) + y_target*sin(2*theta_pitch_current)*2i - cos(theta_pitch_current)^2*1i + y_target^2*cos(theta_pitch_current)^2*4i - z_target^2*cos(theta_pitch_current)^2*4i - z_target*cos(theta_pitch_current)^2*4i + x_target^2*4i - y_target^2*4i + 8*x_target*z_target*cos(theta_pitch_current) - 8*x_target*y_target*sin(theta_pitch_current) + y_target*z_target*sin(2*theta_pitch_current)*4i)*2i)/(4*z_target*cos(theta_pitch_current)^2 - 2*y_target*sin(2*theta_pitch_current) + cos(theta_pitch_current)^2 - 4*y_target^2*cos(theta_pitch_current)^2 + 4*z_target^2*cos(theta_pitch_current)^2 + 4*x_target^2 + 4*y_target^2 - 4*y_target*z_target*sin(2*theta_pitch_current)) - (2*x_target*sin(theta_yaw_current)*(cos(2*theta_yaw_current)*1i - sin(2*theta_yaw_current))*(4*x_target*cos(theta_pitch_current) + y_target*sin(2*theta_pitch_current)*2i - cos(theta_pitch_current)^2*1i + y_target^2*cos(theta_pitch_current)^2*4i - z_target^2*cos(theta_pitch_current)^2*4i - z_target*cos(theta_pitch_current)^2*4i + x_target^2*4i - y_target^2*4i + 8*x_target*z_target*cos(theta_pitch_current) - 8*x_target*y_target*sin(theta_pitch_current) + y_target*z_target*sin(2*theta_pitch_current)*4i))/(4*z_target*cos(theta_pitch_current)^2 - 2*y_target*sin(2*theta_pitch_current) + cos(theta_pitch_current)^2 - 4*y_target^2*cos(theta_pitch_current)^2 + 4*z_target^2*cos(theta_pitch_current)^2 + 4*x_target^2 + 4*y_target^2 - 4*y_target*z_target*sin(2*theta_pitch_current)) + (cos(theta_yaw_current)*cos(theta_pitch_current)*(cos(theta_yaw_current) + sin(theta_yaw_current)*1i)*(x_target*2i + cos(theta_pitch_current) + 2*z_target*cos(theta_pitch_current) - 2*y_target*sin(theta_pitch_current))*1i)/((cos(theta_yaw_current) - sin(theta_yaw_current)*1i)*(2*x_target + cos(theta_pitch_current)*1i + z_target*cos(theta_pitch_current)*2i - y_target*sin(theta_pitch_current)*2i)) + (cos(theta_pitch_current)*sin(theta_yaw_current)*(cos(theta_yaw_current) + sin(theta_yaw_current)*1i)*(x_target*2i + cos(theta_pitch_current) + 2*z_target*cos(theta_pitch_current) - 2*y_target*sin(theta_pitch_current)))/((cos(theta_yaw_current) - sin(theta_yaw_current)*1i)*(2*x_target + cos(theta_pitch_current)*1i + z_target*cos(theta_pitch_current)*2i - y_target*sin(theta_pitch_current)*2i)) + (z_target*cos(theta_yaw_current)*cos(theta_pitch_current)*(cos(theta_yaw_current) + sin(theta_yaw_current)*1i)*(x_target*2i + cos(theta_pitch_current) + 2*z_target*cos(theta_pitch_current) - 2*y_target*sin(theta_pitch_current))*2i)/((cos(theta_yaw_current) - sin(theta_yaw_current)*1i)*(2*x_target + cos(theta_pitch_current)*1i + z_target*cos(theta_pitch_current)*2i - y_target*sin(theta_pitch_current)*2i)) - (y_target*cos(theta_yaw_current)*sin(theta_pitch_current)*(cos(theta_yaw_current) + sin(theta_yaw_current)*1i)*(x_target*2i + cos(theta_pitch_current) + 2*z_target*cos(theta_pitch_current) - 2*y_target*sin(theta_pitch_current))*2i)/((cos(theta_yaw_current) - sin(theta_yaw_current)*1i)*(2*x_target + cos(theta_pitch_current)*1i + z_target*cos(theta_pitch_current)*2i - y_target*sin(theta_pitch_current)*2i)) + (2*z_target*cos(theta_pitch_current)*sin(theta_yaw_current)*(cos(theta_yaw_current) + sin(theta_yaw_current)*1i)*(x_target*2i + cos(theta_pitch_current) + 2*z_target*cos(theta_pitch_current) - 2*y_target*sin(theta_pitch_current)))/((cos(theta_yaw_current) - sin(theta_yaw_current)*1i)*(2*x_target + cos(theta_pitch_current)*1i + z_target*cos(theta_pitch_current)*2i - y_target*sin(theta_pitch_current)*2i)) - (2*y_target*sin(theta_yaw_current)*sin(theta_pitch_current)*(cos(theta_yaw_current) + sin(theta_yaw_current)*1i)*(x_target*2i + cos(theta_pitch_current) + 2*z_target*cos(theta_pitch_current) - 2*y_target*sin(theta_pitch_current)))/((cos(theta_yaw_current) - sin(theta_yaw_current)*1i)*(2*x_target + cos(theta_pitch_current)*1i + z_target*cos(theta_pitch_current)*2i - y_target*sin(theta_pitch_current)*2i)))/(2*x_target*sin(theta_yaw_current) - cos(theta_pitch_current)*sin(theta_yaw_current)*1i + sin(theta_pitch_current)*(((cos(theta_yaw_current) + sin(theta_yaw_current)*1i)*(x_target*2i + cos(theta_pitch_current) + 2*z_target*cos(theta_pitch_current) - 2*y_target*sin(theta_pitch_current))*1i)/((cos(theta_yaw_current) - sin(theta_yaw_current)*1i)*(2*x_target + cos(theta_pitch_current)*1i + z_target*cos(theta_pitch_current)*2i - y_target*sin(theta_pitch_current)*2i)))^(1/2)*2i - x_target*cos(theta_yaw_current)*2i - cos(theta_yaw_current)*cos(theta_pitch_current) - 2*z_target*cos(theta_yaw_current)*cos(theta_pitch_current) + y_target*cos(theta_pitch_current)*(((cos(theta_yaw_current) + sin(theta_yaw_current)*1i)*(x_target*2i + cos(theta_pitch_current) + 2*z_target*cos(theta_pitch_current) - 2*y_target*sin(theta_pitch_current))*1i)/((cos(theta_yaw_current) - sin(theta_yaw_current)*1i)*(2*x_target + cos(theta_pitch_current)*1i + z_target*cos(theta_pitch_current)*2i - y_target*sin(theta_pitch_current)*2i)))^(1/2)*4i + 2*y_target*cos(theta_yaw_current)*sin(theta_pitch_current) - z_target*cos(theta_pitch_current)*sin(theta_yaw_current)*2i + z_target*sin(theta_pitch_current)*(((cos(theta_yaw_current) + sin(theta_yaw_current)*1i)*(x_target*2i + cos(theta_pitch_current) + 2*z_target*cos(theta_pitch_current) - 2*y_target*sin(theta_pitch_current))*1i)/((cos(theta_yaw_current) - sin(theta_yaw_current)*1i)*(2*x_target + cos(theta_pitch_current)*1i + z_target*cos(theta_pitch_current)*2i - y_target*sin(theta_pitch_current)*2i)))^(1/2)*4i + y_target*sin(theta_yaw_current)*sin(theta_pitch_current)*2i + (x_target*cos(theta_yaw_current)*(cos(2*theta_yaw_current)*1i - sin(2*theta_yaw_current))*(4*x_target*cos(theta_pitch_current) + y_target*sin(2*theta_pitch_current)*2i - cos(theta_pitch_current)^2*1i + y_target^2*cos(theta_pitch_current)^2*4i - z_target^2*cos(theta_pitch_current)^2*4i - z_target*cos(theta_pitch_current)^2*4i + x_target^2*4i - y_target^2*4i + 8*x_target*z_target*cos(theta_pitch_current) - 8*x_target*y_target*sin(theta_pitch_current) + y_target*z_target*sin(2*theta_pitch_current)*4i)*2i)/(4*z_target*cos(theta_pitch_current)^2 - 2*y_target*sin(2*theta_pitch_current) + cos(theta_pitch_current)^2 - 4*y_target^2*cos(theta_pitch_current)^2 + 4*z_target^2*cos(theta_pitch_current)^2 + 4*x_target^2 + 4*y_target^2 - 4*y_target*z_target*sin(2*theta_pitch_current)) + (2*x_target*sin(theta_yaw_current)*(cos(2*theta_yaw_current)*1i - sin(2*theta_yaw_current))*(4*x_target*cos(theta_pitch_current) + y_target*sin(2*theta_pitch_current)*2i - cos(theta_pitch_current)^2*1i + y_target^2*cos(theta_pitch_current)^2*4i - z_target^2*cos(theta_pitch_current)^2*4i - z_target*cos(theta_pitch_current)^2*4i + x_target^2*4i - y_target^2*4i + 8*x_target*z_target*cos(theta_pitch_current) - 8*x_target*y_target*sin(theta_pitch_current) + y_target*z_target*sin(2*theta_pitch_current)*4i))/(4*z_target*cos(theta_pitch_current)^2 - 2*y_target*sin(2*theta_pitch_current) + cos(theta_pitch_current)^2 - 4*y_target^2*cos(theta_pitch_current)^2 + 4*z_target^2*cos(theta_pitch_current)^2 + 4*x_target^2 + 4*y_target^2 - 4*y_target*z_target*sin(2*theta_pitch_current)) - (cos(theta_yaw_current)*cos(theta_pitch_current)*(cos(theta_yaw_current) + sin(theta_yaw_current)*1i)*(x_target*2i + cos(theta_pitch_current) + 2*z_target*cos(theta_pitch_current) - 2*y_target*sin(theta_pitch_current))*1i)/((cos(theta_yaw_current) - sin(theta_yaw_current)*1i)*(2*x_target + cos(theta_pitch_current)*1i + z_target*cos(theta_pitch_current)*2i - y_target*sin(theta_pitch_current)*2i)) - (cos(theta_pitch_current)*sin(theta_yaw_current)*(cos(theta_yaw_current) + sin(theta_yaw_current)*1i)*(x_target*2i + cos(theta_pitch_current) + 2*z_target*cos(theta_pitch_current) - 2*y_target*sin(theta_pitch_current)))/((cos(theta_yaw_current) - sin(theta_yaw_current)*1i)*(2*x_target + cos(theta_pitch_current)*1i + z_target*cos(theta_pitch_current)*2i - y_target*sin(theta_pitch_current)*2i)) - (z_target*cos(theta_yaw_current)*cos(theta_pitch_current)*(cos(theta_yaw_current) + sin(theta_yaw_current)*1i)*(x_target*2i + cos(theta_pitch_current) + 2*z_target*cos(theta_pitch_current) - 2*y_target*sin(theta_pitch_current))*2i)/((cos(theta_yaw_current) - sin(theta_yaw_current)*1i)*(2*x_target + cos(theta_pitch_current)*1i + z_target*cos(theta_pitch_current)*2i - y_target*sin(theta_pitch_current)*2i)) + (y_target*cos(theta_yaw_current)*sin(theta_pitch_current)*(cos(theta_yaw_current) + sin(theta_yaw_current)*1i)*(x_target*2i + cos(theta_pitch_current) + 2*z_target*cos(theta_pitch_current) - 2*y_target*sin(theta_pitch_current))*2i)/((cos(theta_yaw_current) - sin(theta_yaw_current)*1i)*(2*x_target + cos(theta_pitch_current)*1i + z_target*cos(theta_pitch_current)*2i - y_target*sin(theta_pitch_current)*2i)) - (2*z_target*cos(theta_pitch_current)*sin(theta_yaw_current)*(cos(theta_yaw_current) + sin(theta_yaw_current)*1i)*(x_target*2i + cos(theta_pitch_current) + 2*z_target*cos(theta_pitch_current) - 2*y_target*sin(theta_pitch_current)))/((cos(theta_yaw_current) - sin(theta_yaw_current)*1i)*(2*x_target + cos(theta_pitch_current)*1i + z_target*cos(theta_pitch_current)*2i - y_target*sin(theta_pitch_current)*2i)) + (2*y_target*sin(theta_yaw_current)*sin(theta_pitch_current)*(cos(theta_yaw_current) + sin(theta_yaw_current)*1i)*(x_target*2i + cos(theta_pitch_current) + 2*z_target*cos(theta_pitch_current) - 2*y_target*sin(theta_pitch_current)))/((cos(theta_yaw_current) - sin(theta_yaw_current)*1i)*(2*x_target + cos(theta_pitch_current)*1i + z_target*cos(theta_pitch_current)*2i - y_target*sin(theta_pitch_current)*2i))))*1i)/2);

if theta_yaw_destiny>pi/2 || theta_yaw_destiny<-pi/2
    theta_pitch_destiny=-theta_pitch_destiny;
    theta_yaw_destiny=theta_yaw_destiny-sign(theta_yaw_destiny)*pi;
end
if theta_pitch_destiny>pi/2 || theta_pitch_destiny<-pi/2
    theta_pitch_destiny=theta_pitch_destiny-sign(theta_pitch_destiny)*pi;
end

T_s_yaw_destiny=expm(CrossMatrix_yaw_s*theta_yaw_destiny)*T_s_yaw_origin; % 旋转后世界坐标系到yaw电机
T_s_pitch_destiny=expm(CrossMatrix_yaw_s*theta_yaw_destiny)*expm(CrossMatrix_pitch_s*theta_pitch_destiny)*T_s_pitch_origin; % 旋转后世界坐标系到pitch电机
T_s_camera_destiny=expm(CrossMatrix_yaw_s*theta_yaw_destiny)*expm(CrossMatrix_pitch_s*theta_pitch_destiny)*T_s_camera_origin; % 旋转后世界坐标系到相机
T_camera_target_destiny=T_s_camera_destiny\T_s_target_current; % 旋转后的相机坐标系到装甲片坐标系

% 画最终位置（需要Robotics Toolbox）
figure('Name', 'After', 'NumberTitle', 'off')
hold on
trplot(T_s_yaw_destiny, 'c', 'rgb', 'frame', 'Yaw', 'length', 0.25)
trplot(T_s_pitch_destiny, 'c', 'rgb', 'frame', 'Pitch', 'length', 0.25)
trplot(T_s_camera_destiny, 'c', 'rgb', 'frame', 'Camera', 'length', 0.25)
trplot(T_s_target_current, 'c', 'rgb', 'frame', 'Target', 'length', 0.25)
view([1, 1, 1])
pbaspect([1, 1, 1])

function se3mat = VecTose3(V)
se3mat = [VecToso3(V(1: 3)), V(4: 6); 0, 0, 0, 0];
end

function AdT = Adjoint(T)
[R, p] = TransToRp(T);
AdT = [R, zeros(3); VecToso3(p) * R, R];
end

function [R, p] = TransToRp(T)
R = T(1: 3, 1: 3);
p = T(1: 3, 4);
end

function so3mat = VecToso3(omg)
so3mat = [0, -omg(3), omg(2); omg(3), 0, -omg(1); -omg(2), omg(1), 0];
end