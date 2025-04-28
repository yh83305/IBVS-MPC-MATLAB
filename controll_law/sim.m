% 清空工作区
clear; clc;

% 相机内参
fx = 330; % x 轴方向的焦距 (像素)
fy = 330; % y 轴方向的焦距 (像素)
cx = 300; % 图像中心 x 坐标 (像素)
cy = 300; % 图像中心 y 坐标 (像素)
K = [fx, 0, cx; 0, fy, cy; 0, 0, 1]; % 相机内参矩阵

% 图像分辨率
image_width = 640; % 图像宽度 (像素)
image_height = 480; % 图像高度 (像素)

F_inv = zeros(8, 8); % 初始化 8x8 的零矩阵

% 设置奇数行对角为 fx
F_inv(1:2:end, 1:2:end) = diag(1/fx * ones(4, 1)); % 奇数行对角
% 设置偶数行对角为 fy
F_inv(2:2:end, 2:2:end) = diag(1/fy * ones(4, 1)); % 偶数行对角

% 定义特征点的三维位置 (方形，边长 20cm)
P_world = [0.1, 0.1, 0; -0.1, 0.1, 0; -0.1, -0.1, 0; 0.1, -0.1, 0]'; % 世界坐标系下的三维点 (3x4 矩阵)

% 定义相机初始位置和姿态
R_camera = [-1, 0, 0; 0, 1, 0; 0, 0, -1];
t_camera = [0.05; 0.05; 1];
theta_x = deg2rad(10); % 绕 x 轴旋转角度
theta_y = deg2rad(-10);  % 绕 y 轴旋转角度
R_x = [1, 0, 0; 0, cos(theta_x), -sin(theta_x); 0, sin(theta_x), cos(theta_x)]; % 绕 x 轴旋转
R_y = [cos(theta_y), 0, sin(theta_y); 0, 1, 0; -sin(theta_y), 0, cos(theta_y)]; % 绕 y 轴旋转
R_camera = R_x * R_y* R_camera; % 组合旋转

% 定义目标相机位置和姿态
R_camera_d = [-1, 0, 0; 0, 1, 0; 0, 0, -1];
t_camera_d = [0; 0; 0.5];

% 仿真参数
Ts = 0.1; % 采样时间 (秒)
Tf = 20; % 仿真总时间 (10秒)
t = 0:Ts:Tf; % 时间向量
N = length(t); % 仿真步数

% 存储变量
s_history = zeros(8, N); % 存储视觉特征
tau_history = zeros(6, N); % 存储控制输入
Z_history = zeros(1, N); % 存储深度 Z

% 初始视觉特征 (将三维点投影到图像平面)
s0 = project_points_to_image(P_world, K, R_camera, t_camera, image_width, image_height);
s_history(:, 1) = s0; % 初始视觉特征
s_star = project_points_to_image(P_world, K, R_camera_d, t_camera_d, image_width, image_height); % 参考视觉特征 (目标位置)

% 初始深度 Z (相机到特征点中心的距离)
P_center = mean(P_world, 2); % 特征点中心 (3x1 向量)
Z = norm(R_camera * P_center + t_camera); % 相机到特征点中心的距离
Z_history(1) = Z; % 存储初始深度

% 控制增益
lambda = -0.4; % 控制增益

% 初始化三维可视化
figure;
hold on;
axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('3D Camera and Feature Points');
view(3);
grid on;

% 绘制初始特征点
h_feature_points = plot3(P_world(1, :), P_world(2, :), P_world(3, :), 'bo', 'MarkerSize', 10, 'LineWidth', 2);

% 绘制初始相机位置
h_camera = plot3(t_camera(1), t_camera(2), t_camera(3), 'rs', 'MarkerSize', 10, 'LineWidth', 2);

% 绘制相机的三个轴
axis_length = 0.2; % 轴的长度
h_x_axis = plot3([t_camera(1), t_camera(1) + axis_length * R_camera(1, 1)], ...
                 [t_camera(2), t_camera(2) + axis_length * R_camera(2, 1)], ...
                 [t_camera(3), t_camera(3) + axis_length * R_camera(3, 1)], 'r', 'LineWidth', 2); % x 轴
h_y_axis = plot3([t_camera(1), t_camera(1) + axis_length * R_camera(1, 2)], ...
                 [t_camera(2), t_camera(2) + axis_length * R_camera(2, 2)], ...
                 [t_camera(3), t_camera(3) + axis_length * R_camera(3, 2)], 'g', 'LineWidth', 2); % y 轴
h_z_axis = plot3([t_camera(1), t_camera(1) + axis_length * R_camera(1, 3)], ...
                 [t_camera(2), t_camera(2) + axis_length * R_camera(2, 3)], ...
                 [t_camera(3), t_camera(3) + axis_length * R_camera(3, 3)], 'b', 'LineWidth', 2); % z 轴

% 初始化图像平面可视化
figure;
hold on;
axis equal;
xlabel('u (pixels)'); ylabel('v (pixels)');
title('Feature Points in Image Plane');
grid on;

% 计算视野边缘的四个角点
corners = [
    0, 0;
    image_width, 0;
    image_width, image_height;
    0, image_height;
];

% 绘制视野边缘
plot(corners([1:end, 1], 1), corners([1:end, 1], 2), 'k--', 'LineWidth', 1.5);

% 限制坐标轴到视野边缘
xlim([0, image_width]);
ylim([0, image_height]);

% 绘制初始特征点投影
h_image_points = plot(s0(1:2:end), s0(2:2:end), 'bo', 'MarkerSize', 10, 'LineWidth', 2);

% 绘制参考视觉特征 s_star
h_image_points_star = plot(s_star(1:2:end), s_star(2:2:end), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
legend('Current Features', 'Target Features');

% 为特征点添加标号
for i = 1:4
    text(s0(2*i-1), s0(2*i), num2str(i), 'Color', 'blue', 'FontSize', 12);
    text(s_star(2*i-1), s_star(2*i), num2str(i), 'Color', 'red', 'FontSize', 12);
end

% 主仿真循环
for k = 1:N-1
    % 当前视觉特征
    s = s_history(:, k);
    
    % 计算误差
    e = s - s_star;
    
    % 计算交互矩阵
    Ls = interaction_matrix(s, Z, fx, fy, cx, cy);
    
    % 计算控制输入
    tau = lambda * pinv(Ls) * F_inv * e;
    
    % 打印控制输入
    fprintf('Time: %.2f s, Control Input (tau): [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n', t(k), tau(1), tau(2), tau(3), tau(4), tau(5), tau(6));
    
    % 存储控制输入
    tau_history(:, k) = tau;
    
    % 更新相机位置和姿态
    [R_camera, t_camera] = update_camera_pose(R_camera, t_camera, tau, Ts);
    
    % 打印相机位置
    fprintf('Camera Position: [%.4f, %.4f, %.4f]\n', t_camera(1), t_camera(2), t_camera(3));
    
    % 重新计算三维点在相机坐标系下的投影
    s_next = project_points_to_image(P_world, K, R_camera, t_camera, image_width, image_height);

    noise = 2 * (2 * rand(size(s_next)) - 1);
    s_next = s_next + noise;
    
    % 更新深度 Z (相机到特征点中心的距离)
    P_center = mean(P_world, 2); % 特征点中心 (3x1 向量)
    Z = norm(R_camera * P_center + t_camera); % 相机到特征点中心的距离
    Z_history(k+1) = Z; % 存储深度
    
    % 存储下一个视觉特征
    s_history(:, k+1) = s_next;
    
    % 更新三维可视化
    set(h_camera, 'XData', t_camera(1), 'YData', t_camera(2), 'ZData', t_camera(3));
    
    % 更新相机轴
    set(h_x_axis, 'XData', [t_camera(1), t_camera(1) + axis_length * R_camera(1, 1)], ...
                 'YData', [t_camera(2), t_camera(2) + axis_length * R_camera(2, 1)], ...
                 'ZData', [t_camera(3), t_camera(3) + axis_length * R_camera(3, 1)]);
    set(h_y_axis, 'XData', [t_camera(1), t_camera(1) + axis_length * R_camera(1, 2)], ...
                 'YData', [t_camera(2), t_camera(2) + axis_length * R_camera(2, 2)], ...
                 'ZData', [t_camera(3), t_camera(3) + axis_length * R_camera(3, 2)]);
    set(h_z_axis, 'XData', [t_camera(1), t_camera(1) + axis_length * R_camera(1, 3)], ...
                 'YData', [t_camera(2), t_camera(2) + axis_length * R_camera(2, 3)], ...
                 'ZData', [t_camera(3), t_camera(3) + axis_length * R_camera(3, 3)]);
    drawnow;
    
    % 更新图像平面可视化
    set(h_image_points, 'XData', s_next(1:2:end), 'YData', s_next(2:2:end));
    drawnow;
end

% 绘制视觉特征
figure; plot(t, s_history); xlabel('Time (s)'); ylabel('Visual Features');
title('Visual Features Evolution'); legend('u1', 'v1', 'u2', 'v2', 'u3', 'v3', 'u4', 'v4');

% 绘制控制输入
figure; plot(t, tau_history); xlabel('Time (s)'); ylabel('Control Input');
title('Control Input Evolution'); legend('tau1', 'tau2', 'tau3', 'tau4', 'tau5', 'tau6');

% 绘制深度 Z 的变化
figure; plot(t, Z_history); xlabel('Time (s)'); ylabel('Depth Z');
title('Depth Z Evolution');

% 交互矩阵计算函数
function Ls = interaction_matrix(s, Z, fx, fy, cx, cy)
    % s: 视觉特征向量 (2Nx1)，形式为 [u1; v1; u2; v2; ...; uN; vN]
    % Z: 特征点的深度 (相机到特征点的距离)
    % fx, fy: 相机焦距 (像素)
    % cx, cy: 图像中心坐标 (像素)
    
    % 提取 u 和 v 坐标
    u = s(1:2:end); % u 坐标
    v = s(2:2:end); % v 坐标
    
    % 将 u 和 v 转换为归一化的 x 和 y
    x = (u - cx) / fx; % 归一化 x 坐标
    y = (v - cy) / fy; % 归一化 y 坐标
    
    % 初始化交互矩阵
    Ls = [];
    
    % 计算每个特征点的交互矩阵
    for i = 1:length(x)
        Lsi = [-1/Z,     0, x(i)/Z, x(i)*y(i), -(1+x(i)^2), y(i);
                  0, -1/Z, y(i)/Z, 1+y(i)^2,    -x(i)*y(i), -x(i)];
        Ls = [Ls; Lsi]; % 将每个特征点的交互矩阵堆叠起来
    end
end

function output_s = project_points_to_image(P_world, K, R, t, image_width, image_height)
    % P_world: 3xN 矩阵，表示N个三维点 (世界坐标系)
    % K: 相机内参矩阵
    % R: 相机旋转矩阵
    % t: 相机平移向量
    % image_width: 图像宽度 (像素)
    % image_height: 图像高度 (像素)
    
    % 将三维点转换为齐次坐标
    P_world_homogeneous = [P_world; ones(1, size(P_world, 2))];

    R_inv = R'; % 旋转矩阵的逆矩阵
    t_inv = -R_inv * t; % 平移向量的逆变换
    
    P_image = [R_inv, t_inv] * P_world_homogeneous;
    % 归一化坐标
    P_image_Normalization = P_image(:, :) ./ P_image(3, :); % 2xN 矩阵

    s_image = K * P_image_Normalization;
    
    % 检查特征点是否在图像平面内
    valid_indices = (s_image(1, :) >= 0) & (s_image(1, :) <= image_width) & ...
                    (s_image(2, :) >= 0) & (s_image(2, :) <= image_height);
    
    % 只保留在图像平面内的点
    s_image_valid = s_image(:, valid_indices);
    s = s_image_valid(1:2, :);
    
    % 将有效的特征点坐标转换为向量形式
    output_s = s(:); % 将2xM矩阵转换为2Mx1向量 (M 是有效点的数量)

    % 如果没有有效的特征点，返回空数组
    if isempty(output_s)
        warning('No feature points are within the image plane.');
    end
end

function [R_new, t_new] = update_camera_pose(R, t, tau, Ts)
    % 提取控制输入
    v = tau(1:3); % 平移速度
    w = tau(4:6); % 旋转速度
    
    % 构造速度变换矩阵
    V = [skew(w), v; 0, 0, 0, 0]; % 4x4 速度变换矩阵
    
    % 使用指数映射计算增量齐次变换矩阵
    delta_T = expm(V * Ts); % 4x4 齐次变换矩阵
    
    % 当前齐次变换矩阵
    T = [R, t; 0, 0, 0, 1]; % 4x4 齐次变换矩阵
    
    % 更新齐次变换矩阵
    T_new = T * delta_T;
    
    % 提取更新后的旋转矩阵和位置
    R_new = T_new(1:3, 1:3); % 3x3 旋转矩阵
    t_new = T_new(1:3, 4);   % 3x1 平移向量
end

function S = skew(w)
    % 构造旋转向量的斜对称矩阵
    S = [0, -w(3), w(2);
         w(3), 0, -w(1);
         -w(2), w(1), 0];
end