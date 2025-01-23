% 清空工作区
clear; clc;
vis_pose = true;
vis_predict = false;

% 相机内参
fx = 300; % x 轴方向的焦距 (像素)
fy = 300; % y 轴方向的焦距 (像素)
cx = 320.5; % 图像中心 x 坐标 (像素)
cy = 240.5; % 图像中心 y 坐标 (像素)
K = [fx, 0, cx; 0, fy, cy; 0, 0, 1]; % 相机内参矩阵

% 图像分辨率
image_width = 640; % 图像宽度 (像素)
image_height = 480; % 图像高度 (像素)

% 定义特征点的三维位置 (方形，边长 20cm)
P_world = [0, 0.1, 0; 0, -0.1, 0; 0, 0.1, 0.2; 0, -0.1, 0.2]'; % 世界坐标系下的三维点 (3x4 矩阵)

% 定义相机初始位置和姿态
R_camera = [0, 0, -1; 1, 0, 0; 0, 1, 0];
t_camera = [2; 1; 0.1];
R_camera0 = R_camera;
t_camera0 = t_camera;
theta_y = deg2rad(0);  % 绕 y 轴旋转角度
R_y = [cos(theta_y), 0, sin(theta_y); 0, 1, 0; -sin(theta_y), 0, cos(theta_y)]; % 绕 y 轴旋转
R_camera = R_camera*R_y; % 组合旋转

% 定义目标相机位置和姿态
R_camera_d = [0, 0, -1; 1, 0, 0; 0, 1, 0];
t_camera_d = [0.3; 0; 0.1];

% 仿真参数
Np = 5;   % 预测时域
diag_values = [1, 1, 1, 1, 1, 1, 1, 1];
Q = diag(diag_values);

% 初始视觉特征 (将三维点投影到图像平面)
s0 = project_points_to_image(P_world, K, R_camera, t_camera, image_width, image_height);
s_star = project_points_to_image(P_world, K, R_camera_d, t_camera_d, image_width, image_height); % 参考视觉特征 (目标位置)

% 初始深度 Z (相机到特征点中心的距离)
o_T_p = [eye(3,3), mean(P_world,2);
    zeros(1,3), 1];
p_T_o = inv(o_T_p);

p_T_cam_d = p_T_o * [t_camera_d; 1];
p_T_cam = p_T_o * [t_camera; 1];

Z_star = norm(p_T_cam_d(1:3));
Z = norm(p_T_cam(1:3)); % 相机到特征点中心的距离
Z_history(1) = Z; % 存储初始深度

% 初始化三维可视化
fig_3d = figure('Name', '3D Camera and Feature Points');
set(fig_3d, 'Position', [100, 500, 600, 500]); % 设置窗口位置和大小
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

[t_camera_curve, R_camera_curve] = generateSmoothCurve(t_camera, t_camera_d, R_camera, R_camera_d);

s_ref = project_and_visualize_curve(P_world, K, R_camera_curve, t_camera_curve, image_width, image_height, s_star, Q);

function [t_camera_curve, R_camera_curve] = generateSmoothCurve(t_camera, t_camera_d, R_camera, R_camera_d)

    point1 = t_camera(1:2)';      % 第一个点的 xy 值 [x1, y1]
    point2 = t_camera_d(1:2)';    % 第二个点的 xy 值 [x2, y2]

    % 计算中点
    midpoint = (point1 + point2) / 2; % 中点 [x_mid, y_mid]
    points = [point1; midpoint; point2];

    % 从旋转矩阵中提取方向向量
    % 假设旋转矩阵的第一列是方向向量
    direction_start = R_camera(:, 1)'; % 取旋转矩阵的第一列作为方向向量
    direction_end = R_camera_d(:, 1)'; % 取旋转矩阵的第一列作为方向向量

    % 计算斜率（dy/dx）
    % 如果分母为 0，斜率设为一个大数（表示接近垂直的方向）
    if direction_start(2) == 0
        slope_start = 1e6; % 如果分母为 0，斜率设为一个大数
    else
        slope_start = direction_start(1) / direction_start(2); % 起点的斜率
    end

    if direction_end(2) == 0
        slope_end = 1e6; % 如果分母为 0，斜率设为一个大数
    else
        slope_end = direction_end(1) / direction_end(2); % 终点的斜率
    end

    % 检查斜率是否为 Inf 或 NaN
    if isinf(slope_start) || isnan(slope_start)
        slope_start = 1e6; % 替换为一个大数
    end
    if isinf(slope_end) || isnan(slope_end)
        slope_end = 1e6; % 替换为一个大数
    end

    % 提取 x 和 y 坐标
    x = points(:, 1);
    y = points(:, 2);

    % 生成更多的点来绘制平滑曲线
    xx = linspace(min(x), max(x), 100);

    % 使用 spline 函数生成曲线
    yy = spline(x, [slope_start; y; slope_end], xx);

    zz = 0.1*ones(1, 100); 

    % 绘制结果
    figure;
    plot(xx, yy, '-', x, y, 'o');
    legend('curve', 'point');
    xlabel('x');
    ylabel('y');
    title('curve');
    grid on;

    % 提取曲线上的点和朝向
    t_camera_curve = [xx; yy; zz]'; % 曲线上的点 [x, y, z]
    num_points = length(xx);

    % 初始化 R_camera_curve
    R_camera_curve = zeros(3, 3, num_points);

    % 计算每个点的朝向
    for i = 1:num_points
        % 计算曲线的导数（切线方向）
        if i == 1
            % 第一个点的导数使用前向差分
            dx = xx(i+1) - xx(i);
            dy = yy(i+1) - yy(i);
        elseif i == num_points
            % 最后一个点的导数使用后向差分
            dx = xx(i) - xx(i-1);
            dy = yy(i) - yy(i-1);
        else
            % 中间点的导数使用中心差分
            dx = xx(i+1) - xx(i-1);
            dy = yy(i+1) - yy(i-1);
        end

        % 归一化切线方向
        tangent = [dx; dy; 0]; % 将 2D 向量扩展为 3D 向量
        tangent = tangent / norm(tangent);

        % 构造旋转矩阵
        % y 轴垂直于平面（即垂直于 xy 平面的方向）
        y_axis = [0; 0; 1]; % y 轴方向
        x_axis = cross(tangent, y_axis); % x 轴方向
        x_axis = -x_axis / norm(x_axis); % 归一化
        z_axis = -tangent; % z 轴方向

        % 构造旋转矩阵
        R_camera_curve(:, :, i) = [x_axis, y_axis, z_axis];
    end

    % 可视化 R_camera_curve 的三个轴
    figure;
    hold on;
    axis equal;
    grid on;
    xlabel('x');
    ylabel('y');
    zlabel('z');
    title('axis');

    % 绘制曲线
    plot3(t_camera_curve(:, 1), t_camera_curve(:, 2), t_camera_curve(:, 3), 'k-', 'LineWidth', 1);

    % 绘制每个点的三个轴
    for i = 1:10:num_points % 每隔 10 个点绘制一次，避免过于密集
        % 提取当前点的位置
        point = [t_camera_curve(i, 1), t_camera_curve(i, 2), t_camera_curve(i, 3)];

        % 提取旋转矩阵的三个轴
        x_axis = R_camera_curve(:, 1, i);
        y_axis = R_camera_curve(:, 2, i);
        z_axis = R_camera_curve(:, 3, i);

        % 绘制 x 轴（红色）
        quiver3(point(1), point(2), point(3), x_axis(1), x_axis(2), x_axis(3), 'r', 'LineWidth', 1);

        % 绘制 y 轴（绿色）
        quiver3(point(1), point(2), point(3), y_axis(1), y_axis(2), y_axis(3), 'g', 'LineWidth', 1);

        % 绘制 z 轴（蓝色）
        quiver3(point(1), point(2), point(3), z_axis(1), z_axis(2), z_axis(3), 'b', 'LineWidth', 1);
    end

    % 设置图例
    legend('curve', 'x', 'y', 'z');
    hold off;
end

function projected_points = project_and_visualize_curve(P_world, K, R_camera_curve, t_camera_curve, image_width, image_height, s_star, Q)
    projected_points = [];
    J_history = [];

    % 遍历每个点并投影到图像平面
    for i = 1:size(t_camera_curve, 1)
        % 提取当前点的位置和旋转矩阵
        t_current = t_camera_curve(i, :)'; % 提取当前点的位置 (3x1 向量)
        R_current = R_camera_curve(:, :, i); % 提取当前点的旋转矩阵 (3x3 矩阵)

        % 将点投影到图像平面
        s = project_points_to_image(P_world, K, R_current, t_current, image_width, image_height);

        e = s - s_star;
        J = e' * Q * e;

        % 如果投影点有效，则保存
        if ~isempty(s)
            projected_points = [projected_points; s']; % 保存为 [x1, y1, x2, y2, ...]
            J_history = [J_history; J];
        end
    end

    % 可视化投影后的点序列
    figure;
    hold on;
    axis equal;
    grid on;
    xlabel('u (pixels)');
    ylabel('v (pixels)');
    title('Feature Points in Image Plane');
    xlim([0, image_width]);
    ylim([0, image_height]);

    % 绘制视野边缘
    corners = [
        0, 0;
        image_width, 0;
        image_width, image_height;
        0, image_height;
    ];
    plot(corners([1:end, 1], 1), corners([1:end, 1], 2), 'k--', 'LineWidth', 1.5);

    % 绘制初始特征点投影
    s0 = project_points_to_image(P_world, K, R_camera_curve(:, :, 1), t_camera_curve(1, :)', image_width, image_height);
    plot(s0(1:2:end), s0(2:2:end), 'bo', 'MarkerSize', 10, 'LineWidth', 2);

    % 绘制目标特征点投影
    s_star = project_points_to_image(P_world, K, R_camera_curve(:, :, end), t_camera_curve(end, :)', image_width, image_height);
    plot(s_star(1:2:end), s_star(2:2:end), 'rx', 'MarkerSize', 10, 'LineWidth', 2);

    % 绘制投影后的点序列
    plot(projected_points(:, 1:2:end), projected_points(:, 2:2:end), 'go-', 'LineWidth', 2, 'MarkerSize', 10);

    % 设置图例
    legend('FOV Boundary', 'Initial Features', 'Target Features', 'Projected Curve');
    hold off;

    % 可视化 J 的变化曲线
    figure;
    plot(J_history, 'b-o', 'LineWidth', 2);
    grid on;
    xlabel('Iteration');
    ylabel('J (Error Squared)');
    title('Error Squared (J) over Iterations');
end

% 三维点到二维图像平面的投影
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