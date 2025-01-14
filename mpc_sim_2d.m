% 清空工作区
clear; clc;
vis_pose = false;
vis_predict = true;

% 相机内参
fx = 400; % x 轴方向的焦距 (像素)
fy = 400; % y 轴方向的焦距 (像素)
cx = 320.5; % 图像中心 x 坐标 (像素)
cy = 240.5; % 图像中心 y 坐标 (像素)
K = [fx, 0, cx; 0, fy, cy; 0, 0, 1]; % 相机内参矩阵

% 图像分辨率
image_width = 640; % 图像宽度 (像素)
image_height = 480; % 图像高度 (像素)

F = zeros(8, 8); % 初始化 8x8 的零矩阵

% 设置奇数行对角为 fx
F(1:2:end, 1:2:end) = diag(fx * ones(4, 1)); % 奇数行对角
% 设置偶数行对角为 fy
F(2:2:end, 2:2:end) = diag(fy * ones(4, 1)); % 偶数行对角

% 定义特征点的三维位置 (方形，边长 20cm)
P_world = [0.1, 0, 0; -0.1, 0, 0; 0.1, 0, 0.2; -0.1, 0, 0.2]'; % 世界坐标系下的三维点 (3x4 矩阵)

% 定义相机初始位置和姿态
R_camera = [1, 0, 0; 0, 0, -1; 0, 1, 0];
t_camera = [0.5; 2; 0.1];
R_camera0 = R_camera;
t_camera0 = t_camera;
theta_y = deg2rad(-20);  % 绕 y 轴旋转角度
R_y = [cos(theta_y), 0, sin(theta_y); 0, 1, 0; -sin(theta_y), 0, cos(theta_y)]; % 绕 y 轴旋转
R_camera = R_camera*R_y; % 组合旋转

% 定义目标相机位置和姿态
R_camera_d = [1, 0, 0; 0, 0, -1; 0, 1, 0];
t_camera_d = [0; 0.3; 0.1];

% 仿真参数
Np = 5;   % 预测时域
Q = eye(8); % 加权矩阵 (8x8单位矩阵)
R_weight = 1000; % 正则化权重
Tf = 2; % 仿真总时间 (10秒)
Ts = 0.04; % 采样时间 (40ms)
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
Z_star = t_camera_d(3,1);
Z = norm(R_camera * P_center + t_camera); % 相机到特征点中心的距离
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

% 初始化图像平面可视化
fig_image = figure('Name', 'Feature Points in Image Plane');
set(fig_image, 'Position', [800, 500, 600, 500]); % 设置窗口位置和大小
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

% 初始化预测历史可视化
fig_handle = figure('Name', 'Predicted Feature Points History');
set(fig_handle, 'Position', [1500, 500, 600, 500]); % 设置窗口位置和大小

t_camera_history = zeros(3, N); % 相机位置历史 (3xN 矩阵)
R_history = zeros(3, 3, N); % 相机旋转矩阵历史 (3x3xN 矩阵)

% 主仿真循环
for k = 1:N-1
    % 当前视觉特征
    s = s_history(:, k);

    if k > 1
        tau_now = tau_history(:, k-1);
    else
        tau_now = 0.1 * ones(6, 1);
    end
    
    % MPC控制器计算控制输入
    tau = mpc_controller(s, s_star, Z, Np, Q, R_weight, Ts, K, F, image_width, image_height, tau_now, fig_handle, vis_predict);
    
    % 存储控制输入
    tau_history(:, k) = tau;
    
    % 更新相机位置和姿态
    [R_camera, t_camera] = update_camera_pose(R_camera, t_camera, tau, Ts);
    
    % 重新计算三维点在相机坐标系下的投影
    s_next = project_points_to_image(P_world, K, R_camera, t_camera, image_width, image_height);
    
    % 更新深度 Z (相机到特征点中心的距离)
    P_center = mean(P_world, 2); % 特征点中心 (3x1 向量)

    
    Z = norm(R_camera * P_center + t_camera); % 相机到特征点中心的距离
    % Z = t_camera_d(3,1);
    Z_history(k+1) = Z; % 存储深度
    
    % 存储下一个视觉特征
    s_history(:, k+1) = s_next;

    if vis_pose
        % 更新三维可视化
        set(h_camera, 'XData', t_camera(1), 'YData', t_camera(2), 'ZData', t_camera(3));
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
        
        % 为特征点添加标号
        drawnow;
    end

    % 打印控制输入
    fprintf('Time: %.2f s, Control Input (tau): [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f], R_weight: %.4f\n', t(k), tau(1), tau(2), tau(3), tau(4), tau(5), tau(6), R_weight);

    % 打印相机位置
    fprintf('Camera Position: [%.4f, %.4f, %.4f], Z: %.4f \n', t_camera(1), t_camera(2), t_camera(3), Z);

    t_camera_history(:, k+1) = t_camera; % 存储最后一帧的相机位置
    R_history(:, :, k+1) = R_camera; % 存储最后一帧的旋转矩阵

end

plot_simulation_results(t, s_history, s_star, Z_history, Z_star, tau_history, t_camera_history, t_camera_d, R_history, R_camera_d, t_camera0, R_camera0)

% 定义交互矩阵
function Ls = interaction_matrix(s, Z, K)
    % s: 视觉特征向量 (2Nx1)，形式为 [u1; v1; u2; v2; ...; uN; vN]
    % Z: 特征点的深度 (相机到特征点的距离)
    % K: 相机内参矩阵 (3x3)
    
    % 提取相机内参
    fx = K(1, 1); % 焦距 fx
    fy = K(2, 2); % 焦距 fy
    cx = K(1, 3); % 图像中心 cx
    cy = K(2, 3); % 图像中心 cy
    
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

function tau = mpc_controller(s, s_star, Z, Np, Q, R_weight, Ts, K, F, image_width, image_height, tau_now, fig_handle, vis_predict)

    % 定义成本函数
    fun = @(tau_seq) cost_function(tau_seq, s, s_star, Z, Np, Q, R_weight, Ts, K, F);

    % 定义视野约束
    nonlcon = @(tau_seq) fov_constraints(tau_seq, s, Z, Np, Ts, K, F, image_width, image_height);

    % 优化选项
    options_headless = optimoptions('fmincon', ...
                       'Display', 'off', ...
                       'Algorithm', 'sqp', ...
                       'MaxFunctionEvaluations', 5000);

    options_vis = optimoptions('fmincon', ...
                       'Display', 'iter', ...
                       'Algorithm', 'sqp', ...
                       'MaxFunctionEvaluations', 5000, ...
                       'PlotFcn', {@optimplotx, @optimplotfval});

    if vis_predict
        options = options_vis;
    else
        options = options_headless;
    end

    % 初始猜测 (6*Nc 维向量)
    tau_seq0 = repelem(tau_now, Np, 1);

    % 调用 fmincon 求解
    tau_seq = fmincon(fun, tau_seq0, [], [], [], [], [], [], nonlcon, options);

    % 提取第一个控制输入
    tau = tau_seq(1:6);

    [~, s_pred_history] = cost_function(tau_seq, s, s_star, Z, Np, Q, R_weight, Ts, K, F);

    if vis_predict
        plot_s_pred_history(s_pred_history, image_width, image_height, fig_handle);
    end
end


function [c, ceq] = fov_constraints(tau_seq, s, Z, Np, Ts, K, F, image_width, image_height)

    % 初始化约束
    c = []; % 不等式约束
    ceq = []; % 等式约束 (无)

    % 预测视觉特征
    s_pred = s; % 初始视觉特征
    for j = 1:Np
        % 计算交互矩阵
        Ls = interaction_matrix(s_pred, Z, K);

        % 提取当前控制输入 (每次取 6 个)
        tau = tau_seq((j-1)*6 + 1 : j*6); % 第 j 个控制输入

        % 更新视觉特征
        s_pred = s_pred + Ts * F * Ls * tau;

        % 提取特征点坐标 (u, v)
        u = s_pred(1:2:end); % u 坐标
        v = s_pred(2:2:end); % v 坐标

        % 添加视野约束 (确保特征点在图像边界内)
        c = [c; 
             -u + 0;                % u >= 10 (留出边界)
             u - (image_width - 0); % u <= image_width - 10
             -v + 0;                % v >= 10
             v - (image_height - 0)]; % v <= image_height - 10

        ceq = [ceq; 
               tau(1);  % vx = 0
               tau(2);  % vy = 0
               tau(4);  % wx = 0
               tau(6)]; % wz = 0
    end
end

function [J, s_pred_history] = cost_function(tau_seq, s, s_star, Z, Np, Q, R_weight, Ts, K, F)
    J1 = 0;
    J2 = 0;
    s_pred = s;

    % 初始化存储 s_pred 历史
    s_pred_history = zeros(size(s, 1), Np + 1); % 存储每次迭代的 s_pred
    s_pred_history(:, 1) = s; % 记录初始值

    for j = 1:Np
        % 计算交互矩阵
        Ls = interaction_matrix(s_pred, Z, K);

        % 提取当前控制输入
        tau = tau_seq((j-1)*6 + 1 : j*6); % 第 j 个控制输入

        % 更新视觉特征
        s_pred = s_pred + Ts * F * Ls * tau;

        % 记录当前 s_pred
        s_pred_history(:, j + 1) = s_pred;

        % 计算误差
        e = s_pred - s_star;

        % 累加成本
        J1 = J1 + e' * Q * e;
        J2 = J2 + R_weight * (tau' * tau);
    end

    J = J1 + J2;
    % fprintf('J: [%.4f, %.4f, %.4f]\n', J1, J2, J);
end

function plot_s_pred_history(s_pred_history, image_width, image_height, fig_handle)
    % fig_handle: 图形窗口的句柄，用于指定操作的图形窗口

    % 如果未提供 fig_handle，则创建一个新的图形窗口
    if nargin < 4 || isempty(fig_handle)
        fig_handle = figure; % 创建新的图形窗口
    end

    % 切换到指定的图形窗口
    figure(fig_handle);

    % 清除当前图形窗口的内容
    clf(fig_handle);

    % 绘制 s_pred 的二维散点图
    hold on;
    grid on;
    xlabel('u (pixels)');
    ylabel('v (pixels)');
    title('Feature Points in Image Plane (s\_pred)');
    axis equal;

    % 提取 u 和 v 坐标
    u = s_pred_history(1:2:end, :); % 所有特征点的 u 坐标
    v = s_pred_history(2:2:end, :); % 所有特征点的 v 坐标

    % 定义颜色映射（每个时间步长一个颜色）
    num_time_steps = size(u, 2); % 时间步长数量
    colors = lines(num_time_steps); % 使用 lines 颜色映射

    % 绘制散点图
    for t = 1:num_time_steps % 遍历每个时间步长
        % 提取当前时间步长的所有特征点坐标
        u_t = u(:, t); % 当前时间步长的 u 坐标
        v_t = v(:, t); % 当前时间步长的 v 坐标

        % 绘制当前时间步长的点
        if t == 1
            scatter(u_t, v_t, 50, colors(t, :), 'filled', 'DisplayName', 'Initial');
        else
            scatter(u_t, v_t, 50, colors(t, :), 'filled', 'DisplayName', ['Time ' num2str(t)]);
        end
    end

    % 添加图例
    legend('Location', 'best');

    % 设置坐标轴范围为图像分辨率范围
    xlim([0, image_width]);
    ylim([0, image_height]);

    % 绘制图像边界
    plot([0, image_width, image_width, 0, 0], [0, 0, image_height, image_height, 0], 'k--', 'LineWidth', 1.5, 'DisplayName', 'Image Boundary');

    % 强制刷新图形
    drawnow;
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

function plot_simulation_results(t, s_history, s_star, Z_history, Z_star, tau_history, t_camera_history, t_camera_d, R_history, R_camera_d, t_camera_initial, R_camera_initial)
    % 绘制视觉特征与期望值的对比
    figure;
    hold on;
    colors = lines(8); % 使用 lines 颜色映射，生成 8 种颜色（4 个点，每个点 2 个颜色）
    line_styles = {'-', '--'}; % 实线表示实际值，虚线表示期望值
    for i = 1:4
        % 绘制实际值 (u, v)
        plot(t, s_history(2*i-1, :), 'Color', colors(2*i-1, :), 'LineStyle', line_styles{1}, 'LineWidth', 1.5); % u
        plot(t, s_history(2*i, :), 'Color', colors(2*i, :), 'LineStyle', line_styles{1}, 'LineWidth', 1.5); % v

        % 绘制期望值 (u^*, v^*)
        plot(t, repmat(s_star(2*i-1), 1, length(t)), 'Color', colors(2*i-1, :), 'LineStyle', line_styles{2}, 'LineWidth', 1.5); % u^*
        plot(t, repmat(s_star(2*i), 1, length(t)), 'Color', colors(2*i, :), 'LineStyle', line_styles{2}, 'LineWidth', 1.5); % v^*
    end

    hold off;
    xlabel('Time (s)');
    ylabel('Visual Features');
    title('Visual Features Evolution');
    legend('u1', 'v1', 'u1^*', 'v1^*', ...
           'u2', 'v2', 'u2^*', 'v2^*', ...
           'u3', 'v3', 'u3^*', 'v3^*', ...
           'u4', 'v4', 'u4^*', 'v4^*');
    grid on;

    % 绘制深度 Z 的变化与期望值的对比
    figure;
    hold on;
    plot(t, Z_history, 'LineWidth', 1.5); % 绘制实际深度 Z
    plot(t, repmat(Z_star, 1, length(t)), '--', 'LineWidth', 1.5); % 绘制期望深度 Z
    hold off;
    xlabel('Time (s)');
    ylabel('Depth Z');
    title('Depth Z Evolution');
    legend('Z', 'Z^*');
    grid on;

    % 绘制控制输入
    figure;
    plot(t, tau_history);
    xlabel('Time (s)');
    ylabel('Control Input');
    title('Control Input Evolution');
    legend('tau1', 'tau2', 'tau3', 'tau4', 'tau5', 'tau6');
    grid on;

    % 绘制 t_camera 和 t_camera_d 的变化曲线
    figure;
    hold on;
    plot(t, t_camera_history(1, :), 'r', 'LineWidth', 1.5, 'DisplayName', 't_{camera,x}');
    plot(t, t_camera_history(2, :), 'g', 'LineWidth', 1.5, 'DisplayName', 't_{camera,y}');
    plot(t, repmat(t_camera_d(1), 1, length(t)), 'r--', 'LineWidth', 1.5, 'DisplayName', 't_{camera,d,x}');
    plot(t, repmat(t_camera_d(2), 1, length(t)), 'g--', 'LineWidth', 1.5, 'DisplayName', 't_{camera,d,y}');
    hold off;
    xlabel('Time (s)');
    ylabel('Camera Position');
    title('Camera Position Evolution');
    legend('Location', 'best');
    grid on;

    % 绘制初始位姿、期望位姿、最终位姿的三维图
    figure;
    hold on;
    axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('Initial, Desired, and Final Camera Pose');
    view(3);
    grid on;

    % 定义轴的长度
    axis_length = 0.2;

    % 绘制初始相机位置和姿态
    plot3(t_camera_initial(1), t_camera_initial(2), t_camera_initial(3), 'ko', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Initial Camera Position');
    plot3([t_camera_initial(1), t_camera_initial(1) + axis_length * R_camera_initial(1, 1)], ...
          [t_camera_initial(2), t_camera_initial(2) + axis_length * R_camera_initial(2, 1)], ...
          [t_camera_initial(3), t_camera_initial(3) + axis_length * R_camera_initial(3, 1)], 'k', 'LineWidth', 2, 'DisplayName', 'Initial X Axis');
    plot3([t_camera_initial(1), t_camera_initial(1) + axis_length * R_camera_initial(1, 2)], ...
          [t_camera_initial(2), t_camera_initial(2) + axis_length * R_camera_initial(2, 2)], ...
          [t_camera_initial(3), t_camera_initial(3) + axis_length * R_camera_initial(3, 2)], 'k', 'LineWidth', 2, 'DisplayName', 'Initial Y Axis');
    plot3([t_camera_initial(1), t_camera_initial(1) + axis_length * R_camera_initial(1, 3)], ...
          [t_camera_initial(2), t_camera_initial(2) + axis_length * R_camera_initial(2, 3)], ...
          [t_camera_initial(3), t_camera_initial(3) + axis_length * R_camera_initial(3, 3)], 'k', 'LineWidth', 2, 'DisplayName', 'Initial Z Axis');

    % 绘制期望相机位置和姿态
    plot3(t_camera_d(1), t_camera_d(2), t_camera_d(3), 'rx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Desired Camera Position');
    plot3([t_camera_d(1), t_camera_d(1) + axis_length * R_camera_d(1, 1)], ...
          [t_camera_d(2), t_camera_d(2) + axis_length * R_camera_d(2, 1)], ...
          [t_camera_d(3), t_camera_d(3) + axis_length * R_camera_d(3, 1)], 'r--', 'LineWidth', 2, 'DisplayName', 'Desired X Axis');
    plot3([t_camera_d(1), t_camera_d(1) + axis_length * R_camera_d(1, 2)], ...
          [t_camera_d(2), t_camera_d(2) + axis_length * R_camera_d(2, 2)], ...
          [t_camera_d(3), t_camera_d(3) + axis_length * R_camera_d(3, 2)], 'g--', 'LineWidth', 2, 'DisplayName', 'Desired Y Axis');
    plot3([t_camera_d(1), t_camera_d(1) + axis_length * R_camera_d(1, 3)], ...
          [t_camera_d(2), t_camera_d(2) + axis_length * R_camera_d(2, 3)], ...
          [t_camera_d(3), t_camera_d(3) + axis_length * R_camera_d(3, 3)], 'b--', 'LineWidth', 2, 'DisplayName', 'Desired Z Axis');

    % 绘制最终相机位置和姿态
    t_camera_final = t_camera_history(:, end); % 最终相机位置
    R_camera_final = R_history(:, :, end); % 最终相机旋转矩阵
    plot3(t_camera_final(1), t_camera_final(2), t_camera_final(3), 'bs', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Final Camera Position');
    plot3([t_camera_final(1), t_camera_final(1) + axis_length * R_camera_final(1, 1)], ...
          [t_camera_final(2), t_camera_final(2) + axis_length * R_camera_final(2, 1)], ...
          [t_camera_final(3), t_camera_final(3) + axis_length * R_camera_final(3, 1)], 'r', 'LineWidth', 2, 'DisplayName', 'Final X Axis');
    plot3([t_camera_final(1), t_camera_final(1) + axis_length * R_camera_final(1, 2)], ...
          [t_camera_final(2), t_camera_final(2) + axis_length * R_camera_final(2, 2)], ...
          [t_camera_final(3), t_camera_final(3) + axis_length * R_camera_final(3, 2)], 'g', 'LineWidth', 2, 'DisplayName', 'Final Y Axis');
    plot3([t_camera_final(1), t_camera_final(1) + axis_length * R_camera_final(1, 3)], ...
          [t_camera_final(2), t_camera_final(2) + axis_length * R_camera_final(2, 3)], ...
          [t_camera_final(3), t_camera_final(3) + axis_length * R_camera_final(3, 3)], 'b', 'LineWidth', 2, 'DisplayName', 'Final Z Axis');

    hold off;
    legend('Location', 'best');
end