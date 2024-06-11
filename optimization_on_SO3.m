%% 预处理
clc
close all
clear
matlabrc

%% 生成随机的目标值
Target_Eulerd = [360*rand-180, 180*rand-90, 360*rand-180];
% Target_Eulerd = [-40, 20, 30];
fprintf('目标欧拉角为(degree)：%5.2f, %5.2f, %5.2f. \n', Target_Eulerd(1), Target_Eulerd(2), Target_Eulerd(3));
Target_Euler = deg2rad(Target_Eulerd);  % 单位转换为弧度

Target_SO3 = so3(Target_Euler, 'eul', 'ZYX');
Target_axang = axang(Target_SO3);  % 轴角

%% 优化初值
% Ini_Eulerd = [360*rand-180, 180*rand-90, 360*rand-180];
Ini_Eulerd = [0 0 0];  % TODO: 为什么初值为0可以一步解决，初值随意给的时候会优化失败？

fprintf('优化初值为(degree)：%5.2f, %5.2f, %5.2f. \n', Ini_Eulerd(1), Ini_Eulerd(2), Ini_Eulerd(3));
Ini_Euler = deg2rad(Ini_Eulerd);  % 单位转换为弧度
Ini_SO3 = so3(Ini_Euler, 'eul', 'ZYX');
Ini_axang = axang(Ini_SO3);  % 轴角

%% 一步优化
disp('--------------------------------------------------------------------------')
[result, iter] = gauss_newton(Target_SO3, Ini_SO3);
result = rad2deg(eul(result));
fprintf('优化结果值为(degree)：%5.2f, %5.2f, %5.2f. \n', result(1), result(2), result(3));
fprintf('迭代次数为：%d. \n', iter);
disp('--------------------------------------------------------------------------')
error = result - Target_Eulerd;
error = error * 3600;  % 度转角秒
fprintf('优化误差为(角秒)：%f, %f, %f. \n', error(1), error(2), error(3));

%% gauss_newton
function [x, iter] = gauss_newton(Target_SO3, Ini_SO3)
    % Gauss-Newton 方法用于解决非线性最小二乘问题
    %
    % fun     - 残差函数句柄
    % jac     - 残差雅可比矩阵的句柄
    % x0      - 参数的初始猜测
    % tol     - 收敛容差
    % max_iter - 最大迭代次数
    %
    % 返回值:
    % x       - 解向量
    % iter    - 执行的迭代次数
    
    % 初始化变量
    tol = 1e-4;
    max_iter = 100;

    x = Ini_SO3;
    iter = 0;

    while iter < max_iter
        [x, delta] = GN_single(Target_SO3, x);
        
        % 检查收敛条件
        if norm(delta) < tol
            break;
        end
        
        % 增加迭代计数
        iter = iter + 1;
    end
end

%% GN_single
function [ans_SO3, dw] = GN_single(Target_SO3, Ini_SO3)
    % 在当前猜测值处计算残差和雅可比矩阵
    residual = residual_function(rotm(Target_SO3), rotm(Ini_SO3));
    %cost = cost_function(residual);
    %fprintf('cost为：%f. \n', cost);
    jocabian = numerical_jacobian(rotm(Ini_SO3));

    % 解方程 J'*J*delta = -J'*r
    dw = (jocabian' * jocabian)\(-jocabian' * residual);

    % 更新参数
    ans_DCM = expm_wedge(dw) * rotm(Ini_SO3);
    ans_SO3 = so3(ans_DCM);
    % result = rad2deg(eul(Ini_SO3));
    %fprintf('优化结果值为(degree)：%5.2f, %5.2f, %5.2f. \n', result(1), result(2), result(3));
    %disp('--------------------------------------------------------------------------')
end
%% residual function
% A residual function r(R) = log(R_target.T @ R)
function r_R = residual_function(R_target, R)
    r_R = vee_logm(R_target' * R);
end

%% cost function
% cost(R) = ||residual(R)||^2
function cost = cost_function(r_R)
    cost = r_R' * r_R;
end

%% numerical_jacobian
% dr/dw = (r(w + dw) - r(w - dw)) / (2*dw)
function Jacobian = numerical_jacobian(R)
%dr = jacobian @ dw
    Jacobian = zeros(3,3);

    DELTA = 1e-5;  % maybe 1e-8 for fwd diff. 1e-5 for center diff? (considered by MIT 18.S096 lesson 4)
    for i = 1:3
        dw_plus = [0 0 0];
        dw_plus(i) = DELTA;
        R_plus = R * expm_wedge(dw_plus);

        dw_minus = [0 0 0];
        dw_minus(i) = -DELTA;
        R_minus = R * expm_wedge(dw_minus);

        J_i = (residual_function(R, R_plus) - residual_function(R, R_minus)) / (2 * DELTA);

        Jacobian(:, i) = J_i;
    end
end