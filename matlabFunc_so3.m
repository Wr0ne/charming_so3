%% 预处理
clc
close all
clear
matlabrc

%% 验证欧拉角与旋转矩阵之间的转换
disp('验证欧拉角与旋转矩阵之间的转换：');

disp('这里注意欧拉角的取值范围，pitch取[-90 +90], roll和yaw是[-180 +180]');
Target_Eulerd = [360*rand-180, 180*rand-90, 360*rand-180];
fprintf('目标欧拉角为(degree)：%5.2f, %5.2f, %5.2f. \n', Target_Eulerd(1), Target_Eulerd(2), Target_Eulerd(3));
Target_Euler = deg2rad(Target_Eulerd);  % 单位转换为弧度

Target_SO3 = so3(Target_Euler, 'eul', 'ZYX');
result_rotm = rotm(Target_SO3);
result_euler = rotm2eul(result_rotm, "ZYX");
result_eulerd = rad2deg(result_euler);
fprintf('结果欧拉角为(degree)：%5.2f, %5.2f, %5.2f. \n', result_eulerd(1), result_eulerd(2), result_eulerd(3));
result_error = Target_Eulerd - result_eulerd;
fprintf('error为(degree)：%8.7f, %8.7f, %8.7f. \n', result_error(1), result_error(2), result_error(3));

disp('验证完毕，清除过程变量...')
clear result_error result_eulerd result_euler result_rotm

fprintf('\n\n');

%% 验证欧拉角与四元数
disp('验证欧拉角与四元数之间的转换：');
fprintf('目标欧拉角为(degree)：%5.2f, %5.2f, %5.2f. \n', Target_Eulerd(1), Target_Eulerd(2), Target_Eulerd(3));

result_quat = eul2quat(Target_Euler);  % 实部在前四元数
norm_quat = norm(result_quat);  % 验证是否为单位四元数
result_euler = quat2eul(result_quat);
result_eulerd = rad2deg(result_euler);
fprintf('结果欧拉角为(degree)：%5.2f, %5.2f, %5.2f. \n', result_eulerd(1), result_eulerd(2), result_eulerd(3));
result_error = Target_Eulerd - result_eulerd;
fprintf('error为(degree)：%8.7f, %8.7f, %8.7f. \n', result_error(1), result_error(2), result_error(3));

disp('验证完毕，清除过程变量...')
clear result_error result_eulerd result_euler result_quat norm_quat

fprintf('\n\n');

%% 验证欧拉角与轴角
disp('验证欧拉角与轴角之间的转换：');
fprintf('目标欧拉角为(degree)：%5.2f, %5.2f, %5.2f. \n', Target_Eulerd(1), Target_Eulerd(2), Target_Eulerd(3));

result_axang = axang(Target_SO3);
result_quat = axang2quat(result_axang);
norm_quat = norm(result_quat);  % 验证是否为单位四元数
result_euler = quat2eul(result_quat);
result_eulerd = rad2deg(result_euler);
fprintf('结果欧拉角为(degree)：%5.2f, %5.2f, %5.2f. \n', result_eulerd(1), result_eulerd(2), result_eulerd(3));
result_error = Target_Eulerd - result_eulerd;
fprintf('error为(degree)：%8.7f, %8.7f, %8.7f. \n', result_error(1), result_error(2), result_error(3));

disp('验证完毕，清除过程变量...')
clear result_error result_eulerd result_euler result_quat norm_quat result_axang

fprintf('\n\n');

%% 验证逆旋转
disp('验证逆旋转：');
fprintf('目标欧拉角为(degree)：%5.2f, %5.2f, %5.2f. \n', Target_Eulerd(1), Target_Eulerd(2), Target_Eulerd(3));
% 旋转矩阵求逆
inv_SO3 = inv(Target_SO3);
% 欧拉角反转回去要注意沿旋转顺序反向，角度取负
inv_euler_SO3 = so3(-[Target_Euler(3) Target_Euler(2) Target_Euler(1)],"eul","XYZ");

% 验证矩阵求逆和欧拉角求逆两种方法是否一致
if abs(rad2deg(eul(inv_SO3)) - rad2deg(eul(inv_euler_SO3))) < 1e-8
    disp('矩阵求逆和欧拉角求逆两种方法一致');
    inv_eul = rad2deg(eul(inv_euler_SO3));
    fprintf('反旋的欧拉角为(degree)：%5.2f, %5.2f, %5.2f. (注意这里为ZYX顺序)\n', inv_eul(1), inv_eul(2), inv_eul(3));
end
% 验证欧拉角求逆方法
result_SO3 = inv_euler_SO3 * Target_SO3;
result_eul = rad2deg(eul(result_SO3));
fprintf('正反旋转后的欧拉角为(degree)：%8.7f, %8.7f, %8.7f. (注意这里为ZYX顺序)\n', result_eul(1), result_eul(2), result_eul(3));

disp('验证完毕，清除过程变量...')
clear inv_eul inv_euler_SO3 inv_SO3 result_eul result_SO3

fprintf('\n\n');

%% 验证轴角与DCM
disp('验证轴角与DCM之间的转换：');

result_axang = axang(Target_SO3);  % 轴角

% 验证axang转为rotm的两种方式是否一致
% 指数映射：
axang_vec = result_axang(1:3);  % 轴
axang_scalor = result_axang(4);  % 角
axang_vec_wedge = wedge(axang_vec .* axang_scalor);  % hat
axang2rotm_by_expm = expm(axang_vec_wedge);
% matlab函数：
axang2rotm_by_matlab = axang2rotm(result_axang);
% 检查两种方式的结果
if max(max(abs(axang2rotm_by_expm - axang2rotm_by_matlab))) > 1e-10
    error('axang转为rotm的两种方式结果不一致');
else
    disp('axang转为rotm的两种方式结果一致');
end

% 验证rotm转为axang的对数映射
Target_SO3_vee = vee(logm(rotm(Target_SO3)));
target_axang = axang_vec .* axang_scalor;
% 检查rotm转为axang的对数映射
if max(max(abs(Target_SO3_vee - target_axang'))) > 1e-10
    error('rotm转为axang的对数映射结果不一致');
else
    disp('rotm转为axang的对数映射结果一致');
end

disp('验证完毕，清除过程变量...')
clear axang2rotm_by_expm axang2rotm_by_matlab axang_scalor axang_vec axang_vec_wedge result_axang target_axang Target_SO3_vee

fprintf('\n\n');

