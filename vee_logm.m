function vector = vee_logm(rotm)
    % VEE_LOGM 计算旋转矩阵的对数并将其转换为向量形式
    %
    % 输入:
    % rotm - 3x3 旋转矩阵
    %
    % 输出:
    % vector - 3x1 对应的旋转向量

    % 检查 rotm 是否为有效的旋转矩阵
    assert(all(all(abs(rotm' * rotm - eye(3)) < 1e-10)), 'R is not a valid rotation matrix');

    % 检查 rotm 是否为对称矩阵
    if all(all(abs(rotm - rotm') < 1e-10))
        % 打破对称性
        % 例如，从0到π的旋转，可以顺时针或逆时针进行。
        vector = vee_logm(rotm * expm_wedge([1e-8; 0; 0]));
    else
        % 计算旋转矩阵的对数并转换为向量形式
        vector = vee(logm(rotm));
    end
end
