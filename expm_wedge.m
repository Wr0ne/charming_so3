function rotm = expm_wedge(vector)
    % EXPM_WEDGE 计算给定向量对应的反对称矩阵的指数映射（旋转矩阵）
    %
    % 输入:
    % vector - 3x1 或 6x1 向量
    %          如果是 3x1 向量，则代表 SO(3) 的旋转部分
    %          如果是 6x1 向量，则代表 SE(3) 的旋转和平移部分
    %
    % 输出:
    % rotm - 3x3 旋转矩阵 或 4x4 刚体变换矩阵

    % 将输入向量转换为反对称矩阵
    skewMatrix = wedge(vector);

    % 计算反对称矩阵的指数映射（旋转矩阵）
    rotm = expm(skewMatrix);
end
