%% skewSymmetricMatrixToVector: 将一个3x3反对称矩阵转换为3维向量
function vector = vee(skewMatrix, tol)
    % skewSymmetricMatrixToVector 将一个3x3反对称矩阵转换为3维向量
    %
    % 输入:
    %   skewMatrix - 3x3反对称矩阵
    %   tol - 容差值（可选），用于检查反对称性
    %
    % 输出:
    %   vector - 3维列向量 [v1; v2; v3]
    
    % 检查输入矩阵是否为3x3矩阵
    [rows, cols] = size(skewMatrix);
    if rows ~= 3 || cols ~= 3
        error('输入矩阵必须为3x3矩阵');
    end
    
    % 如果没有指定容差值，使用默认值
    if nargin < 2
        tol = 1e-10;
    end
    
    % 检查输入矩阵是否为反对称矩阵（考虑容差值）
    if max(max(abs(skewMatrix + skewMatrix'))) > tol
        error('输入矩阵必须为反对称矩阵');
    end
    
    % 提取反对称矩阵中的分量
    v1 = (skewMatrix(3, 2) - skewMatrix(2, 3))/2;
    v2 = (skewMatrix(1, 3) - skewMatrix(3, 1))/2;
    v3 = (skewMatrix(2, 1) - skewMatrix(1, 2))/2;
    
    % 生成3维向量
    vector = [v1; v2; v3];
end