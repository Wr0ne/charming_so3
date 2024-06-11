%% vectorToSkewSymmetricMatrix: 将一个3维向量转换为3x3反对称矩阵
function skewMatrix = wedge(vector)
% vectorToSkewSymmetricMatrix 将一个3维向量转换为3x3反对称矩阵
%
% 输入:
%   vector - 3维列向量 [v1; v2; v3]
%
% 输出:
%   skewMatrix - 3x3反对称矩阵
    
    % 检查输入向量是否为3维
    if length(vector) ~= 3
        error('输入必须为3维向量');
    end
    
    % 提取向量的分量
    v1 = vector(1);
    v2 = vector(2);
    v3 = vector(3);
    
    % 生成3x3反对称矩阵
    skewMatrix = [  0   -v3   v2;
                   v3    0   -v1;
                  -v2   v1    0 ];
end