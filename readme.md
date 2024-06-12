# 流形优化姿态的MATLAB实现

## Acknowledgement

**Special thanks to [Wang Yimu's blog](https://wang-yimu.com/) for providing open-source content and inspiration, which made this project possible.**

该项目是对[Introduction to Optimization on Manifolds/Lie-groups](https://wang-yimu.com/introduction-to-optimization-on-manifolds/)博客中Python代码的MATLAB实现。本文将介绍在李群或流形上优化姿态的优势，并通过一个具体案例展示如何在MATLAB中实现流形优化姿态。

## 项目背景

李群和李代数提供了一种优雅的方式来处理旋转和变换等几何问题。在很多优化问题中，使用李群上的参数化可以让问题的求解更加快速和简洁。

## 目录结构

```
.
├── README.md
├── optimization_on_SO3.m
├── wedge.m
├── expm_wedge.m
├── vee.m
└── vee_logm.m
```

## 文件说明

- **README.md**：项目说明文档。
- **optimization_on_SO3.m**：主要的优化函数实现，包含一个优化的具体示例。
- **wedge.m**：实现向量到反对称矩阵的转换。
- **expm_wedge.m**：实现指数映射，即将旋转向量转换为旋转矩阵。
- **vee.m**：实现反对称矩阵到向量的转换。
- **vee_logm.m**：实现对数映射，即将旋转矩阵转换为旋转向量。

## 旋转向量和旋转矩阵

旋转向量 $\boldsymbol{\omega} \in \mathbb{R}^3$ 可以表示一个三维旋转。将当前坐标系围绕单位向量 $\frac{\boldsymbol{\omega}}{||\boldsymbol{\omega}||}$ 旋转 $||\boldsymbol{\omega}||$ 弧度。每个旋转矩阵都可以表示成一个旋转向量，转换方式为：

- 指数映射（Exponential map）： $\boldsymbol{R}=\text{exp}(\boldsymbol{\omega}^{\wedge})$
- 对数映射（Log map）： $\boldsymbol{\omega}=\text{Log}(\boldsymbol{R})$

## 目标函数

在 $SO(3)$ 上定义目标函数：
$$\text{cost}(\boldsymbol{R}) = ||{\text{Log}\left(\boldsymbol{R}_{\text{target}}^{\top}\boldsymbol{R}\right)}||^2$$

转换为旋转向量形式：
$$\text{cost}(\boldsymbol{\omega}) = \text{cost}(\boldsymbol{R}(\boldsymbol{\omega}))$$
其中 $\boldsymbol{R}(\boldsymbol{\omega}) = \text{Exp}(\boldsymbol{\omega}), \boldsymbol{R}(\boldsymbol{\omega}) \in SO(3)$。

## 优化算法

高斯-牛顿法被用于解决非线性最小二乘问题。主要步骤包括：
1. 初始化参数。
2. 计算残差和雅可比矩阵。
3. 解法向方程更新参数。
4. 检查收敛条件。

详见代码文件 **optimization_on_SO3.m**。

## 函数说明

### wedge.m
将向量 $\boldsymbol{v} = [v_1, v_2, v_3]^\top$ 转换为反对称矩阵: 

$${\boldsymbol{v}^{\wedge}}={\begin{bmatrix} 0 & -v_3 & v_2 
\\\\ v_3 & 0 & -v_1 
\\\\ -v_2 & v_1 & 0 \end{bmatrix}}$$

### expm_wedge.m
使用 MATLAB 内置函数 `expm()` 实现旋转向量到旋转矩阵的转换：
$$\text{Exp}(\boldsymbol{\omega}) = \text{exp}(\boldsymbol{\omega}^\wedge)$$

### vee.m
将反对称矩阵转换为向量：
$$\boldsymbol{R}^{\vee} = [v_1, v_2, v_3]^\top$$

### vee_logm.m
使用 MATLAB 内置函数 `logm()` 实现旋转矩阵到旋转向量的转换：
$$\text{Log}(\boldsymbol{R}) = (\text{log}(\boldsymbol{R}))^\vee$$

## 示例

在 `examples/example_optimization.m` 文件中，包含了一个完整的优化示例。该示例展示了如何定义目标旋转矩阵，并使用高斯-牛顿法进行优化，找到最优的旋转向量。

## 运行指南

1. 克隆或下载项目文件。
2. 在 MATLAB 中打开项目目录。
3. 运行 `examples/example_optimization.m` 文件，查看优化结果。

## 参考文献

- 博客：[Introduction to Optimization on Manifolds/Lie-groups](https://wang-yimu.com/introduction-to-optimization-on-manifolds/)

---

如有任何问题或建议，请联系项目维护者。

---

Enjoy your optimization on manifolds with MATLAB!

