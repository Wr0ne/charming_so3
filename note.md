# 一个简单的使用流形优化姿态的例子

> 本文参考博客：[Introduction to Optimization on Manifolds/Lie-groups](https://wang-yimu.com/introduction-to-optimization-on-manifolds/)，是对博客中的python代码的MATLAB实现。

在阅读这篇博客前，我一直不明白李群李代数到底实际操作起来是如何用于位姿优化的。这篇文章，浅显易懂的阐述了一个流形优化的最简单案例。

下面对这个最简单的案例进行MATLAB的复现。

## 在李群、或流形上优化的优点

对于很多优化问题而言，在李群上参数化可以让优化问题的求解更快速简洁。

## 旋转参数

原博文中比较了**3×3 matrix + cost，yaw-pitch-roll，Axis-angle representation，Quaternions**四种旋转的参数化表达的优缺点，这里仅对后续优化采用的旋转向量形式简单叙述。

一个旋转向量可以代表一个三维旋转，我们将一个旋转向量记为：
$$
\boldsymbol{\omega} \in \boldsymbol{R}^3
$$
含义为：将当前坐标系围绕单位向量 ${\boldsymbol{\omega}}/{||\boldsymbol{\omega}||}$ 旋转 $||\boldsymbol{\omega}||$ 弧度。

每个旋转矩阵都可以表示成一个旋转向量。

对于旋转一个向量的情况，可以用 Rodrigues 公式将一个旋转向量转换为旋转矩阵，然后使用旋转矩阵的形式旋转向量。

Rodrigues 公式与指数映射（**the Exponential map**）是等价的。
$$
\boldsymbol{R} = Exp(\boldsymbol{\omega})
$$
这里可以参考slam十四讲的内容，Exp与exp是不同的。

类似的有对数映射（**the Log map**），
$$
\boldsymbol{\omega}=Log(\boldsymbol{R})
$$
指数映射与对数映射可以简单的理解成旋转向量与旋转矩阵的转换关系，看成两个函数就可以。

这里的 $\boldsymbol{\omega}$ 就是 $so(3)$ 的元素，$\boldsymbol{R}$ 就是 $SO(3)$ 的元素。

指数映射就是 $so(3) \rightarrow SO(3) $ ，对数映射就是 $SO(3) \rightarrow so(3)$。

## 目标函数

在 $SO(3)$ 上定义目标函数，即用旋转矩阵 $\boldsymbol{R}$ 的形式：
$$
cost(\boldsymbol{R}) = {\norm{Log( \boldsymbol{R}_{target}^{\top} \boldsymbol{R} )}}^2
$$

------

使用旋转向量的形式表达目标函数：
$$
cost(\boldsymbol{\omega})=cost(\boldsymbol{R}(\boldsymbol{\omega}))
$$
其中 $\boldsymbol{R}(\boldsymbol{\omega}) = Exp(\boldsymbol{\omega}), \boldsymbol{R}(\boldsymbol{\omega}) \in SO(3)$ 。

> 人们在2015年后喜欢使用它，因为：
>
> 1. 轴角表示法是3D球面的均匀划分（需要证明）。优化速度更快，因为相对于so(3)的雅可比矩阵/梯度对残差/成本函数是更好的局部近似。
> 2. 使用起来很方便。雅可比矩阵的推导是可管理的。如果我们进行旋转的局部参数化，推导会更加简单。
> 3. 看起来很高大上。你可以称它为：“流形上的xxx”，“xxx李群”。通过好的故事讲述，很容易发表论文。

## 流形上的优化

$$
cost(\boldsymbol{R}) = {\norm{Log( \boldsymbol{R}_{target}^{\top} \boldsymbol{R} )}}^2
$$

$\boldsymbol{R}_{target}^{\top} \boldsymbol{R}$ 是 $\boldsymbol{R}_{target}$  与 $\boldsymbol{R}$ 之间的“差值”，如果二者一样，结果应该是单位矩阵。我们在这里可以记为 $\boldsymbol{R}_{target}^{\top} \boldsymbol{R} = \text{diff-}SO(3)$ 。

对数映射可以完成元素从 $SO(3) \rightarrow so(3)$ 的映射。那么可以将 $\text{diff-}SO(3)$ 通过对数映射转换为  $\text{diff-}so(3) = Log(\text{diff-}SO(3))$ 。

$\norm{\text{diff-}so(3)}^2$ 将旋转向量上的差异转换成一维的，也就是损失值。

可以通过对 $so(3)$ 参数求导，证明这个残差函数是线性的。

## 证明残差函数是线性

**TODO**

## 单步高斯牛顿法

由于残差函数是线性的，并且损失函数是残差函数的二次形式，这种优化问题可以一步高斯牛顿解决。

## 代码实现

> 这里附上源python实现: https://github.com/yimuw/yimu-blog/tree/master/least_squares/lie_linear_residual

### 对数映射与指数映射

这里使用了四个函数来分别表达：

+ **wedge.m**
	对于向量 $\boldsymbol{v} = {\begin{bmatrix} v_1& v_2& v_3 \end{bmatrix}}^\top$, 有 $\boldsymbol{v}^{\wedge} = \begin{bmatrix} 0& -v_3& v_2 \\ v_3& 0 & -v_1 \\  -v_2& v_1 &0  \end{bmatrix} $
+ **expm_wedge.m**
	MATLAB内置函数 expm() 为矩阵指数，此函数调用 expm() 实现 $Exp(\boldsymbol{\omega}) = exp(\boldsymbol{\omega} ^ \wedge)$ 
+ **vee.m**
	对于反对称矩阵$\boldsymbol{R} = \begin{bmatrix} 0& -v_3& v_2 \\ v_3& 0 & -v_1 \\  -v_2& v_1 &0  \end{bmatrix} $，有 $\boldsymbol{R}^{\vee} = \begin{bmatrix} v_1& v_2& v_3 \end{bmatrix}^\top$
+ **vee_logm.m**
	MATLAB内置函数 logm() 为矩阵对数，此函数调用 logm() 实现 $ Log(\boldsymbol{R}) = (log(\boldsymbol{R})) ^ \vee$ 

**optimization_on_SO3.m** 中定义了高斯牛顿法，与优化问题的解决。

## 损失

TODO