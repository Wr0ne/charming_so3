# 一个简单的使用流形优化姿态的例子

> 本文参考博客：[Introduction to Optimization on Manifolds/Lie-groups](https://wang-yimu.com/introduction-to-optimization-on-manifolds/)，是对博客中的python代码的MATLAB实现。

在阅读这篇博客前，我一直不明白李群李代数到底实际操作起来是如何用于位姿优化的。这篇文章，浅显易懂的阐述了一个流形优化的最简单案例。

下面对这个最简单的案例进行MATLAB的复现。

## 在李群、或流形上优化的优点

对于很多优化问题而言，在李群上参数化可以让优化问题的求解更快速简洁。

## 旋转参数

原博文中比较了 **3×3 matrix + cost，yaw-pitch-roll，Axis-angle representation，Quaternions**四种旋转的参数化表达的优缺点，这里仅对后续优化采用的旋转向量形式简单叙述。

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

指数映射就是 $so(3) \mapsto SO(3) $ ，对数映射就是 $SO(3) \mapsto so(3)$。

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

对数映射可以完成元素从 $SO(3) \mapsto so(3)$ 的映射。那么可以将 $\text{diff-}SO(3)$ 通过对数映射转换为  $\text{diff-}so(3) = Log(\text{diff-}SO(3))$ 。

$\norm{\text{diff-}so(3)}^2$ 将旋转向量上的差异转换成一维的，也就是损失值。

可以通过对 $so(3)$ 参数求导，证明这个残差函数是线性的。

## 证明残差函数是线性

由上文我们可以知道残差函数为
$$
\begin{align}
r(\boldsymbol{\omega}) &= {Log( \boldsymbol{R}_{target}^{\top} \boldsymbol{R} )}
\\
&= Log( Exp(\boldsymbol{\omega}_{\boldsymbol{R}_{target}^{\top}}) Exp(\boldsymbol{\omega}_{\boldsymbol{R}}) )
\end{align}
$$
由于
$$
Log( Exp(\boldsymbol{\phi}) Exp(\delta \boldsymbol{\phi}) ) 
\approx
\boldsymbol{\phi} + J_{r}^{-1}(\boldsymbol{\phi})\delta \boldsymbol{\phi}
$$

$$
J_{r}(\boldsymbol{\phi}) = \boldsymbol{I} 
- \frac{1-cos(\norm{\boldsymbol{\phi}})}{\norm{\boldsymbol{\phi}}^2}\boldsymbol{\phi}^{\wedge}
+ \frac{\norm{\boldsymbol{\phi}}-sin(\norm{\boldsymbol{\phi}})}{\norm{\boldsymbol{\phi}^3}} (\boldsymbol{\phi}^{\wedge})^2
$$

所以有
$$
\begin{align}
r(\boldsymbol{\omega}) &= 
Log( Exp(\boldsymbol{\omega}_{\boldsymbol{R}_{target}^{\top}}) Exp(\boldsymbol{\omega}_{\boldsymbol{R}}) )
\\
&\approx
\boldsymbol{\omega}_{\boldsymbol{R}_{target}^{\top}} + 
J_{r}^{-1}(\boldsymbol{\omega}_{\boldsymbol{R}_{target}^{\top}})\boldsymbol{\omega}_{\boldsymbol{R}}
\end{align}
$$
由于 $\boldsymbol{\omega}_{\boldsymbol{R}_{target}^{\top}}$ 在残差函数里是确定不变的，所以 $r(\boldsymbol{\omega})$ 也就是 $r(\boldsymbol{\omega}_{\boldsymbol{R}})$ 是一个线性函数。

## 单步高斯牛顿法

由于残差函数是线性的，并且损失函数是残差函数的二次形式，这种优化问题可以一步高斯牛顿解决。这个地方不难理解，联系牛顿迭代法解方程，对于线性函数而言，第一步求出来的根就是方程的根。

![牛顿迭代法示意图](https://wrone.top/img/20160803135148447)

下面给出高斯牛顿法的一般流程：

高斯-牛顿法（Gauss-Newton Method）是一种用于非线性最小二乘问题的迭代算法。它在最小化非线性函数的残差平方和时非常有效，特别是在拟合模型参数时。以下是高斯-牛顿法的算法流程：

### 高斯-牛顿法的算法流程

1. **初始化**：
  
   - 选择初始参数向量 $\mathbf{x}_0$。
   - 设置容许误差 $\epsilon$ 和最大迭代次数 $N_{\text{max}}$。
   
2. **迭代**：
   对于每次迭代 $k = 0, 1, 2, \ldots, N_{\text{max}}$：
   
   a. **计算残差向量** $\mathbf{r}(\mathbf{x}_k)$：
   $$
   \mathbf{r}(\mathbf{x}_k) = \begin{bmatrix} r_1(\mathbf{x}_k) \\ r_2(\mathbf{x}_k) \\ \vdots \\ r_m(\mathbf{x}_k) \end{bmatrix}
   $$
   其中 $r_i(\mathbf{x})$ 是目标函数的第 $i$ 个残差。

   b. **计算雅可比矩阵** $J(\mathbf{x}_k)$：
   $$
   J(\mathbf{x}_k) = \begin{bmatrix} \frac{\partial r_1}{\partial x_1} & \frac{\partial r_1}{\partial x_2} & \cdots & \frac{\partial r_1}{\partial x_n} \\ \frac{\partial r_2}{\partial x_1} & \frac{\partial r_2}{\partial x_2} & \cdots & \frac{\partial r_2}{\partial x_n} \\ \vdots & \vdots & \ddots & \vdots \\ \frac{\partial r_m}{\partial x_1} & \frac{\partial r_m}{\partial x_2} & \cdots & \frac{\partial r_m}{\partial x_n} \end{bmatrix}
   $$

   c. **计算法向方程**：
   解法向方程 $J(\mathbf{x}_k)^T J(\mathbf{x}_k) \Delta \mathbf{x}_k = -J(\mathbf{x}_k)^T \mathbf{r}(\mathbf{x}_k)$ 求解增量 $\Delta \mathbf{x}_k$。
   
   d. **更新参数向量**：
   $$
   \mathbf{x}_{k+1} = \mathbf{x}_k + \Delta \mathbf{x}_k
   $$
   
   e. **检查收敛条件**：
   如果 $\|\Delta \mathbf{x}_k\| < \epsilon$ 或 $\|\mathbf{r}(\mathbf{x}_k)\| < \epsilon$，则停止迭代。

3. **输出结果**：
   - 迭代结束后，输出参数向量 $\mathbf{x}^*$ 作为最终的估计值。

### 高斯-牛顿法的关键点

1. **雅可比矩阵的计算**：
   雅可比矩阵 $J(\mathbf{x})$ 是高斯-牛顿法中的核心，它包含了残差函数的导数信息。对于复杂的残差函数，计算雅可比矩阵可能会比较繁琐。

2. **法向方程的求解**：
   需要求解线性方程组 $J(\mathbf{x})^T J(\mathbf{x}) \Delta \mathbf{x} = -J(\mathbf{x})^T \mathbf{r}(\mathbf{x})$。在矩阵 $J(\mathbf{x})^T J(\mathbf{x})$ 是满秩的情况下，该方程组有唯一解。

3.  **收敛性**：
  
   高斯-牛顿法通常对初始值比较敏感，选择好的初始值有助于算法更快收敛。对于一些病态问题或者残差函数具有较强非线性的情况，可能需要结合其他优化方法（如Levenberg-Marquardt算法）来提高收敛性能。

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