# 第三节 渐进稳定性及证明

本节主要介绍 MPC 的稳定性理论及渐进稳定性的证明。

## Recap

对于如下的控制系统 

$$
x^{+} = f(x, \kappa(x))
$$

如果在状态空间 $\mathbb{X}$ 上存在一个李亚普诺夫函数 $V$，则系统在 $\mathbb{X}$ 上是渐进稳定的。

> **定义**：**李亚普诺夫函数 (Lyapunov function)** 
>
> 假设集合 $\mathbb{X}$ 是系统 $x^+ = f(x)$ 的正不变集 (positive invariant set)，若函数 $V : \mathbb{R}^n \rightarrow \mathbb{R}_{\geq 0}$ 满足以下条件

$$
\alpha_2 (|x|) \leq V(x) \leq \alpha_1 (|x|) \\
V(f(x)) - V(x) \leq - \alpha_3(|x|)
$$
> 则这个函数是李亚普诺夫函数。

> 在如上定义中提到了正不变集的概念，正不变集的定义如下：如果任意状态 $x \in \mathbb{X}$ 满足 $x^+ = f(x) \in \mathbb{X}$ ，则集合$\mathbb{X}$是正不变集。



通常为了证明系统稳定性，需要证明无限时间最优控制问题 (infinite horizon optimal control) 的代价函数 (cost function)  为李亚普诺夫函数。在 MPC 问题中，通常选取有限时间最优控制问题的代价函数$V_N^0(\cdot)$，证明其满足李亚普诺夫函数的定义，即证明了 MPC 系统的稳定性。下面我们推导无状态约束 (state constraints) 下的 MPC 稳定性的相关结论。



## 无约束下的稳定性

### 符号说明

在无约束问题中，状态空间为整个 $\mathbb{R}^n$ 空间，即 $\mathbb{X} = \mathbb{R}^n$ 。为了保证在状态空间上的稳定性，要求$k$ 步内能控状态的集合 $\mathcal{X}_k = \mathbb{R}^n$ ，这样系统能够保证在状态空间中任意状态初始化时均能控制到终端约束集合 ( terminal set ) 中。

我们可以使用如下符号表示状态在$x$时步长为N的最优控制序列为
$$
\boldsymbol{u}_N^0(x) = (~u_N^0(0;x), ~ u_N^0(1; x),  ~\cdots , ~u_N^0 (N-1; x))
$$
其中$\boldsymbol{u}$ 加粗，代表它是一个序列，下标 $N$ 代表预测步长，上标$0$ 代表该序列为最优控制序列，$u_N^0(k; x)$ 代表最优控制序列的第 $k$ 步输入。

我们用如下符号表示最优控制下的状态序列 $\boldsymbol{x}_N^0(x)$
$$
\boldsymbol{x}_N^0(x) = ( x _ { N } ^ { 0 } ( 0 ; x ) , x _ { N } ^ { 0 } ( 1 ; x ) , \ldots , x _ { N } ^ { 0 } ( N ; x ) )
$$
我们用 $x^+$ 表示系统位于状态$x$输入最优控制$u_N^0(0;x)$ 后的下一个状态，即最优状态序列 $\boldsymbol{x}_N^0(x)$的第二个状态（$x _ { N } ^ { 0 } ( 0 ; x )$是第一个状态）
$$
x^+ = f(x, \kappa(x))=  f(x, u_N^0(0;x)) = u_N^0(1;x)
$$
我们需要推导出，在无约束情形下，如果选取$V_N^0(\cdot)$ 为闭环系统的$x^+ = f(x, \kappa(x))$李亚普诺夫函数，需要满足什么样的条件。

### 推导







### 形象理解





## 时不变系统的稳定性







## 时变系统的稳定性







参考文献
