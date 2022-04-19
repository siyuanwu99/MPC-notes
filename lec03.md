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

我们可以使用如下符号表示状态在$x$时步长为 N 的最优控制序列为
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

$V_N^0(\cdot)$ 需满足李亚普诺夫函数定义的两个条件，第一个条件很容易证明满足，我们的关注点在第二个条件上，即，
$$
\underbrace { V _ { N } ^ { 0 } ( x ^ { + } ) } _ { ( 1 ) } \leq \underbrace{V _ { N } ^ { 0 } ( x )}_{(2)} - \alpha _ { 3 } ( | x | )
$$
为了辅助证明，需要定义初始状态为$x^+$ 状态下的最优代价函数：
$$
V _ { N } ^ { 0 } ( x ^ { + } ) \stackrel { \text { def. } } { = } V _ { N } ( x ^ { + } , \boldsymbol{u} _ { N } ^ { 0 } ( x ^ { + } ) ) 
$$
由于最优性，任意改变输入序列第 N 项的输入，$V _ { N } ^ { 0 } ( x ^ { + } )$ 应该是任意输入序列$\tilde { \boldsymbol u }$对应的代价函数$V _ { N } ( x ^ { + }, \tilde{\boldsymbol{u}})$的极小值。这个任意输入序列叫做偏移控制序列 (shifted control sequence)，即只有最后一项输入为控制空间$\mathbb{U}$ 中的任意输入，而前$N-1$项输入为最优控制序列的输入，
$$
\tilde { \boldsymbol u } = ( u _ { N } ^ { 0 } ( 1 ; x ) , u _ { N } ^ { 0 } ( 2 ; x ) , \ldots , u _ { N } ^ { 0 } ( N - 1 ; x ) , \mathbb { u } )
$$
注意这个输入序列是从$u _ { N } ^ { 0 } ( 1 ; x ) $ 开始的，初始状态是$x^{+}$。对系统输入偏移控制序列后得到状态序列称为偏移状态序列
$$
\tilde { \boldsymbol x } = ( ~ x _ { N } ^ { 0 } ( 1 ; x ) , ~  x _ { N } ^ { 0 } ( 2 ; x ) , ~ \ldots ~  , ~  x _ { N } ^ { 0 } ( N ; x ), ~  f ( x _ { N } ^ { 0 } ( N ; x ) , \mathbb{u} ))
$$
因此我们可以得到以下关系
$$
\underbrace { V _ { N } ^ { 0 } ( x ^ { + } ) } _ { ( 1 ) } \leq  \underbrace{V _ { N } ( x ^ { + }, \tilde{\boldsymbol{u}})}_{(3)}
$$
现在我们将 (2) 式展开，它是初始状态为$x$ 时的最优目标函数
$$
(2)= V _ { N } ^ { 0 } ( x ) = \ell ( x , u _ { N } ^ { 0 } ( 0 ; x ) ) + \sum_{k=1}^{N -1} \{ \ell ( x_N^0(k;x), u_N^0(k;x) ) \} + V _ { f } ( x _ { N } ^ { 0 } ( N ; x ) )
$$
然后我们再将 (3) 式展开，它是初始状态为 $x^+$时，前$N-1$ 步最优，第$N$步任意输入下的目标函数
$$
(3) = V _ { N } ( x ^ { + }, \tilde{\boldsymbol{u}}) = \sum _ { k = 1 } ^ { N - 1 } \{ \ell ( x_N^0(k;x), u_N^0(k;x) )  \} + \ell ( x _ { N } ^ { 0 } ( N ; x ) , \mathbb{u} ) +  V _ { f } ( f (  x _ { N } ^ { 0 } ( N ; x ) , \mathbb{u})
$$
通过比较 (2) 式和 (3)，我们发现第 1 步到第 N-1 步的 stage cost 是一样的，可以消去，
$$
(3) - (2) =  V _ { f } ( f (  x _ { N } ^ { 0 } ( N ; x ) , \mathbb{u}) + \ell ( x _ { N } ^ { 0 } ( N ; x ) , \mathbb{u} ) - \ell ( x , u _ { N } ^ { 0 } ( 0 ; x ) ) -V _ { f } ( x _ { N } ^ { 0 } ( N ; x ) )
$$
由于已知$(1)  \leq (3) $ ，我们希望 $(3) \leq (2) - \alpha_3(|x|)$ ，即
$$
V _ { f } ( f (  x _ { N } ^ { 0 } ( N ; x ) , \mathbb{u}) + \ell ( x _ { N } ^ { 0 } ( N ; x ) , \mathbb{u} ) - \ell ( x , u _ { N } ^ { 0 } ( 0 ; x ) ) -V _ { f } ( x _ { N } ^ { 0 } ( N ; x ) ) \leq - \alpha_3(|x|)
$$
这里为了简化，我们令$\mathbb{x} = x_N^0(N; x)$ ，表示初时状态为$x$时最优控制下的末状态，整理后上式变为
$$
V _ { f } ( f (  \mathbb{x} , \mathbb{u})) -V _ { f } ( \mathbb{x}) + \ell ( \mathbb{x} , \mathbb{u} )  - \ell ( x , u _ { N } ^ { 0 } ( 0 ; x ) )   + \alpha_3(|x|) \leq 0
$$
 从上式得出，$V_N^0(\cdot)$ 是李亚普诺夫函数的两个充分不必要条件分别为
$$
\ell ( x , u _ { N } ^ { 0 } ( 0 ; x ) )  \geq \alpha_3(|x|) \\
V _ { f } ( f (  \mathbb{x} , \mathbb{u})) -V _ { f } ( \mathbb{x}) + \ell ( \mathbb{x} , \mathbb{u} ) \leq 0
$$
前者要求每步的 stage cost 是正的，后者要求 terminal cost $V_f(\cdot)$ 是 [控制李亚普诺夫](https://zh.wikipedia.org/wiki/%E6%8E%A7%E5%88%B6%E6%9D%8E%E4%BA%9E%E6%99%AE%E8%AB%BE%E5%A4%AB%E5%87%BD%E6%95%B8) 函数。

> 注意！
>
> 显然，当需要系统全局渐进稳定时，意味着存在全局的控制李亚普诺夫函数。但是，假如存在全局的控制李亚普诺夫函数，我们就不必采用 MPC 求解这个问题。我们可以采用 最小化全局控制李亚普诺夫函数 $u^\star = \operatorname{argmin}_u V(f(x, u))$  来找到该问题的最优解。

### 形象理解

## 时不变系统的稳定性

## 时变系统的稳定性

参考文献
