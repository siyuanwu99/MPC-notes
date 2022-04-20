# MPC终端集（Terminal set）的设计
## 简介
根据之前的内容，为了保证MPC控制闭环系统的渐进稳定性（Asymptotic stability），在预测步数（Horizon）最后一步时(第N步)，我们需要让状态$x_N$在终端集$\mathcal{X_f}$中。
$$x_N \in \mathcal{X_f}$$
本章介绍了三种常用的终端集$\mathcal{X_f}$设计方法[1]，主要针对线性时不变（LTI）系统，并在最后进行了简单的讨论。
## MPC 终端集$\mathcal{X_f}$的性质
根据之前第二章和第三章的内容，我们了解到了终端集$X_f$的性质。其中三条重要的性质是控制不变性（control invariant），约束容许性（constraint admissible）和李雅普诺夫递减性（lyapunov decrease）。
* 控制不变性（control invariant）
    $$\forall x \in \mathcal{X_f} \ ,\exists u \in \mathcal{U}\ , x^+=f(x,u)\in \mathcal{X_f}$$
    终端集$X_f$中任意状态$x$的下一步$x^+$依然在$X_f$里。表示当状态$x$进入终端集$X_f$后，后续状态不会再离开$X_f$。
* 约束容许性（constraint admissible）
    $$\forall x \in \mathcal{X_f} \ , x\in \mathcal{X}$$
    终端集$X_f$中任意状态$x$的符合状态约束集$\mathcal{X}$。状态约束指施加在状态变量上的约束，比如通过在四旋翼无人机系统的横滚角（roll）和俯仰角（pitch）上施加约束，来限制角度，保证系统在线性化模型的平衡点附近工作。
    
    输入$u$也需要符合输入约束。比如四旋翼无人机系统中，限制推力或转矩上的约束。若控制输入$u$为状态变量的函数（状态反馈）,则也可以写成对状态$x$的约束。
    $$\forall x \in \mathcal{X_f} \ , u=\mathcal{K}_N(x)\in \mathcal{U}$$
    下图为一个简单的二维示意图。$X_f$（红色）中所有状态满足状态约束，即包含于蓝色集合$X$。同时状态轨迹保持在$X_f$内。
    <div align=center>
    <img width="400" height="350" src=figures/Lec0401.png/>
    </div>
* 李雅普诺夫递减性 （lyapunov decrease）
    $$V_{f}(f(x))\leq V_{f}(x)-l(x,u) \ ,\forall x \in \mathcal{X_f} \ ,\exists u \in \mathcal{U}$$
    对于终端集中任意状态$x$,总有$u \in \mathcal{U}$,使下一步的终端代价$V_{f}(f(x))$小于等于此状态下的终端代价$V_{f}(x)$减去状态代价$l(x,u)$。此性质主要针对终端代价函数，但在终端集中的任意状态$x$都需满足。

除此之外，$X_f$还需要为紧集（compact set）且包含平衡点（equilibrium point）。下面介绍三种保证满足以上顺序的终端集$\mathcal{X_f}$设计方法。
## 1. 基于无约束LQR的多面体（Polyhedron）终端集 Terminal set as invariant constraint admissible set
第一种方法采用基于无约束LQR的方法控制第N步之后的闭环系统。换句话说，当预测步长最后一步的状态$x_N$进入终端集之后，MPC采用和无约束LQR相同的控制策略。具体方法其实就是把离散代数Riccati方程（DARE）的解作为终端代价函数的二次型矩阵。
### 1.1 李雅普诺夫递减性
下面简单证明李雅普诺夫递减性。

考虑LTI系统状态空间方程：
$$x_{k+1}=Ax_k+Bu_k$$
和二次状态代价函数：
$$l(x, u) = \frac{1}{2}(x^TQx + u^TRu)$$
其中$（A，B）$可控，$Q，R$正定。对于此系统考虑无限步长(infinite horizon)的无约束(uc)优化问题：

$$\mathbb{P}_{\infty}^{\text {uc }}(x): \min _{u} \sum_{k=0}^{\infty} \ell(x(k), u(k))$$

根据离散代数Riccati方程（DARE）,具体推导过程请参考第二章：
$$P=A_K^{T} P A_K+Q_K>0$$
其中$A_K=A+BK \ $ （有些资料写的是$A-BK$,是因为这里取$K$的时候将前边的负号也算上了）, $\ Q_K=Q+K^TRK$:
$$x_{k+1}=(A+BK)x_k=A_Kx_k$$
$$l(x_k)=\frac{1}{2}x_k^T(Q+K^TRK)x_k=\frac{1}{2}x_k^TQ_Kx_k$$
此时输入$u=Kx$为状态反馈。$K$为无约束LQR控制增益矩阵：
$$K=-\left(B^{\top} P B+R\right)^{-1} B^{\top} P A^{\top}$$
求解上述DARE方程，将$P$作为终端代价函数的二次型矩阵：
$$V_f(x)=\frac{1}{2}x^TPx$$
当MPC控制状态进入终端集后，MPC的终端代价函数$V_f$和LQR代价函数相同。二者都会找到$V_f$作为代价函数的最优解，所以MPC控制输入会和无约束LQR控制输入相同,即$u=Kx$。

对于任意对于终端集中任意状态$x_k$,下一步的终端代价$V_{f}(x_{k+1})$为：
$$V_{f}(x_{k+1})=V_f(A_Kx_k)=\frac{1}{2}x_K^T(A_K^TPA_K)x_k=\frac{1}{2}x_K^T(P-Q_K)x_k$$
所以满足李雅普诺夫递减性：
$$V_{f}(x_{k+1})=\frac{1}{2}x_K^T(P-Q_K)x_k=V_{f}(x_k)-l(x_k)$$

### 1.2 生成终端集的方法

生成$X_f$具体算法如下[2]：
<div align=center>
    <img width="380" height="420" src=figures/Lec0402.png/>
    </div>

其中$f_i$表示第$i$个线性约束函数，即$H_ix$。$H$是包含了所有终端线性不等式约束函数的矩阵，它的行数等于约束的个数，列数等于状态变量的个数。终端集$X_f$为线性不等式约束$Hx \leq h$下的集合。可以看出，每个线性不等式约束相当于状态空间中的一个超平面。又由于$X_f$为紧集，最后形成$X_f$的形状是用多个超平面围出来的一个高维度多面体(Polyhedron)。多面体中的所有点（状态$x$）满足终端集性质。

可以通过Matlab的dlqr函数，或Python的control library中的control.dare函数求解DARE得到$K$。

**注意该算法中$A_K=A-BK$，和我们之前的介绍符号不同，这是因为它这里的K本身与我们取的K符号也相反。**

### 1.3 生成终端集原理

根据约束容许性质，状态约束$x \in \mathcal{X} $, 输入约束$u \in \mathcal{U} $应分别被满足。写为线性不等式约束：

$$ \mathcal{X} \subseteq\left\{H_x x \leq h_{x}\right\} \\
\mathcal{U} \subseteq\left\{H_{u} u \leq h_{u}\right\}$$

当$u=Kx$，即输入为状态反馈时有：
$$\left[\begin{array}{ll}
H_{u} & 0 \\
0 & H_{x}
\end{array}\right]\left[\begin{array}{l}
u \\
x
\end{array}\right]=\left[\begin{array}{cc}
H_{u} & 0 \\
0 & H_{x}
\end{array}\right]\left[\begin{array}{l}
K \\
I
\end{array}\right] x \leqslant\left[\begin{array}{l}
h_{u} \\
h_{x}
\end{array}\right]$$

上式的$[K ; I]$就是之前算法中的$K_{aug}$。同理，对于下一步的状态$x^+$同样需要符合约束：
$$\left[\begin{array}{ll}
H_{u} & 0 \\
0 & H_{x}
\end{array}\right]K_{aug} x^+=\left[\begin{array}{cc}
H_{u} & 0 \\
0 & H_{x}
\end{array}\right]K_{aug}  A_Kx \leqslant\left[\begin{array}{l}
h_{u} \\
h_{x}
\end{array}\right]$$
每前进一步就会多乘一个$A_K$,所以会有$A_K$的多次幂。

**而这个算法构造不变集（invariant set）的方法核心其实就是每前进一步时，检查下一步的所有状态$\mathbf{x_{k+1}}$组成的集合是不是前一步所有状态$\mathbf{x_k}$组成集合的子集。如果不是，那么取所有这两步状态集合的交集（通过添加约束）作为当前状态集合，然后用这个集合往后接着推，检查再下一步的所有状态的集合。直到找到连续两步中，后一步的所有状态$\mathbf{x_{k+1}}$组成的集合完全包含于前一步所有状态组成的集合，则前一步所有状态$x_k$组成的集合为不变集。**

更直观的理解请看下面的二维简化示意图：
<div align=center>
    <img width="325" height="250" src=figures/Lec0403.png/>
    <img width="325" height="250" src=figures/Lec0404.png/>
    <img width="325" height="250" src=figures/Lec0405.png/>
</div>

## 2. 二次代价函数的水平子集作为终端集 Terminal set as quadratic level set
## 3. 通过增大终端代价函数权重保证终端集性质的满足

## 4.讨论
## 5.参考
[1] S. Grammatico, “Lecture 4. model predictive control” 2022

[2] M. B. Alexander Berndt, “Balancing an inverted pendulum on a quadrotor: A model predictive approaach,” 2021.

