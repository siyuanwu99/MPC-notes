# Lec 4 MPC 终端集（Terminal set）的设计

## 简介

根据之前的内容，为了保证 MPC 控制闭环系统的渐进稳定性（Asymptotic stability），在预测步数（Horizon）最后一步时（第 N 步），我们需要让状态$x_N$在终端集$\mathcal{X_f}$中。

$$
x_N \in \mathcal{X_f}
$$

本章介绍了三种常用的终端集$\mathcal{X_f}$设计方法 [1]，主要针对线性时不变（LTI）系统，并在最后进行了简单的讨论。

对于非线性系统 $x^+=f(x,u)$，在原点（平衡点）进行线性化后：

$$
\begin{array}{l}
A=\left.\frac{\partial f}{\partial x}\right|_{x=0, u=0} \\
B=\left.\frac{\partial f}{\partial u}\right|_{x=0, u=0}
\end{array}\\
x^+=Ax+Bu
$$

只要$(A,B)$可控，依然可以使用下面介绍的方法。

## MPC 终端集$\mathcal{X_f}$的性质

根据之前第二章和第三章的内容，我们了解到了终端集$X_f$的性质。其中三条重要的性质是控制不变性（control invariant），约束容许性（constraint admissible）和李雅普诺夫递减性（lyapunov decrease）。

* 控制不变性（control invariant）
  
$$
\forall x \in \mathcal{X_f} \ ,\exists u \in \mathcal{U}\ , x^+=f(x,u)\in \mathcal{X_f}
$$

对于终端集$\mathcal{X}_f$中任意状态$x$，存在符合输入约束的 $u$ 使它下一步状态$x^+$依然在$\mathcal{X_f}$里。表示当状态$x$进入终端集$\mathcal{X_f}$后，后续状态不会再离开$\mathcal{X_f}$。

* 约束容许性（constraint admissible）
  
$$
\forall x \in \mathcal{X_f} \ , x\in \mathcal{X}
$$

终端集$X_f$中任意状态$x$的符合状态约束集$\mathcal{X}$。状态约束指施加在状态变量上的约束，比如通过在四旋翼无人机系统的横滚角（roll）和俯仰角（pitch）上施加约束，来限制角度，保证系统在线性化模型的平衡点附近工作。

输入$u$也需要符合输入约束。比如四旋翼无人机系统中，限制推力或转矩上的约束。若控制输入$u$为状态变量的函数（状态反馈）, 则也可以写成对状态$x$的约束。

$$
\forall x \in \mathcal{X_f} \ , u=\mathcal{K}_N(x)\in \mathcal{U}
$$
下图为一个简单的二维示意图。$\mathcal{X_f}$（红色）中所有状态满足状态约束，即包含于蓝色集合$X$。同时后续状态轨迹保持在$X_f$内。
    <div align=center>
    <img width="400" height="350" src=figures/Lec0401.png/>
    </div>

* 李雅普诺夫递减性 （lyapunov decrease）
  
$$
V_{f}(f(x))\leq V_{f}(x)-l(x,u) \ ,\forall x \in \mathcal{X_f} \ ,\exists u \in \mathcal{U}
$$

对于终端集中任意状态$x$, 总有 $u \in \mathcal{U}$ , 使下一步的终端代价$V_{f}(f(x))$小于等于此状态下的终端代价$V_{f}(x)$减去状态代价$l(x,u)$。此性质主要针对终端代价函数，但在终端集中的任意状态$x$都需满足。

除此之外，$X_f$还需要为紧集（compact set）且包含平衡点（equilibrium point）。下面介绍三种保证满足以上顺序的终端集$\mathcal{X_f}$设计方法。

## 1. 基于无约束 LQR 的多面体（Polyhedron）终端集 Terminal set as invariant constraint admissible set

第一种方法采用基于无约束 LQR 的方法控制第 N 步之后的闭环系统。换句话说，当预测步长最后一步的状态$x_N$进入终端集之后，MPC 采用和无约束 LQR 相同的控制策略。具体方法其实就是把离散代数 Riccati 方程（DARE）的解作为终端代价函数的二次型矩阵。最终得到终端集的是由线性不等式约束围成的多面体。

$$
\mathcal{X}_f=\{x \in \mathbf{R}^n | Hx \leq h\}
$$

### 1.1 李雅普诺夫递减性

下面简单证明李雅普诺夫递减性。

考虑 LTI 系统状态空间方程：

$$
x_{k+1}=Ax_k+Bu_k
$$
和二次状态代价函数：

$$
l(x, u) = \frac{1}{2}(x^TQx + u^TRu)
$$
其中$（A，B）$可控，$Q，R$正定。对于此系统考虑无限步长 (infinite horizon) 的无约束 (uc) 优化问题：

$$
\mathbb{P}_{\infty}^{\text {uc }}(x): \min _{u} \sum_{k=0}^{\infty} \ell(x(k), u(k))
$$

根据离散代数 Riccati 方程（DARE）, 具体推导过程请参考第二章：

$$
P=A_K^{T} P A_K+Q_K>0
$$
其中$A_K=A+BK \ $ （有些资料写的是$A-BK$, 是因为这里取$K$的时候将前边的负号也算上了）, $\ Q_K=Q+K^TRK$:

$$
x_{k+1}=(A+BK)x_k=A_Kx_k
$$

$$
l(x_k)=\frac{1}{2}x_k^T(Q+K^TRK)x_k=\frac{1}{2}x_k^TQ_Kx_k
$$
此时输入$u=Kx$为状态反馈。$K$为无约束 LQR 控制增益矩阵：

$$
K=-\left(B^{\top} P B+R\right)^{-1} B^{\top} P A^{\top}
$$
求解上述 DARE 方程，将$P$作为终端代价函数的二次型矩阵：

$$
V_f(x)=\frac{1}{2}x^TPx
$$
当 MPC 控制状态进入终端集后，MPC 的终端代价函数$V_f$和 LQR 代价函数相同。二者都会找到$V_f$作为代价函数的最优解，所以 MPC 控制输入会和无约束 LQR 控制输入相同，即$u=Kx$。

对于任意对于终端集中任意状态$x_k$, 下一步的终端代价$V_{f}(x_{k+1})$为：

$$
V_{f}(x_{k+1})=V_f(A_Kx_k)=\frac{1}{2}x_K^T(A_K^TPA_K)x_k=\frac{1}{2}x_K^T(P-Q_K)x_k
$$
所以满足李雅普诺夫递减性：

$$
V_{f}(x_{k+1})=\frac{1}{2}x_K^T(P-Q_K)x_k=V_{f}(x_k)-l(x_k)
$$

### 1.2 生成终端集的方法

生成$X_f$具体算法如下 [2]：
<div align=center>
    <img width="380" height="420" src=figures/Lec0402.png/>
    </div>

其中$f_i$表示第$i$个线性约束函数，即$H_ix$。$H$是包含了所有终端线性不等式约束函数的矩阵，它的行数等于约束的个数，列数等于状态变量的个数。终端集$X_f$为线性不等式约束$Hx \leq h$下的集合。可以看出，每个线性不等式约束相当于状态空间中的一个超平面。又由于$X_f$为紧集，最后形成$X_f$的形状是用多个超平面围出来的一个高维度多面体 (Polyhedron)。多面体中的所有点（状态$x$）满足终端集性质。

可以通过 Matlab 的 dlqr 函数，或 Python 的 control library 中的 control.dare 函数求解 DARE 得到$K$。

**注意该算法中$A_K=A-BK$，和我们之前的介绍符号不同，这是因为它这里的 K 本身与我们取的 K 符号也相反。**

### 1.3 生成终端集原理

根据约束容许性质，状态约束$x \in \mathcal{X} $, 输入约束$u \in \mathcal{U} $应分别被满足。写为线性不等式约束：

$$
 \mathcal{X} \subseteq\left\{H_x x \leq h_{x}\right\} \\
\mathcal{U} \subseteq\left\{H_{u} u \leq h_{u}\right\}
$$

当$u=Kx$，即输入为状态反馈时有：

$$
\left[\begin{array}{ll}
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
\end{array}\right]
$$

上式的$[K ; I]$就是之前算法中的$K_{aug}$。同理，对于下一步的状态$x^+$同样需要符合约束：

$$
\left[\begin{array}{ll}
H_{u} & 0 \\
0 & H_{x}
\end{array}\right]K_{aug} x^+=\left[\begin{array}{cc}
H_{u} & 0 \\
0 & H_{x}
\end{array}\right]K_{aug}  A_Kx \leqslant\left[\begin{array}{l}
h_{u} \\
h_{x}
\end{array}\right]
$$
每前进一步就会多乘一个$A_K$, 所以会有$A_K$的多次幂。

**而这个算法构造不变集（invariant set）的方法核心其实就是每前进一步时，通过求解线性规划（LP）问题检查下一步的所有状态$\mathbf{x_{k+1}}$组成的集合是不是前一步所有状态$\mathbf{x_k}$组成集合的子集。如果不是，那么取所有这两步状态集合的交集（通过添加约束）作为当前状态集合，然后用这个集合往后接着推，检查再下一步的所有状态的集合。直到找到连续两步中，后一步的所有状态$\mathbf{x_{k+1}}$组成的集合完全包含于前一步所有状态组成的集合，则前一步所有状态$x_k$组成的集合为不变集。**

更直观的理解请看下面的二维简化示意图：

<div align=center>
    <img width="325" height="250" src=figures/Lec0403.png/>
    <img width="325" height="250" src=figures/Lec0404.png/>
    <img width="325" height="250" src=figures/Lec0405.png/>
</div>

最终得到终端集约束：

$$
x_N \in \mathcal{X_f}\Rightarrow Hx_N\leq h
$$

代码实例请参考：[方法一生成四旋翼无人机系统终端集](https://github.com/SailorBrandon/MPC-for-Quadrotors/blob/9c60221b052ea10942d0039b1183f16d06b6d2f4/terminal_set.py)。

## 2. 二次代价函数的水平子集作为终端集 Terminal set as quadratic level set

将正定连续的二次函数作为终端代价函数：

$$
V_f(x)=\frac{1}{2}x^TPx
$$
其中$P$不一定需要像之前一样是 DARE 的解，但需要额外验证满足李雅普诺夫递减性。如果依然选取 DARE 的解作为$P$, 根据之前证明，李雅普诺夫递减性会有保证。

选择$V_f$的水平子集作为终端集$\mathcal{X}_f$:

$$
\mathcal{X}_f=\{x \in \mathbf{R}^n | V_f(x) \leq c\}
$$

$$
c>0 \; s.t. \ \mathcal{X}_f \subseteq \mathcal{X}, K(\mathcal{X}_f)\subseteq \mathcal{U}
$$

其中$c$取一大于零常数。终端集$\mathcal{X}_f$的控制不变性也很容易从李雅普诺夫递减性推出。

$$
V_{f}(f(x))\leq V_{f}(x)-l(x,u) \leq c
$$
由于下一步的终端代价函数值$V_f(f(x))$一定小于等于当前步的终端代价$V_f(x)$，所以下一步的状态$x^+=f(x)$一定依然属于终端集。换句话说，进入终端集后下一步所有的可能状态的终端代价值一定小于等于$c$ , 所以它们组成的集合一定为$\mathcal{X}_f$子集。

由于生成的终端集为正定二次函数的水平子集，终端集的形状是一个在高维度中的椭圆体。

最后，约束容许性是通过将 c 设置并调整为一个较小值，从而保证生成的终端集较小，不会超出约束容许集的范围来满足的。一般通过检查椭圆体的外接多面体的顶点是否在约束容许集的范围内来确定是否满足约束容许性。通过计算 $P$ 的特征值及特征向量得到椭圆的半轴 semiaxis 向量，从而得到顶点。当所有顶点都在约束容许集内时，终端集$\mathcal{X}_f$一定满足约束容许性。
<div align=center>
    <img width="380" height="270" src=figures/Lec0408.png/>
</div>

注意此方法会使计算出的终端集比实际终端集更小，而实际上我们希望终端集尽可能大。

<div align=center>
    <img width="300" height="250" src=figures/Lec0406.png/>
</div>

如上图所示，虽然此时水平子集以满足约束容许性，但外接多面体顶点依然处在约束容许集之外。所以需要进一步减小 c 的值。

<div align=center>
    <img width="300" height="250" src=figures/Lec0407.png/>
</div>

当 c 减小到一定值时，$V_{f}$水平子集的外接多面体顶点都会在约束容许集内，将此时的水平子集作为终端集。

$$
x_N \in \mathcal{X_f}\Rightarrow \frac{1}{2}x_N^TPx_N\leq c
$$
<!-- 
代码实例请参考：[方法二生成四旋翼无人机系统终端集](https://github.com/smoggy-P/MPC-Collision-Avoidance/blob/2de1c6459e20e53177c2861ceefe3a305f9139b1/MPC_controller.py)。 -->
## 3. 通过增大终端代价函数权重保证终端集性质的满足

第三种方法不直接计算终端集，并对最后一步状态$x_N$施加硬约束。而是直接通过增大终端代价函数$V_f$作为软约束来使$x_N$尽可能贴近原点（平衡点）。所以每次预测时的总代价函数为：

$$
V_{N}^{\beta}(x, \boldsymbol{u})=\sum_{k=0}^{N-1}\{\ell(x_k, u_k)\}+\beta V_{\mathrm{f}}(x_N), \; \beta \gg 1
$$

可以看到，此方法通过将终端代价乘一个远大于 1 的常数$\beta$，来实现对数值较大的 $x_N$的惩罚。

当选择的$V_f$符合李雅普诺夫递减时，$\beta V_f$同样符合李雅普诺夫递减。由$\beta \gg 1, l(x,u)>0$有：

$$
\beta V_{\mathrm{f}}(f(x, u)) \leq \beta V_{\mathrm{f}}(x)-\beta \ell(x, u) \leq \beta V_{\mathrm{f}}(x)-\ell(x, u)
$$

对于约束容许性和不变性的证明，和之前方法 2 类似。通过假设一个水平子集：
$$
\mathcal{X}_f=W(a)=\{x \in \mathbf{R}^n | V_f(x) \leq a\}
$$

$$
a>0 \; s.t. \ \mathcal{X}_f \subseteq \mathcal{X}, K(\mathcal{X}_f)\subseteq \mathcal{U}
$$
来证明约束容许性和不变性（参考方法 2）。但不同的是，$W(a)$只用来在理论上证明。在实际计算时，并不作为硬约束添加在$x_N$上。而是通过调整增大$\beta$使$x_N$进入$\mathcal{X}_f$.

## 4. 讨论

### 4.1 方法比较

对于**生成终端集$\mathcal{X}_f$的过程**来说，方法 1 不需要直接调参。而方法 2 和 3 可能需要多次调整 c 和 $\beta$ 的值。但用方法 1 计算终端集的过程比较复杂，它通过对每个线性约束求解 LP 问题来验证不变性。而且每次迭代后会增加约束的数量。也就是说方法 1 迭代次数越多，求解复杂程度越高，所需时间越多。并且采用方法 1 时需要确认所生成的终端集是否为紧集（是否有界），如果状态约束不够的话，生成的终端集可能不是紧集。这时需要增加状态约束，使终端集在所有维度上都有界。

对于**MPC 计算速度**来说，方法 1 由于生成的是线性不等式约束，所以 MPC 求解过程仍然是二次规划 QP 问题。而方法 2 的终端集约束则是二次的，导致 MPC 求解变成了二次约束二次规划 QCQP 问题，理论上来讲求解是要比 QP 问题慢的。并且对于 MPC 每次计算，都需要在最后一步添加约束并求解。方法 3 由于不添加终端集约束，所以和方法 1 一样，都是 QP 问题。

对于**解最优性的影响**，方法 1 和 2 由于添加了终端集硬约束，可能会影响得到解的最优性。即 MPC 为了使最终状态$x_N$进入终端集，会牺牲一部分之前状态或输入的最优性。并且当初始条件离终端集太远导致$x_N$无法进入终端集时，MPC 会找不到解。其中方法 1 生成的终端集会大一些，使$x_N$更容易进入。

而方法 3 没有添加终端集硬约束，而只是改变了终端代价函数，所以理论上来讲对最优性的影响会小一些，并且即使$x_N$进入不了终端集，MPC 依然会找到解。但当$\beta$太大时也会降低模型的最优性，$\beta$太小时，$x_N$可能无法进入终端集。

### 4.2 实际应用和更多讨论

#### 4.1.1 可达性（feasibility）

MPC 不能将任意初始状态在预测步数内控制到进入终端集（比如现在位置离目标位置较远时）。这引入了可达集$\mathcal{X}_N$的概念。可达集 (feasible set) 包含所有可以在预测步数（horizon）之内进入终端集的初始状态。也就是说，一旦初始状态在可达集内，MPC 一定会找到解。但一般对于高维系统来说，可达集较难从数值上计算。当找不到解时，可以尝试增大 MPC 预测步数。

下图是一个可达集和终端集的二维截面示例。
<div align=center>
    <img width="300" height="250" src=figures/Lec0409.png/>
</div>
如上图所示，黄色区域代表了终端集，绿色区域代表了可达集，外层深色区域为不可达的状态点。当预测步数 (horizon) 增加时，绿色区域会变大。

#### 4.1.2 目标跟踪 (Reference tracking) 问题

本章介绍的主要内容主要是针对调节 (regulation) 问题讲的，即控制状态趋向于不变的吸引子（原点或平衡点）。对于目标跟踪问题的终端集计算，只需要先根据之前的方法得到调节问题得到终端集，再将终端集平移到需要跟踪的目标轨迹上就可以。具体请参考下一章。

#### 4.1.3 非线性系统和更多证明

本章主要给出了 LTI 系统的终端集设计方法，并简单地证明了一部分重要的性质。具体完整的证明和分析讲解请参照 Model predictive control: Theory, computation, and design[3] 的第二章。

对于一般的系统而言（包括无法线性化的非线性系统），当以下性质满足（充分不必要条件），也可以确认系统的渐近稳定性：

1. f 连续，$\ell,V_f$为二次函数且连续
2. $\forall x \in \mathbb{X}_{\mathrm{f}}, \exists u \in \mathbb{U} \text { s.t. } f(x, u) \in \mathbb{X}_{\mathrm{f}}  $
3. $V_{\mathrm{f}}(f(x, u)) \leq V_{\mathrm{f}}(x)-\ell(x, u) $
4. $\ell(x, u)=\frac{1}{2}\left(x^{\top} Q x+u^{\top} R u\right) \geq \frac{1}{2} x^{\top} Q x \geq \frac{1}{2} \lambda_{\min }(Q)|x|^{2} $
5. $V_{\mathrm{f}}(x)=\frac{1}{2} x^{\top} P x \leq \frac{1}{2} \lambda_{\max }(P)|x|^{2}$
6. $\mathbb{X}$ 为闭集； $\mathbb{U}$, $\mathbb{X}_{\boldsymbol{f}} \subseteq \mathbb{X}$ 为紧集；
7. 终端集包含原点或平衡点

其中，$\lambda_{\min }, \lambda_{\max }$分别代表取矩阵最小和最大特征值。

#### 4.1.4 工程应用

终端集在 MPC 理论中有着重要作用，但在实际工程应用中很多时候没有被采用。我认为这也是导致国内网上资料较少的原因之一。之所以应用较少，是因为当 MPC 的模型和参数设置合理时，大多数情况下不会出现失稳的情况，而且会有不错的表现。

## 5. 参考

[1] S. Grammatico, “Lecture 4. model predictive control” 2022

[2] M. B. Alexander Berndt, “Balancing an inverted pendulum on a quadrotor: A model predictive approaach,” 2021.

[3] Rawlings, J. B., Mayne, D. Q., & Diehl, M. (2017). Model predictive control: Theory, computation, and design.
