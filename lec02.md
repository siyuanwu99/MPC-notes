# Lec 2  从 LQR 到 MPC

## 引子：凸优化（Convex Optimization）
MPC 的实际上就是循环求解凸优化问题，所以有必要简单铺垫相关知识。一本大家都推荐的参考书应该是 Boyd 的 [《凸优化》](https://web.stanford.edu/~boyd/cvxbook/)。

优化问题标准型如下，其中$u$是决策变量（decision variable），$f(u)$是代价函数（cost function），$g(u)$是不等式约束，$h(u)$是等式约束。所有的约束定义了可行域（feasible set）。简单来说，我们想要找一个$u$，让代价函数最小。若想让这个优化问题是凸优化问题，我们需要代价函数和可行域都是凸的。具体来说，我们需要$f(u) 和 g(u)$是凸函数，$h(u)$是仿射的（affine）。

$$
\begin{equation}
\begin{aligned}
\min_{u} \quad & f(u)\\
\textrm{subject to} \quad & g(u) \leq 0 \\
  & h(u) = 0    \\
\end{aligned}
\end{equation}
$$

如果一个问题能被建模成是凸优化问题，我们就很高兴，原因有两个
- 局部最优解也就是全局最优解。
- 已经有成熟的算法处理这类问题了，也就是说，对问题完成建模之后可以直接丢给求解器。

## 线性二次型调节器 （LQR）

### 有限阶段 （Finite horizon）的 LQR 
Finite horizon LQR 是后续方法的基础，它对应的优化问题可以写成如下形式：
$$
\begin{equation}
\begin{aligned}
\min_{\boldsymbol{u}_{N}} \quad & V_{N}\left(x_{0}, \boldsymbol{u}_{N}\right)= \sum_{k=0}^{N-1}\{\ell(x(k), u(k))\}+V_{\mathrm{f}}(x(N)) \\
\textrm{subject to} \quad & x(0)=x_{0} \\
& x(k+1)=Ax(k) + Bu(k), \forall k\in \{0, ..., N-1\} \\
\end{aligned}
\end{equation}
$$
其中代价函数$V_{N}\left(x_{0}, \boldsymbol{u}_{N}\right)$由两部分组成：$\ell(x(k), u(k))$是**阶段代价（stage cost）**，$V_{\mathrm{f}}$是**终端代价（terminal cost）**。我们想让这个优化问题是凸的，所以把这两项都写成二次型的形式，即
$$
\begin{equation} 
\begin{aligned}
    l(x(k), u(k))=& x(k)^T Q x(k) + u(k)^T R u(k)\\
    V_f(x(N))=& x(N)^T P x(N)
\end{aligned}
\end{equation}
$$
x(k)^T Q x(k)$ 意味着尽量缩小状态误差，$u(k)^T R u(k)$则意味着在接近目标状态的同时要尽量减小输入量的值。在这里，我们需要$Q, P\succeq0$ 且 $R\succ0$来保证代价函数是凸的二次型。

上边这个优化问题是多阶段（multistage）优化问题，可以通过动态规划（dynamic programming）的方法求得最优输入的闭式解。**推导如下：**

<!-- 在教材 [Model Predictive Control:
Theory, Computation, and Design 2nd Edition ](https://sites.engineering.ucsb.edu/~jbraw/mpc/MPC-book-2nd-edition-3rd-printing.pdf) 的 18 到 20 页有详细的推导。最后我们可以得到： -->

$N-1$的阶段代价和终端代价可以写为：
$$
\begin{aligned}
&\ell(x(N-1), u(N-1))+V_f(x(N)) \\
&\quad=(1 / 2)\left(|x(N-1)|_{Q}^{2}+|u(N-1)|_{R}^{2}+|A x(N-1)+B u(N-1)|_{P}^{2}\right) \\
&\quad=(1 / 2)\left(|x(N-1)|_{Q}^{2}+|(u(N-1)-v)|_{H}^{2}+d\right)
\end{aligned}
$$
（注：$|x|_Q = x^T Q x$)

其中，
$$
\begin{aligned}
H &=R+B^{T} P B \\
v &=-\left(B^{T} P B+R\right)^{-1} B^{T} P A x(N-1) \\
d &=x(N-1)^{T}\left(A^{T} P A-A^{T} P B\left(B^{T} P B+R\right)^{-1} B^{T} P A\right) x(N-1)
\end{aligned}
$$
如果我们最小化 $\ell(x(N-1), u(N-1))+V_f(x(N))$，就可以得到在 $N-1$ 阶段的应该采用的最优输入 $u_{N-1}^{0}$。以及在采用 $u_{N-1}^{0}$ 的情况下，对应下一个阶段的最优状态 $x_{N}^{0}$ 和最优值函数（optimal cost to go）$V_{N-1}^{0} = min\{\ell(x(N-1), u(N-1))+V_f(x(N))\}$。具体的表达如下所示：
$$
\begin{equation}
\begin{aligned} 
u_{N-1}^{0} &=K(N-1) x(N-1) \\ 
x_{N}^{0} &=(A+B K(N-1)) x(N-1) \\ 
V_{N-1}^{0} &=(1 / 2) x(N-1)^{T} \Pi(N-1) x(N-1)\\
\end{aligned}
\end{equation}
$$
其中，

$$
\begin{equation}
\begin{aligned} 
K(N-1) &=-\left(B^{T} P B+R\right)^{-1} B^{T} P A\\
\Pi(N-1) &=Q+A^{T} P A-A^{T} P B\left(B^{T} P B+R\right)^{-1} B^{T} P A
\end{aligned}
\end{equation}
$$
在优化$V_{N-1}$之后，我们优化下一个子问题，即：

$$
\begin{equation}
\begin{aligned}
\min_{u(N-2), x(N-1)} \quad & \ell(x(N-2), u(N-2))+V_{N-1}^{0}(x(N-1)) \\
\textrm{subject to} \quad & x(N-1)=Ax(N-2) + Bu(N-2)\\
\end{aligned}
\end{equation}
$$
同理可以得到这个问题的解：

$$
\begin{aligned}
u_{N-2}^{0}(x) &=K(N-2) x \\
x_{N-1}^{0}(x) &=(A+B K(N-2)) x \\
V_{N-2}^{0}(x) &=(1 / 2) x^{T} \Pi(N-2) x \\
K(N-2) &=-\left(B^{T} \Pi(N-1) B+R\right)^{-1} B^{T} \Pi(N-1) A \\
\Pi(N-2) &=Q+A^{T} \Pi(N-1) A-\\
& A^{T} \Pi(N-1) B\left(B^{T} \Pi(N-1) B+R\right)^{-1} B^{T} \Pi(N-1) A
\end{aligned}
$$
以此类推，最终我们可以得到一个重要的递推公式：

$$
\Pi(k-1)=Q+A^{T} \Pi(k) A-A^{T} \Pi(k) B\left(B^{T} \Pi(k) B+R\right)^{-1} B^{T} \Pi(k) A， \\
k=N, N-1, \ldots, 1
$$
这个递推公式的终止条件为 $\Pi(N)=P$。

每个阶段的最优输入可以用 $\Pi$ 来表示：

$$
u_{k}^{0}(x)=K(k) x \quad k=N-1, N-2, \ldots, 0\\
K(k)=-\left(B^{T} \Pi(k+1) B+R\right)^{-1} B^{T} \Pi(k+1) A \quad k=N-1, N-2, \ldots, 0
$$
最优值函数为：

$$
V_{k}^{0}(x)=(1 / 2) x^{T} \Pi(k) x \quad k=N, N-1, \ldots, 0
$$
这个最优值函数很有意思，它代表了从第$k$个阶段代价以后的所有代价函数的最优值。

### 无限阶段 （infinite horizon）的 LQR 
当 horizon 从$N$不断增大，知道无穷的时候，finite horizon LQR 就延伸成了 infinite horizion LQR，优化问题如下：
$$
\begin{equation}
\begin{aligned}
\min_{\boldsymbol{u}_{N}} \quad & V_{N}\left(x_{0}, \boldsymbol{u}_{N}\right)= \sum_{k=0}^{\infty}\{x(k)^T Q x(k) + u(k)^T R u(k)\}\\
\textrm{subject to} \quad & x(0)=x_{0} \\
& x(k+1)=Ax(k) + Bu(k), \forall k\in \{0, ..., N-1\} \\
\end{aligned}
\end{equation}
$$
现在我们把 Finite horizon LQR 推出的解推广到 Infinite horizon 的情况。当 $k$ 趋近于无穷的时候，有 $\Pi(k-1)=\Pi(k)$，若我们把 $\Pi(k)$ 统一用 $\Pi$ 表示，可以得到：
$$
\Pi=Q+A^{T} \Pi A-A^{T} \Pi B\left(B^{T} \Pi B+R\right)^{-1} B^{T} \Pi A
$$
这就是离散代数黎卡提方程 Discrete Algebraic Riccati Equation (DARE)。通过解这个方程，我们就可以得到 Infinite horizon LQR 的最优输入，即：
$$
u^{0}(x)=K x \\ 
K=-\left(B^{T} \Pi B+R\right)^{-1} B^{T} \Pi A
$$
此时的最优代价函数为：
$$
V^{0}(x)=(1 / 2) x^{T} \Pi x
$$
## 模型预测控制（MPC）
MPC 的优化问题的形式如下：
$$
\begin{equation}
\begin{aligned}
\min_{\boldsymbol{u}_{N}} \quad & V_{N}\left(x_{0}, \boldsymbol{u}_{N}\right)= \sum_{k=0}^{N-1}\{\ell(x(k), u(k))\}+V_{\mathrm{f}}(x(N)) \\
\textrm{subject to} \quad & x(0)=x_{0} \\
& x(k+1)=f(x(k), u(k)), \forall k \\ & (x(k), u(k)) \in \mathbb{Z}, \forall k \\ 
& x(N) \in \mathbb{X}_{\mathrm{f}}\\
\end{aligned}
\end{equation}
$$

和 Finite horizon LQR 相比，MPC 有几个区别：
- 可以处理非线性模型。
- 当模型是线性的时候，可以通过设计合理的终端约束（terminal constraint) 和终端代价（terminal cost）来保证系统稳定性（stability）。具体的证明在之后的章节会讲到。
- 可以根据任务来设计各种各样的约束，比如可以实现动态避障。

## 总结
在本文中，我们介绍了 finite horizon LQR、infinite horizon LQR、MPC。那么这三种方法孰优孰劣呢？请见下表：
| Method | Stability | Constraints | Model |
| ----------- | ----------- | ----------- | ----------- |
| Finite horizion LQR | No Guarantee| -- | Linear |
| Infinite horizon LQR | Guarantee | Computational intractable |  Linear |
| MPC | (linear case) Guarantee | Yes | (non)linear |