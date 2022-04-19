# 1. 轨迹跟踪问题（reference tracking）
之前几章我们考虑的问题主要是最小化状态变量，即考虑一个不变的吸引子（Attractor）$A\in\mathbb{R}^n$并控制系统使状态变量最终进入吸引子中，然而显然MPC的用途不止于此。实际上我们可以通过事先给定一条轨迹，通过MPC控制系统使状态变量跟上预设的轨迹。
## 1.1 问题描述与假设
对于轨迹跟踪问题，我们作出如下假设：
- 线性系统
  $$
  \begin{align*}
  x^+&=Ax+Bu\\
  y&=Cx
  \end{align*}
  $$ 
- 状态变量 $x$ 可以被完全精准测量
- $(A,B)$系统可控

基于上述假设，我们可以将问题描述为：寻找$(\textbf{x}_{ref},\textbf{u}_{ref})\in \mathbb{X}\times\mathbb{U}$ 使得 $C\textbf{x}_{ref}=\textbf{y}_{ref}$，其中$\mathbb{X}$ 和 $\mathbb{U}$表示状态约束和控制约束。

## 1.2 优化目标选择（Optimal target selection）
根据上述问题描述，我们可以将这个问题写成一个优化问题的形式：
$$
(x_{ref},u_{ref})(y_{ref})\in\begin{cases}
 \argmax\limits_{x_r,u_r} J(x_r,u_r)\\ \\
 s.t.\ \begin{bmatrix}
 I-A & -B\\
 C   & 0
\end{bmatrix}\begin{bmatrix}
x_r \\
u_r
\end{bmatrix}=\begin{bmatrix}
0 \\
y_r
\end{bmatrix}\\ \\
(\textbf{x}_{ref},\textbf{u}_{ref})\in \mathbb{X}\times\mathbb{U}\\
Cx_r\in\mathbb{Y}
\end{cases}
$$
其中代价函数 $J$ 可以是我们自定义的任意代价函数。Optimal target selection(OTS)问题在上述假设条件下可以在控制开始之前被线下解决（offline），解出$(\textbf{x}_{ref},\textbf{u}_{ref})$后，将MPC的代价函数修改为：
$$
V_N(x_0,y_{ref},u_N)=\sum^{N-1}_{k=0}\{l(x(k)-x_{ref},u(k)-u_{ref})\}+V_f(x(N)-x_{ref})
$$