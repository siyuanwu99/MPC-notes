# Lec 5 观测器设计和扰动估计

## 5.1 轨迹跟踪问题（reference tracking）

之前几章我们考虑的问题主要是最小化状态变量，即考虑一个不变的吸引子（Attractor）$A\in\mathbb{R}^n$并控制系统使状态变量最终进入吸引子中，然而显然MPC的用途不止于此。实际上我们可以通过事先给定一条轨迹，通过MPC控制系统使状态变量跟上预设的轨迹。

### 5.1.1 问题描述与假设

对于轨迹跟踪问题，我们作出如下假设：

- 线性系统
  $$
  \begin{align*}
  x^+&=Ax+Bu\\
  y&=Cx
  \end{align*}
  $$
- 状态变量 $x$ 可以被完全精准测量
- $(A,B)$系统能控

基于上述假设，我们可以将问题描述为：寻找$(\textbf{x}_{ref},\textbf{u}_{ref})\in \mathbb{X}\times\mathbb{U}$ 使得 $C\textbf{x}_{ref}=\textbf{y}_{ref}$，其中$\mathbb{X}$ 和 $\mathbb{U}$表示状态约束和控制约束。

### 5.1.2 优化目标选择（Optimal target selection）

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
其中代价函数 $J$ 可以是我们自定义的任意代价函数。假设MPC所使用的模型与实际模型完全一致没有任何误差，则Optimal target selection(OTS)问题可以在控制开始之前被线下解决（offline），即提前算好每一步的$(\textbf{x}_{ref},\textbf{u}_{ref})$。

前面几章我们已经讨论过终端集和终端代价函数的设计，这里由于目标不再是原点，我们需要对终端集和代价函数进行相应的调整。MPC的代价函数会被修改为：
$$
V_N(x_0,y_{ref},u_N)=\sum^{N-1}_{k=0}\{l(x(k)-x_{ref},u(k)-u_{ref})\}+V_f(x(N)-x_{ref})
$$

对于终端集，我们需要在原来终端集的基础上加上 $x_{ref}$ 的偏置，通过这个偏置，之前所证明的稳定性在这里也同样适用，这一章里就不再赘述了。

<div align=center>
    <img width="380" src=figures/lec0501.png />
</div>

## 5.2 全状态反馈下的偏置误差估计(Offset-free MPC with state feedback)

上述问题中我们并没有考虑模型的噪声(Noise)和偏置误差(Disturbance)，这一节中我们将学习通过MPC来估计偏置误差并消除其对控制的影响。首先我们对噪声和误差进行定义与建模，其中噪声指期望为0的随机变量，而偏置指一个未知的常数变量。举例来说，噪声可以出现在传感器上，假设我需要测量一个瓜有多重，由于噪声的影响，我有可能得到一个比实际重量偏大的值，也有可能得到偏小的值，但是如果我测量了无限多次，这个误差的期望会是0。而偏置误差则是指我在称上粘了吸铁石，测量重量始终比实际重量大，并且我并不知道吸铁石有多重，通过设计MPC我们就能对吸铁石的重量进行估计。

<div align=center>
    <img width="380" src=figures/lec0502.png />
</div>

我们分别用 $d$ 和 $v$ 来表示偏置和噪声，其中 $d$ 是常数并且 $\mathbb{E}[v(t)=0]$，并且我们目前只考虑测量阶段的误差，系统模型可以被改写为:
$$
\begin{align*}
  x^+&=Ax+Bu\\
  y&=Cx+d+v
\end{align*}
$$

由于我们需要测量 $d$ 的值，这里的整体思路是**将 $d$ 假设为是一个状态变量**，通过设计对状态变量的观测器来取得 $d$ 的真实值。这里扩展之后的系统(Augmented system)可以被写为：
$$
\begin{align*}
\begin{bmatrix}
x^+ \\
d^+
\end{bmatrix}&=\begin{bmatrix}
 A & 0\\
 0 & I
\end{bmatrix}\begin{bmatrix}
x \\
d
\end{bmatrix}+\begin{bmatrix}
B \\
0
\end{bmatrix}u\\

y&=\begin{bmatrix}
  C & I
\end{bmatrix}\begin{bmatrix}
  x \\ d
\end{bmatrix}+v
\end{align*}
$$

针对这个系统，我们设计一个龙贝格观测器(Luenberger Observer)来估计 $d$ 的值。在观测器中引入变量 $\hat{d}$ 为偏置误差的估计值并通过以下方程来更新:

$$
\hat{d^+}=\hat{d}+L(y-Cx-\hat{d})
$$

这里我们给 $\hat{d}$ 赋予一个初值，在每一步状态更新之后，我们通过 $\hat{d}$ 得到一个预估的输出：$\hat{y}=Cx+\hat{d}$，通过比较估计输出和实际输出来调整 $\hat{d}$ 的值。只要适当配置**观测增益矩阵** $L$ 我们就能让 $\hat{d}$ 收敛到 $d$，下面我们来推导如何选择观测增益矩阵。我们将对 $d$ 的**估计误差**记为 $\tilde{d}=d-\hat{d^+}$，则有:
$$
\begin{align*}
\tilde{d}^+=d-\hat{d^+}&=d-\hat{d}-L(y-Cx-\hat{d})\\
&=\tilde{d}-L(d-\hat{d}+v)\\
&=(I-L)\tilde{d}-Lv
\end{align*}
$$
由于我们已经假设噪声的期望为0，为了得到最佳估计值($\tilde{d}$收敛于0)，我们只需要证明矩阵 $I-L$ 是“稳定”的。
> 对于连续系统 $\dot{x}=Ax$ ，稳定的充要条件为矩阵 $A$ 的特征值都有非正实部，而对于离散系统 $x^+=Ax$ ，稳定的充要条件为矩阵 $A$ 的所有特征值模长都小于等于1

我们将上述观测器与之前的OTS问题相结合，OTS问题可以被重新写为以下格式：
$$
(x_{ref},u_{ref})(\hat{d}, y_{ref})\in\begin{cases}
 \argmax\limits_{x_r,u_r} J(x_r,u_r)\\ \\
 s.t.\ \begin{bmatrix}
 I-A & -B\\
 C   & 0
\end{bmatrix}\begin{bmatrix}
x_r \\
u_r
\end{bmatrix}=\begin{bmatrix}
0 \\
y_r-\hat{d}
\end{bmatrix}\\ \\
(\textbf{x}_{ref},\textbf{u}_{ref})\in \mathbb{X}\times\mathbb{U}\\
Cx_r+\hat{d}\in\mathbb{Y}
\end{cases}
$$
此时OTS问题需要被在线解决(online)，因为我们并不能提前知道整个控制过程中的 $\hat{d}$ 的值，这需要每一步迭代计算才能得到。最终得到整个控制流程图如下：
<div align=center>
    <img width="380" src=figures/lec0503.png />
</div>

## 5.3 输出反馈下的偏置误差估计(Offset-free MPC with Output feedback)

上一节我们探讨了如何在全状态反馈的情况下估计偏置误差，然而大部分情况下我们都无法准确测量状态变量 $x$ 的值，而只能得到系统输出的值 $y$， 这一节我们将探讨如何在输出反馈的情况下估计偏置误差。这一节中我们将假设状态转移过程中也有噪声和偏置误差，此时系统可以写成如下形式：
$$
\begin{align*}
  x^+&=Ax+Bu+B_dd+w\\
  y&=Cx+C_dd+v
\end{align*}
$$
系统有如下假设：

- 状态变量 $x$ 无法被直接测量
- $(A,B)$为能控系统，$(A,C)$为能观系统(observable)
- $d$ 为未知的常数误差
- $v$ 和 $w$ 为期望为0的噪声

和前一节相似，我们将偏置作为新的变量加入到状态空间中:
$$
\begin{align*}
\begin{bmatrix}
x^+ \\
d^+
\end{bmatrix}&=\begin{bmatrix}
 A & B_d\\
 0 & I
\end{bmatrix}\begin{bmatrix}
x \\
d
\end{bmatrix}+\begin{bmatrix}
B \\
0
\end{bmatrix}u\\

y&=\begin{bmatrix}
  C & C_d
\end{bmatrix}\begin{bmatrix}
  x \\ d
\end{bmatrix}+v
\end{align*}
$$
这里需要注意的是扩展后的状态空间不一定是能观的,针对这个问题我们有如下引理：
> 扩展状态空间能观等价于：
>
> - 原状态空间$(A,C)$能观；
> - $rank\begin{bmatrix}I-A & -B_d\\C&C_d\end{bmatrix}=n+n_d$

其中 $n_d$ 为偏置误差的维度，即有几个独立的偏置误差变量。由于第二个条件中，左边矩阵的秩最大为 $n+n_y$，$n_y$ 为输出变量的维度，因此可以预估的偏置误差的个数最多不能超过输出变量 $y$ 的纬度。根据之前的假设，原状态空间为能观的，因此我们只需要验证第二个条件是否满足。如果第二个条件无法满足，则我们**无法**通过配置状态观测器来估计偏置误差。

确认扩展后的状态空间能观后，我们就能开始设计观测器了。与之前不同，这里我们不仅需要重构偏置误差的值，也需要重构状态变量的值，因此这里需要设计将状态变量和偏置误差都包括进来的龙贝格观测器：
$$
\begin{align*}
\begin{bmatrix}
\hat{x}^+ \\
\hat{d}^+
\end{bmatrix}&=\begin{bmatrix}
 A & B_d\\
 0 & I
\end{bmatrix}\begin{bmatrix}
\hat{x} \\
\hat{d}
\end{bmatrix}&+\begin{bmatrix}
B \\
0
\end{bmatrix}u&+\begin{bmatrix}
  L_1\\L_2
\end{bmatrix}(y-\begin{bmatrix}
  C & C_d
\end{bmatrix}\begin{bmatrix}
  \hat{x}\\
  \hat{d}
\end{bmatrix})\\

&=\tilde{A}\begin{bmatrix}
\hat{x} \\
\hat{d}
\end{bmatrix}&+\begin{bmatrix}
B \\
0
\end{bmatrix}u&+\tilde{L}(y-\begin{bmatrix}
  C & C_d
\end{bmatrix}\begin{bmatrix}
  \hat{x}\\
  \hat{d}
\end{bmatrix})
\end{align*}
$$
为了使观测器收敛，我们需要配置 $\tilde{L}$ 矩阵使得矩阵 $\tilde{A}-\tilde{L}\tilde{C}$ 是稳定的，即所有特征值模长都小于等于1。至此我们就完成了观测器的设计。结合之前的OTS问题，此时的控制流程图如下：
<div align=center>
    <img width="380" src=figures/lec0504.png />
</div>

## 5.4 参考

[1] S. Grammatico, “Lecture 5. model predictive control” 2022

[2]G. Pannocchia and J. Rawlings, “Disturbance Models for Offset-Free Model-Predictive Control,” AIChE Journal, vol. 49, pp. 426–437, Feb. 2003, doi: 10.1002/aic.690490213.

[3] Rawlings, J. B., Mayne, D. Q., & Diehl, M. (2017). Model predictive control: Theory, computation, and design.
