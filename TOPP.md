# TOPP

时间最优路径参数化（time-optimal path parameterization），是一种对**已有的特定路径**进行规划的方法。

> 在通常情况下，规划往往可以分为两个方法：**路径**规划和**轨迹**规划，路径指一个几何曲线或离散的一系列坐标点，而轨迹则是带有时间信息的曲线或离散点。
>
> 路径规划的结果往往也需要经过**时间参数化**才能被控制器所用，而轨迹规划也往往需要输入特定路径再对其参数化。但这并非绝对，一些优秀的控制器可以直接跟踪路径（MPCC，模型预测轮廓控制），而有些规划器能利用少数的航路点直接生成带时间参数的轨迹（空间走廊法、多项式轨迹、CCP，进度约束）。
>
> TOPP是建立在路径-轨迹两步结构上的，将原有的路径时间参数化替换为其他的参数化，替代的是其中的轨迹规划部分。

TOPP将沿路径的距离作为基础变量$\theta$，状态向量以此参数化，进而将动力学约束和控制器约束转换为沿路劲加速度的状态依赖性约束，从而将最优时间参数化问题转换为**带有非线性状态依赖约束的二阶线性系统时间最优问题**。其时间最优性通过优化出**产生最大速度曲线的加速度曲线**保证。

>关键因素在于状态依赖约束的优越性。
>
>时间参数下，动力学方程往往以$\dot x(t)=f(x(t),u(t))$体现，对于无人机$[x|u]=[p,\dot p,q,\dot q|T_{1-4}]$，其方程更加复杂；三轴机械臂$[x|u]=[\theta,\dot \theta|T_{1-3}]$。其中的每一个变量都是关于时间变化的。
>
>问题是：时间本身是优化变量，但时间与动力学约束的关系是**非常间接**。以梯度下降优化方法为例，要求取代价$\dot x-f(x,u)$对时间的导数并不过于困难，但对包含动力学约束的时间代价$T=\sum dt$求取关于$dt$的导数则得不到显式，更非凸优化问题，如果没有一个已经足够接近的初始值几乎无法求解。
>
>如果要从优化问题本身下手，就要保证时间代价的同时使其与动力学约束相关联，因此一个自然的想法就是时间空间参数化：将**时间参数空间化**。特别的，对于已有特定路径的情况，可以直接参数化为路径的**进度**，或更具体的弧长（距离）。由此求解关于时间的导数非常容易，而动力学约束也可以只在空间层面上完成。

动力学约束可以写为通式：
$$
\begin{aligned}&\mathbf{M}(\mathbf{q})\ddot{\mathbf{q}}+\dot{\mathbf{q}}^\top\mathbf{C}(\mathbf{q})\dot{\mathbf{q}}+\mathbf{g}(\mathbf{q})=\tau,\\&\tau_{i}^{\mathrm{min}}\leq\tau_{i}(t)\leq\tau_{i}^{\mathrm{max}},\forall i\in[1,\ldots,n],t\in[0,T]\\\Rightarrow
&\mathbf{A(q)\ddot{q}+\dot{q}^\top B(q)\dot{q}+f(q)\in\mathscr{C}(q)}\\
&\mathscr{C}(\mathbf{q}):=\mathbf{S}^\top\left([\tau_1^{\min},\tau_1^{\max}]\times\cdots\times[\tau_n^{\min},\tau_n^{\max}]\right)
\end{aligned}
$$

>动力学约束是通过拉格朗日力学方程推导而来的。根据物理学定律，系统始终向着能量下降的方向运动，但总的孤立系统能量守恒，那么就可以用能量替代力描述系统；在动力学内，只考虑动能和势能，易得能量$L=K-P$，关于速度$\dot p$的偏导数的导数$\frac{d}{dt}\frac{\partial L}{\partial\dot{p}}=\frac{d}{dt}\frac{\partial K}{\partial\dot{p}}$，其量纲为力；关于坐标$p$的偏导数$\frac{\partial L}{\partial{p}}=-\frac{\partial P}{\partial{p}}$量纲也为力，即两个能量的关于时间的导数。于是能量守恒定理对应受力平衡：
>$$
>\frac{d}{dt}\left(\frac{\partial L}{\partial\dot{p}}\right)-\frac{\partial L}{\partial p}=Q
>$$
>在进行动力学状态空间建模时，状态变量总是设定为位置和位置的导数，而控制量对应非保守力，因此拉格朗日方程和状态空间微分方程完全等价。

原时间参数$\mathbf{q}$，空间参数$s$，有映射$\mathbf{q}(s),\dot{\mathbf{q}}=\mathbf{q}^{\prime}\dot{s},\quad\ddot{\mathbf{q}}=\mathbf{q}^{\prime\prime}\dot{s}^2+\mathbf{q}^{\prime}\ddot{s}$，则上式全部改写为：
$$
\begin{aligned}&\mathbf{a}(s)\ddot{s}+\mathbf{b}(s)\dot{s}^{2}+\mathbf{c}(s)\in\mathscr{C}(s),\mathrm{where}\\&\mathbf{a}(s):=\mathbf{A}(\mathbf{q}(s))\mathbf{q}^{\prime}(s),\\&\mathbf{b}(s):=\mathrm{A}(\mathbf{q}(s))\mathbf{q}^{\prime\prime}(s)+\mathbf{q}^{\prime}(s)^\top\mathrm{B}(\mathbf{q}(s))\mathbf{q}^{\prime}(s),\\&\mathbf{c}(s):=\mathbf{f}(\mathbf{q}(s)),\\&\mathscr{C}(s):=\mathscr{C}(\mathbf{q}(s)).\end{aligned}
$$
空间参数可以直接由路径离散化得到，由于需要同时得到$s,\dot s,\ddot s$，因此离散化时需要满足连续性约束。由于动力学建模只考虑二阶，因此连续性也只考虑二阶，即认为离散点间恒加速度。连续性约束即满足：
$$
\\\begin{aligned}
&\Delta s_i=\frac{(\dot s_i+\dot s_{i+1})\Delta t_i}{2}\\
&\Delta\dot s_i=\Delta t_i \ddot s_i\\
\end{aligned}\Rightarrow\dot s_{i+1}^2=\dot s_i^2+2(s_{i+1}-s_i)\ddot s_i\to x_{i+1}=x_i+2\Delta_iu_i\\
$$
至此，约束被完全转化为了状态依赖的非线性约束，完整问题如下：
$$
\min_{s}t_f=\int_{s_0}^{s_f}\frac{1}{\dot{s}}ds\\
object:\mathbf{a}_iu_i+\mathbf{b}_ix_i+\mathbf{c}_i\in\mathscr{C}_i,\\\mathrm{where~}\mathbf{a}_i:=\mathbf{a}(s_i),\mathbf{b}_i:=\mathbf{b}(s_i),\mathbf{c}_i:=\mathbf{c}(s_i),\mathscr{C}_i:=\mathscr{C}(s_i).\\
$$

直接采用bang-bang控制的思路，对每个离散点，可以得到可行域$\mathcal{K}$。由于**无法由闭式解求出切换时间，只能通过数值计算得到可行域**，因此采用“图解法”：从起点设最大加速度前向计算、从终点设最大减速度后向计算、二者交点即为切换点，由此就确定了最优轨迹，在相平面中，如果起点不在曲线上需采用最大加、减速度抵达曲线即可：
$$
\begin{aligned}
&f(s,\:\dot{s})=\max_{i}f_{i}(s,\:\dot{s})\\
&g(s,\:\dot{s})=\min_{i}g_{i}(s,\:\dot{s})\\
&f(s,\dot{s})\leqslant\ddot{s}\leqslant g(s,\dot{s})\to(s,\dot{s})\in\mathcal{K}\\
\end{aligned}
$$
然而，直接套用前后向相交获得的最优曲线不一定可行，因为加速度的上、下界因为约束而不定，在速度足够打的时候有可能出现下界大于上界的情形。因此切换加速度上、下界的时机不能如bang-bang控制一样只有一次，而且还需要考虑加速度上下界之间的关系：

- 保证在可行域内，同时考虑最大速度曲线；
- 使用bang-bang控制，只有最大控制量和最小控制量两种模式；
- 使得$\int_{s_0}^{s_f}\dot{s}ds$尽量大（直接体现为相平面积分），即与后向曲线相交前，尽可能使用加速度曲线。

那么达成最小时间的方法就是：尽可能沿着可行域边界行动。受限于动力学约束，往往无法沿可行域的边界，因此尽量与边界相切；而在首尾处仍使用前向和后向计算保证可达。

然而，即使简单的单切换控制都无法求得闭式解，对多切换更无法求解，甚至难以使用数值算法。因此采用了一种更工程化的非标准方法：

- 构建常规的前向曲线和后向曲线，保证可达；
- 如果前向曲线与可行域边界相交，对交点作以下处理：
  - **迭代**降低该点速度（相平面上向下平移），直到该点所在的最大减速度曲线与可行域相切；
  - 减速度曲线自该点延伸与前向曲线相交，得到第一个切换点。
- 与可行域的切线总是加速度和减速度曲线的交点，如果还没与后向曲线相交，则切换为加速度曲线；
  - 如果加速度曲线再次与可行域边界相交，与前步骤同样处理；
  - 如果曲线与后向曲线相交，则得到最后一个切换点，完成曲线。

- 对于最优曲线下方初始点，只需要由最大加、减速度曲线抵达曲线即可；同理，初末速度也可以不为零。

在整个过程中，除了动力学约束外只考虑了最大速度曲线约束，但是仍然可以以此类推，将其他需要的约束转变为相平面曲线，用类似的方法处理.

不管如何，计算前后向曲线、迭代寻找切线时计算曲线都需要数值积分或凸优化，而路径切换点则带来了巨大的风险（尤其是数值积分法），当切换点是动态奇异点（零惯性点）时，会造成计算风险乃至求解失败，这要求了更为高级的计算方法、或原约束求解还需要进一步的改进。

**基于可达性分析的时间最优路径参数化**（**TOPP-RA**）求出。除了最大速度以外，显而易见地$s$还应该满足$x_{i+1}=x_i+2\Delta_iu_i$的约束，这个约束称为成为可达性。这个约束只在初始化路径时使用，在后续中内涵在更为复杂的动力学约束中。与可行域对应，对状态集$\mathbb{I}$，，定义直达域(reach set)$\mathcal{R}$可达域(reachable set)$\mathcal{L}$:
$$
\mathcal{R}_i(\mathbb{I})=(x^{-},x^{+})\\
x^{-}:=\min_{(u,\tilde{x})\in\Omega_{i}(\mathbb{I}),x^{-}\in\mathcal{X}_{i+1}}\tilde{x}+2\Delta_{i}u,\\
x^{+}:=\max_{(u,\tilde{x})\in\Omega_{i}(\mathbb{I}),x^{+}\in\mathcal{X}_{i+1}}\tilde{x}+2\Delta_{i}u.\\
\mathrm{subject~to:~a}_iu+\mathrm{b}_i\tilde{x}+\mathrm{c}_i\in\mathscr{C}_i,\tilde{x}\in\mathbb{I}\mathrm{~and~}x^+,x^-\in\mathcal{X}_{i+1},u\in\mathcal{U}_{i}(\tilde{x})\\\\
\begin{aligned}&\mathcal{L}_0(\mathbb{I}_0)=\mathbb{I}_0\cap\mathcal{X}_0,\\&\mathcal{L}_i(\mathbb{I}_0)=\mathcal{R}_{i-1}(\mathcal{L}_{i-1}(\mathbb{I}_0)).\end{aligned}\\
\mathcal{L}_i(\mathbb{I}_0)=\emptyset\Longrightarrow\forall j\geq i,\mathcal{L}_j(\mathbb{I}_0)=\emptyset.
$$
类似的单步域$\mathcal{Q}$和控制域$\mathcal{K}$：
$$
\mathcal{Q}_i(\mathbb{I})=(\tilde x^{-},\tilde x^{+})\\
\tilde x^{-}:=\min_{(u,{x})\in\Omega_{i}(\mathbb{I}),\tilde x^{-}\in\mathcal{X}_{i+1}}{x}+2\Delta_{i}u,\\
\tilde x^{+}:=\max_{(u,{x})\in\Omega_{i}(\mathbb{I}),\tilde x^{+}\in\mathcal{X}_{i+1}}{x}+2\Delta_{i}u.\\
\mathrm{subject~to:~a}_iu+\mathrm{b}_i\tilde{x}+\mathrm{c}_i\in\mathscr{C}_i,\tilde{x}\in\mathbb{I}\mathrm{~and~}x^+,x^-\in\mathcal{X}_{i+1},u\in\mathcal{U}_{i}(\tilde{x})\\\\
\begin{aligned}&\mathcal{K}_{N}(\mathbb{I}_{N})=\mathbb{I}_N\cap\mathcal{X}_N,\\
&\mathcal{K}_{i}(\mathbb{I}_{N})=\mathcal{Q}_i(\mathcal{K}_{i+1}(\mathbb{I}_N)).\\
\end{aligned}\\
\mathcal{K}_i(\mathbb{I}_N)=\emptyset\Longrightarrow\forall j\leq i,\mathcal{K}_j(\mathbb{I}_N)=\emptyset.
$$
即可达域和控制域分别替代了前向和后向曲线，实现则分别计算出可达集和控制集，再直接贪心获取域内最优路径，最后两条路径取最小速度交集即得。其中，总状态域$\Omega_{i}(\mathbb{I})$无需显式，由于约束的存在${(u,{x})\in\Omega_{i}(\mathbb{I}),\tilde x^{-}\in\mathcal{X}_{i+1}}$这一要求可以化为$(u,\tilde x)\in \mathbb{R}^2$；同时，也根据可达定义设计了早停条件。

在具有更高鲁棒性和计算效率的基础上，TOPP-RA与TOPP-NI的最优性是一致的，其本质区别是计算的步骤有所不同：

可以通过数学证明，对于孤立的零惯性点，代价可以通过减小步长收敛到最优，即满足渐进最优性。

>