# Minimum Snap Trajectory Generation and Control for Quadrotors

这是一篇经典论文，通过位置和偏航角序列实时生成最优轨迹，同时确保通过指定走廊的安全通行，并满足对速度、加速度和输入的约束，即：
$$
generate\quad min||u||\\
s.t.\quad\{p\},\{yaw\},obs,|v|<v_{max},|a|<a_{max},|u|<u_{max}\\
$$

>LQR-tree是什么？

其中，动力学表示为：
$$
x=[p,q,\dot p,\dot q]^T_{dim=12}\\
u=\begin{bmatrix}k_F&k_F&k_F&k_F\\0&k_FL&0&-k_FL\\-k_FL&0&k_FL&0\\k_M&-k_M&k_M&-k_M\end{bmatrix}\begin{bmatrix}\omega_1^2\\\omega_2^2\\\omega_3^2\\\omega_4^2\end{bmatrix}\\
ma=-mg\mathbf{z}_W+u_1\mathbf{z}_B\\
\dot{\omega}_{\mathcal{BW}}=\mathcal{I}^{-1}\left[-\omega_{\mathcal{BW}}\times\mathcal{I}\omega_{\mathcal{BW}}+\begin{bmatrix}u_2\\u_3\\u_4\end{bmatrix}\right]
$$
我们希望尽可能减少表示轨迹全部信息的未知量，以简化优化过程。根据微分平坦理论，轨迹只需要四个量及其各阶导数即可表示其他所有量，由于位置是首要被考虑的，其次是姿态，因此选用：
$$
\sigma = [x,y,z,\psi]^T_{dim=4},即\sigma(t)\rightarrow \mathbb{R}^3\times SO(2)\\
\min||u||\to \min||\ddddot p||+||\ddot q||\to \min \sigma^T H\sigma+f^T\sigma,\quad H_{3j,i3}=0,f_{0-2}=0
$$
设计系数矩阵H和系数f的过程被称为无量纲化。由于路径的好坏和路径的总长度、总时间没有关系，为了避免路径越长代价越大，可以利用将任意时间的路径都投影到[0,1]区间，这样可以用同样的标准衡量不同长度的路径，也能使得优化各项间的比例不改变。

投影是一个线性变换的过程，可以分解为空间缩放和时间缩放两步，使用待定系数法反解；先针对某一个元素分析，然后再拓展到整个向量：
$$
\int_0^\alpha\frac{d^kw(t)}{dt^k}dt\to^{proj} \int_{0}^{1}\frac{d^{k}\tilde{w}(\tau)}{d\tau^{k}}^{2}d\tau,\quad need\quad \tilde{w}(\tau)\\
define\quad w(t)=w(\alpha\tau)=\beta_1+\beta_2\tilde{w}(\tau)\\
\int_{0}^{1}\frac{d^{k}\tilde{w}(\tau)}{d\tau^{k}}^{2}d\tau=\int_0^\alpha(\frac{\alpha^{k}}{\beta_2}\frac{d^kw(\alpha\tau)}{d\tau^k})^2d\tau=\frac{\alpha^{2k-1}}{(\beta_2)^2}\int_0^\alpha\frac{d^kw(t)}{dt^k}dt\\
其中：\beta_1=w_0,\beta_2=w_f-w_0,a=t_f
$$
由此，前两项约束的表达形式就已经得到，接下来对障碍的处理需要其他的计算方法。论文将避障问题简化为走廊约束，即在对应段中需要保证轨迹与走廊的垂直距离不小于阈值。这就需要求取垂线段距离以及如何生成可解的二次项代价：
$$
corridor:distence_i(\mathbf{r}_T(t),r_{i+1}-r_{i})
<\delta_i \to ||d_i(t)||_{\infty}<\delta_i\\
\mathbf{d}_i(t)=(\mathbf{r}_T(t)-\mathbf{r}_i)-((\mathbf{r}_T(t)-\mathbf{r}_i)\cdot\mathbf{e}_{r_i\to r_{i+1}})\mathbf{e}_{r_i\to r_{i+1}}\\
\left|\mathbf{x}_W\cdot\mathbf{d}_i\left(t_i+\frac{j}{1+n_c}(t_{i+1}-t_i)\right)\right|\leq\delta_i\mathrm{~for~}j=1,...,n_c\\
$$
代价项使用插点法，走廊等时采样$\frac{1}{1+n_c}(t_{i+1}-t_i)$，满足各点处都满足三轴上的距离约束。然而时间约束还需要进一步的确定，为了轨迹顺利生成至少需要提供总的时间，在此基础上对每段作时间分配。然而时间分配不仅需要满足总时间约束，还可能需要满足特定段时间约束，此时可以用经过设计的梯度下降法求解：
$$
\min f(T),\quad \nabla_{g_i}f=\frac{f(T+hg_i)-f(T)}{h}\\
如果总时间保持不变，则\sum_{i=0}^m g_i=0;如果指定某段T_i不变，则g_i=1,其余平均;
$$
