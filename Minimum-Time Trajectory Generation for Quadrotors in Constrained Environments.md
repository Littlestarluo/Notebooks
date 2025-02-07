# Minimum-Time Trajectory Generation for Quadrotors in Constrained Environments（约束环境下四旋翼无人机的最小时间轨迹生成方法，2017）

fixed-horizon reformulation 空间参数（transverse distance）作为自变量，参数化坐标位置，相比于时间参数化更容易解决路点约束问题。支持投影算子牛顿法PRONTO和障碍函数方法（【22】【23】）能求解无限维数值解。从而模型不依赖微分平坦，也无需简化（线性化、降低阶数）。

提供了时间最优相对完整的表述：
$$
\min_{x(\cdot),u(\cdot),T} T，使得:\\
&动力学约束：\dot x(t)=f(x(t),u(t)),x(0)=x_0(dynamics)\\
&终点约束：x(T)\in X_T\\
&roll约束:|p(t)|\leq p_{max}\\
&pitch约束:|p(t)|\leq p_{max}\\
&yaw约束:|p(t)|\leq p_{max}\\
&roll变化率约束:|p(t)|\leq p_{max}\\
&pitch变化率约束:|p(t)|\leq p_{max}\\
&yaw变化率约束：|p(t)|\leq p_{max}\\
&推力约束：0<F_{min}\leq F(t)\leq F_{max}\\
&无碰撞：c_{obs}(p(t))\leq 0\\
$$
而所谓的空间参数实际上思路和后来的MPCC相同，都是将路劲化为弧长单位，其中选用横坐标作为其自变量，而当前路劲积分（长度）就是进度。这样，时间最优问题表示为了最小化在[0,L]上的积分函数；同时，点约束也可以轻松标识而不需要时间作为中介。

与MPCC相同，参数化是为首要问题，即如何用s标识轨迹、位置和动力学。论文没有给出参数化轨迹、找到轨迹上最近点的方法，即假定了已知，利用轨迹+横向距离参数化无人机位置，即无人机的位置通过与轨迹最近点来表示。特殊的是，论文使用了更加切合进度的坐标系：Serret-Ferrari系，并在此基础上重写了整个优化问题（代价函数和约束），并且使用PROTON作为数值计算方法更新x、u的值。

##  Serret-Ferrari系

以进度作为变量实现全状态与空间耦合，那么能否进一步建立“曲线系”，使得弧长方向（切线）总是对应同一个轴，这样进度就等同于一个新三维系的横坐标。这样，各个状态只需要进行坐标转换即可实现进度依赖的空间参数化。

更具要求，可以轻易确定切线和曲率两个轴方向，再通过叉积得到第三轴，记得到了新坐标。然而，空间曲线的曲率如何确定？如果与切线垂直的曲率方向约定为与原点到无人机当前位置的向量共面，而由于每个时刻的位置都是通过无人机到轨迹最短距离确定，因此无人机到该点向量必然是垂直的，即曲率方向又与之共线，此时无人机的位置在转换后的切线坐标恒为0，从而只用计算后两个坐标。

这三个向量都是相对于单位矩阵基的空间做变换，因此我们希望能直接将这三个列向量排列出的3×3矩阵作为旋转矩阵使用。首先，切线向量由定义为$[x,0,0]^T$的形式，由于进度为参数的特殊性，对进度s求导恒为1；曲率随曲线变化，与进度无关，因此需要额外除以模长归一化；只要两个向量都为单位长度叉积自然也为单位向量；最后，由于切线向量恒为$[1,0,0]^T$，因此归一化后的矩阵即满足旋转矩阵的格式：
$$
\bar R_{SF}:=[\bar t,\bar n,\bar b]=\begin{bmatrix}\bar p'_f(s),\frac{\bar p''_f(s)}{\bar k(s)},\bar t×\bar n(s)\end{bmatrix}\\
\bar R'_{SF}=\bar R_{SF}
\begin{bmatrix}
0&-\bar k(s)&0\\
\bar k(s)&0&-\bar \tau(s)\\
0&\bar \tau(s)&0
\end{bmatrix}\\
进度s\in[0,L],轨迹 \bar p_f(s),p''_f(s)的二阶范数\bar k(s)=||\bar p''_f(s)||_2,扭矩\bar \tau(s)=\bar n(s)\bar b'(s)\\
（注意1，此处旋转矩阵求导与通常情况[\omega]_×R不同，但原理一样，都是通过利用正交特性构造反对称矩阵）\\
（注意2，易得R\cdot p的第一个元素结果应为p\cdot \bar n=0,因此实际上的旋转矩阵应该是R^T，即此处缺少转置）
$$
因为原点在曲线上移动，因此需要加上平移变换：
$$
d:=\bar R_{SF}^T(s)(p(t)-\bar p_f(s))=[0,\omega_1(t),\omega_2(t)]^T\\
p(t)=[x(t),y(t),z(t)]^T=\bar p_f(s)+\bar R_{SF}(s)d(t)=\bar p_f(s)+p(t)-\bar p_f(s)
$$
至此完整了整个坐标变换推导。那么要把状态$[x,v,\phi],[\omega,F]$全部转换为参数，初状态确定，其余都通过梯度更新，因此其实是需要将状态变量和控制量的梯度给参数化。由于u的两项都是x中变量的微分，因此只需要确定三个状态变量的微分；由于位置坐标变换后切线坐标恒为0，因此只需要两个变量，总的来说只需要计算四个变量微分的参数化。
$$
s_f(t):t\rightarrow s,\bar t(s_f^t)=s\rightarrow t\\
y'(t)=\frac{dy}{dt},\dot y(t)=\frac{dy}{ds}=\frac{dy}{dt}\frac{dt}{ds}=y'(s)\dot s_f^t,y'(s)=\dot y(s) \frac{1}{\dot s^t_f}\\
我们需要的是\frac{dy(s)}{dt},即y'(s),例如w_1'(s)=\bar n(s)^Tv(t)+\bar \tau(s)w_2
$$

## 重写动力学约束

我们的全部目的是：已知上一帧的状态和控制量，需要求得下一帧的状态，而更新利用的是牛顿法，因此需要求得状态对时间的一阶微分；因此需要由上一帧状态和控制量$[w_1,w_2,v,\phi,\omega,F]$表示状态的微分$[\bar{w}_{1}^{\prime},\bar{w}_{1}^{\prime},v',\phi']$.
$$
p(t)=\bar p_f(s)+\bar R_{SF}(s)d(t)\\
p'(t)=v(t)=\bar{p}_{f}^{\prime}\left(s_{f}^{t}\right)\dot{s}_{f}^{t}+\bar{R}_{\mathrm{SF}}^{\prime}\left(s_{f}^{t}\right)\dot{s}_{f}^{t}d(t)+\bar{R}_{\mathrm{SF}}\left(s_{f}^{t}\right)\dot{d}(t).\\
移项，代入d:=[0,\omega_1(t),\omega_2(t)]^T得到：\\\begin{bmatrix}0\\\dot{w}_1(t)\\\dot{w}_2(t)\end{bmatrix}+\dot{s}_f^t\begin{bmatrix}1-\bar{k}\left(s_f^t\right)w_1(t)\\-\bar{\tau}\left(s_f^t\right)w_2(t)\\\bar{\tau}\left(s_f^t\right)w_1(t)\end{bmatrix}-\bar{R}_{\mathrm{SF}}^T\left(s_f^t\right)\mathbf{v}(t)=0\\
每行都是一组方程，分别解得：
\begin{cases}\begin{aligned}\dot{s}_{f}^t&=\frac{\bar{t}\left(s_f^t\right)^T\mathbf{v}(t)}{1-\bar{k}\left(s_f^t\right)w_1(t)}\\\dot{w}_{1}(t)&=\bar{\boldsymbol{n}}\left(s_f^t\right)^T\mathbf{v}(t)+\bar{\tau}\left(s_f^t\right)\dot{s}_f^tw_2(t)\\\dot{w}_{2}(t)&=\bar{\boldsymbol{b}}\left(s_f^t\right)^T\mathbf{v}(t)-\bar{\tau}\left(s_f^t\right)\dot{s}_f^tw_1(t).\end{aligned}\end{cases}\\
带入\bar{w}_{1}^{\prime},\bar{w}_{2}^{\prime},a和\phi原微分式，得：\\
\begin{cases}\begin{aligned}&\bar{w}_{1}^{\prime}=\bar{n}^T\bar{\mathbf{v}}\frac{1-\bar{k}\bar{w}_1}{\bar{t}^T\bar{\mathbf{v}}}+\bar{\tau}\bar{w}_2\\&\bar{w}_{2}^{\prime}=\bar{b}^T\bar{\mathbf{v}}\frac{1-\bar{k}\bar{w}_1}{\bar{t}^T\bar{\mathbf{v}}}-\bar{\tau}\bar{w}_1\\&\bar{\mathbf{v}}^{\prime}=(ge_3-\frac{\bar{F}}{m}R(\bar{\Phi})e_3)\frac{1-\bar{k}\bar{w}_1}{\bar{t}^T\bar{\mathbf{v}}}\\&\bar{\Phi}^{\prime}=J(\bar{\Phi})\bar{\omega}\frac{1-\bar{k}\bar{w}_1}{\bar{t}^T\bar{\mathbf{v}}}.\end{aligned}\end{cases}
$$

## 约束和问题实现

在编程中，各种限值约束都需要写成$x<0$之类的形式，例如在casadi库中优化问题的约束g就是一系列系统变量组成的数组，在运算中保证g恒大/小于零实现约束优化。论文提供了处理约束的思路：
$$
\left(\frac{\bar{p}(s)}{p_{\max}}\right)^2-1\leq0;\\
\left(\frac{2\bar{F}(s)-(F_{\max}+F_{\min})}{(F_{\max}-F_{\min})}\right)^2-1\leq0\\
\Rightarrow\left||\bar{F}(s)-\frac{F_{\max}+F_{\min}}{2}\right||\leq \frac{||F_{\max}-F_{\min}||}{2}.
$$
上两式分别展示了单边和双边约束的处理方法，其中双边约束相对难理解，其实可以写作第三式的格式，即距离平均值不能超过域宽的一半，保证在平均值附近。

而优化问题由于用s参数化，因此又改写为：
$$
\int_0^T1dt=\int_{s_f(0)}^{s_f(T)}\bar{t}_f^{^{\prime}}(s)ds=\int_{s_f(0)}^{s_f(T)}\frac{1}{\dot s^t_f}ds=\int_0^L\frac{1-\bar{k}(s)\bar{w}_1(s)}{\bar{t}(s)^T\bar{\mathbf{v}}(s)}ds.
$$
在工程上，这个优化问题仍然需要放松，因此考虑将一部分限制约束作为代价，减少约束数量。由于约束都表现为负值需要大于0，因此考虑使用二次惩罚；但另一方面大多数约束都需要大于零，对应代价不趋于无穷，因此在另一侧使用更平滑温和的对数函数，表现如下：
$$
\beta_\ell(x):=\begin{cases}-\log(x),&x>\ell\\-\log(\ell)+\frac{1}{2}\left[\left(\frac{x-2\ell}{\ell}\right)^2-1\right],&x\leq\ell.&\end{cases}\\
(注意1，随着迭代进行，l逐渐减小，对应约束逐渐放宽，边界外惩罚更大，将解逐渐推向边界)\\
\min_{\bar{\boldsymbol{x}}_{w}(\cdot),\bar{\boldsymbol{u}}(\cdot)}\int_{0}^{L}\left(\frac{1-\bar{k}(s)\bar{w}_{1}(s)}{\bar{\boldsymbol{t}}(s)^{T}\bar{\mathbf{v}}(s)}+\epsilon\sum_{j}\beta_{\nu}(-c_{j}(\bar{\boldsymbol{x}}_{w}(s),\bar{\boldsymbol{u}}(s)))\right)ds+\epsilon_{f}\sum_{i}\beta_{\nu_{f}}(-c_{f,i}(\bar{\boldsymbol{x}}_{w}(L)))\\
\mathrm{s.t.}\quad\bar{\boldsymbol{x}}_{w}^{\prime}(s)=\bar{f}(\bar{\boldsymbol{x}}_{w}(s),\bar{\boldsymbol{u}}(s)),\quad\forall s\in[0,L],\quad\bar{\boldsymbol{x}}_{w}(0)=\boldsymbol{x}_{w0}\\
(注意2，c约束函数，即上文提到的约束数学表示)\\
(注意3，正系数\epsilon也随迭代减小，对应约束越来越宽)\\
$$
