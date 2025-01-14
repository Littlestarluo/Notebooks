# CPC

互补进度控制，核心在于1.离散轨迹；2.时间优化问题的路点约束处理，后者是因为前者而产生的问题，也称为算法的标志性贡献。

多项式轨迹的平滑保证是时间最优的巨大限制，因为高机动往往需要极强的转变，使用离散轨迹能够更加高效的处理，也才能解放因为平滑导致的限制。

离散轨迹可以视作一系列x[k]表示，这对时间优化问题的路点约束带来巨大的困难：路点对应的p[k]只能确定位置而没法确定时间，因为时间本身是优化量，使得难以确定路点的离散序列，即时间分配。这种限制决定了离散轨迹几乎不可用，但可以将路点约束转化为不依赖时间分配的方式。

如果优化能够做到实际经过路点的节点不定，就可以解决路点约束问题，因此可以将每一个节点x[k]增设一个布尔量进度标识$\mu_k$，当其进入路点领域后置为1，反之为0，且单增；再为整条轨迹设计初始值为路点数的进度变量$\lambda_k=\lambda_0-\sum^k\mu_i$指示目前还剩多少路点约束还未满足。因此，只用将$\lambda,\mu$加入变量参与优化就可以完成路点约束，在优化过程中自动分配抵达路点的节点。

其中，由于进度标识$\mu_k$单增，即初始化为0，一旦置1后不变，因此只需要判断是否跨越邻域边界即可，即判断$||p_k-p_{wj}||==0$。如此体现为$\mu_k$和$||p_k-p_{wj}||$始终互异，乘积为0，因此将此方法成为互补进度控制。

## 实现

CPC的开源代码基于Python，除了`__init__.py`外由10个python文件组成，以planner.py和trajectory.py为核心，其余：

- quad.py提供动力学模型类，通过progress.py更新状态方程，由integrator.py提供的数值方法实现；
- trajectory_conversion.py将动捕存于csv内的数据读入，初步处理（单位转换，四元数转换，筛选，轨迹初始化）提取为轨迹类；
- track.py则是将路点约束（门）从yaml内读入，提取为Track类，作为路点（门）约束的来源，其定义的Track类还可以认为添加新路点（门）；
  - 其他辅助部分：plot.py绘图，quaternion.py提供四元数运算。注意plot.py作为可视化计算过程的范例需要单独研究。

### 规划器（planner）

与C++的执行类结构一样，planner也由初始化方法、辅助方法和执行方法三部分，由于不是容器类，其辅助方法除了一个线性插值函数interpolate()其余都是set函数，用于传参；而执行实际上就是优化问题的设立`setup()`和求解`solve()`。

两个函数实现了典型的优化器结构，在此会详细地解释优化问题是如何用代码实现的。优化问题数学表述如下：
$$
\min_x J=t\\
约束：u\in[u_{min},u_{max}],x_0=x_{init}^①\\
进度约束：\lambda_{k+1}=\lambda_{k}-\mu_k,\lambda_0=N,\lambda_N=1,\lambda_k^j<\lambda_k^{j+1③}\\
容差的互补约束：\mu_k^j\cdot(||p_k-p_{\omega j}||^2_2-v_k^j)=0,v_k^j\in[0,d_{dot}^2]^④\\
$$
其中：
$$
①x=[t_N,x_0,...,x_N],x_k=[x_{dyn,k},u_k,\lambda_k,\mu_k,v_k],x_N=[x_{dyn,k},\lambda_N])^②\\
②k指总长N的离散时刻，x_{dyn,k}、u_k指k时状态、控制量，v_k指此处容差松弛，t_N=t/N)\\
（因此自变量就是轨迹所有离散点的状态、控制量，加上时间段、进度控制和容差控制）\\
③上标j是迭代次数，从0开始，有上界m\\
④d_{dot}指路点阈值，即是否满足接近路点的判定距
$$

在程序中，只能用预设的功能实现。在casadi库中预设了问题的定义和求解方式：

```python
self.nlp = {'f': self.J, 'x': self.x, 'g': self.g} # 生成问题

self.solver = nlpsol('solver', self.solver_type, self.nlp, self.solver_options) # 生成求解器，self.solver_type一般是‘ipop’，solver_options是字典
self.solution = self.solver(x0=self.xg, lbg=self.lb, ubg=self.ub) # 导入初值和约束，得到答案
self.x_sol = self.solution['x'].full().flatten() # 返回值包含各项，只取优化对象x
return self.x_sol
```

这部分和ego planner在C++内使用的casadi库一样，需要先设定问题，由问题生成求解器，再传给求解器实值返回结果。值得注意的是，casadi所有参数传递都基于字典类型。

casadi的问题是字典类型，其将问题分为了四个字段：优化问题函数f，优化对象自变量x，问题约束g，问题参数p（此处未用到）。然而，如何向求解器传入“函数”成为问题，如果直接使用函数指针不确定性太多，因此casadi提供了真正意义上的数学变量ca.SX.sym或ca.MX.sym，各种数学式就通过sym变量表达。

```python
x = [] # x是一个复杂的多维向量，通过x += [y]实现拓维（即变为[x,y]）
J = 0 # 目标函数是标量总时间
t = MX.sym('t', 1), x += [t] # 增加t变量，x=[t],sym指定变量在ca中唯一名和维数
xk = MX.sym('x_init', self.NX), x += [xk] # xk指x0，x=[t,xk]
muk = MX.sym('mu_init', self.NW), x += [muk] # mu指μ
for i in range(self.N):
    uk = MX.sym('u'+str(i), self.NU), x += [uk] 
    xk = MX.sym('x'+str(i), self.NX), x += [xk]
    lam = MX.sym('lam'+str(i), self.NW), x += [lam]
    tau = MX.sym('tau'+str(i), self.NW), x += [tau] # 对应容忍v
...
self.J = t
self.x = vertcat(*x) # 变为列向量，x=[t,xk,muk,N个uk,N个xk,N个lam,N个tau,N个muk]^T
```

约束也是利用变量表达，相当于将数学约束统一转化为`lb<g<ub`的形式，因此约束就通过制定变量g以及确定上下界即可，而无需影响目标函数。注意三者维数一致，因此往往设置时三者都同一时间设置。

```python
g=[t,xk[...],muk,N个uk,N个[xk-xn],N个lam,N个tau，N个[lam[j] * (dot(diff, diff)-tau[j])],N个[mul-lam-muk],N个xk[10:13],N个xk[2],N个[muk[j+1]-muk[j]],muk,xk[...]]
self.g = vertcat(*g)
```

当约束为确定等式时，lb和ub相等，例如初末状态、末尾进度等；运动学约束被表述为xk必须等于xn，xn调用`self.fdyn = Integrator(dynamics)`即利用龙格库塔方法计算运动约束下的下一个状态；

最后一个问题就是初始状态的估计，在程序中，初值的实值是xg：

```python
x0 = [self.p_init[0], self.p_init[1], self.p_init[2], vel_guess[0], vel_guess[1], vel_guess[2], self.q_init[0], self.q_init[1], self.q_init[2], self.q_init[3], 0, 0, 0]
pos_guess = self.p_init
if self.track.init_vel is not None:
	x0[3:6] = self.track.init_vel
xg += x0 # x0
xg += [1]*(self.NW) # muk_init
for i in range(self.N):
	xg += [T_max]*self.NU # uk
    xg += [pos_guess, vel_guess, self.q_init, [0]*3] # 此处的pos_guess和上面的不一样
    if ((i_wp == 0) and (i + 1 >= self.i_switch[0])) or i + 1 - self.i_switch[i_wp-1] >= self.i_switch[i_wp]:
        lamg = [0] * self.NW
        lamg[i_wp] = 1.0
        xg += lamg
    else:
        xg += [0] * self.NW # lambda
	xg += [0] * self.NW # tau
    for j in range(self.NW):
        if i+1 >= self.i_switch[j]:
          xg += [0]
        else:
          xg += [1] # muk
if not self.xg:
	self.xg = xg
self.xg = veccat(*self.xg)
```

xg作为初始值与x严格保持一一对应，其中的一些计算步骤都是实质上的初始化（例如mu和lambda），许多量还需要作估计，最大推力控制量、估计速度和位置算法略过。

得到以上各式，就可以完成整个求解过程。但是CPC算法在求解器部分增加了回调函数，由于并非ROS系统，因此回调的写法相对扭曲：

```python
if hasattr(self, 'iteration_callback') and self.iteration_callback is not None: # 类内是否有此字段，是否非空
	if inspect.isclass(self.iteration_callback): # 是否为类
		callback = self.iteration_callback("IterationCallback")
	elif self.iteration_callback: # 若为实例
		callback = self.iteration_callback
callback.set_size(self.x.shape[0], self.g.shape[0], self.NPW) # 大小设置
callback.set_wp(self.wp) # 路点设置
self.solver_options['iteration_callback'] = callback # 给求解器增设回调
```

回调函数中有三个检查环节可以作为典型例子；回调的增设通过将回调函数类写入求解器设置字典的对应字段实现。至此，问题设置-求解器-求解三个步骤全部完成。

### 轨迹类（trajectory）

CPC使用的轨迹类相对特殊，其除了init和很多用于绘图（可视化）的函数外，特别设置了分析`parse()`和逆分析`unparse()`用于将轨迹和一位数组`self.x`间互相转化，从`unparse()`可以明显看到x的结构：

```python
def unparse(self):
    n_slice = self.NX + self.NU + 3 * self.NW
    n_start = 1 + self.NX + self.NW
    self.x = np.zeros(n_start + self.N * n_slice)
    idx = np.array([0, *list(range(n_start+self.NU-1, len(self.x)-1, n_slice))]) # 实现不用循环完成for
    self.x[0] = self.t_total
    self.x[1+idx] = self.p[0,:]
    self.x[2+idx] = self.p[1,:]
    self.x[3+idx] = self.p[2,:]
    ...
    self.x[6+idx] = self.v[2,:]
    ...
    self.x[10+idx] = self.q[3,:]
    ...
    self.x[13+idx] = self.w[2,:]
    ...
    self.x[n_start+3::n_slice] = self.u[3,:]
```

`range(n_start+self.NU-1, len(self.x)-1, n_slice)` 则生成了一系列索引，从 `n_start+self.NU-1` 开始，每隔 `n_slice` 个元素取一次，直到接近 `self.x` 的末尾。这部分索引指向的是每个时间点最后一个控制输入变量之后的位置，即下一个时间点开始的地方。

而解包的过程中，除了上述逆过程，也添加的不少信息：

```python
self.t_x = ca.DM(np.linspace(0, self.t_total, self.N+1, True)) # 均匀时间
self.t_u = self.t_x[0:self.N] # 前N项时间（除最后一项）
dt = self.t_total / self.N

self.a_lin = np.zeros((3, self.N+1))
self.a_rot = np.zeros((3, self.N+1)) # 以下零向量初始化省略
self.a_lin[:,0:-1] = np.diff(self.v) / dt
self.a_rot[:,0:-1] = np.diff(self.w) / dt

for i in range(self.NW):
    self.mu[i,:] = self.x[1+self.NX+i::n_slice]
    self.nu[i,:] = self.x[n_start+self.NU+self.NX+i::n_slice]
    self.tau[i,:] = self.x[n_start+self.NU+self.NX+self.NW+i::n_slice]

for i in range(self.N):
	self.thrust[:,i] = np.array(
		rotate_quat(ca.DM(self.q[:,i]), ca.vertcat(0, 0, ca.cumsum(self.u[:,i]))))[:,0]
	self.dir[:,i] = self.thrust[:,i] / np.linalg.norm(self.thrust[:,i])
```

加速度、推力等信息都是通过x的量间接算出的，都先初始化为零向量，然后再实例化。

类内的绘图主要通过plot.py内的回调函数调用的。

### 绘图（plot）

plot.py内分为回调函数和主函数两部分，二者都是确定各项参数，再通过调用traj类内绘图函数完成绘图。参数最重要的，莫非总的图大小，子图排列方法，以及轴标题。图的形态和排布高度依赖子图数量，因此确定每个字段是否需要绘图至关重要。

回调函数中，图字段直接传入函数，储存在`self.pos`；而主函数则使用`parser = argparse.ArgumentParser()`对象，参数组，在函数内（手动）`parser.add_argument()`添加参数，储存在`args = parser.parse_args()`；二者也都通过`if XXXX.pos is not None: n_plots += 1`为图计数，根据数量分类讨论绘图形式（例如分不分两行）。最后，调用轨迹类函数绘图，提供总图长宽和子图序号：
```python
# 回调函数：
if self.pos is not None:
	self.axes[i_plot].cla() # 清理子图
	traj.plotPos(self.axes[i_plot], plot_axis=self.pos)
	i_plot += 1
# 主函数：
if args.pos is not None:
    traj.plotPos(axes[i_fig], plot_axis=args.pos, wp_style='rx')
    i_fig += 1 # 下一个图序号
```

绘图回调函数是一个casadi库内定义的函数，可以作为求解器的回调函数由`solver_options`引入求解器（具体操作在上节已给出代码）。求解器初始化时回调函数初始化，求解器迭代时，调用回调函数的`eval()`函数。因此与主函数不同，回调函数的初始化只确定轴参数、图个数、子图排布并生成一张空白画布，然后在`eval()`内调用上述的轨迹类绘图。基于绘图的基本原理，需要把上一次图清除重新绘制，由于每一个数据点都和轴绑定，因此利用`self.axes[i_plot].cla()`清除对应子图，然后重新绘图。相应的，主函数只需要绘制csv文件中确定的静态轨迹，因此只需要绘制一次

主函数利用`parser`库设定了所有的图参数，以主函数的位置图为例可以查看绘图所需要的所有信息，相比回调函数更加复杂（后者只需要得到子图数即可），因此以主函数为例：

```python
import matplotlib.pyplot as plt
import argparse
...
parser = argparse.ArgumentParser(description='Plots a given trajectory from .csv format.')
parser.add_argument('filename', metavar='file', type=str, help='filename of the trajectory csv file')
  parser.add_argument('-p', '--position', dest='pos', type=str, help='plot position over specified axes')
... 
args = parser.parse_args()
# 其他各图，parser对象建立，增设文件名、各图所需字符串参数，存到args内

traj = Trajectory(args.filename) # 生成轨迹

if all(arg==None for arg in [
    args.pos, args.vel, args.omega, args.ori, args.thrust, args.prog]):
    args.subp = [2, 3]
    args.pos = 'xy'
    ... # 其他各图，给每个图赋值轴名称
    
n_plots = 0
if args.pos is not None: n_plots += 1
... # 其他各图，计子图数

use_subplot = False
if args.subp is not None:
	subr = args.subp[0]
	subc = args.subp[1]
	use_subplot = True
	if subr * subc < n_plots:
		Warning('Subplot too small')
		subc = math.ceil(n_plots/subr)
if use_subplot:
	fig = plt.figure(0)
axes = []
for i in range(n_plots):
if use_subplot:
	axes += [plt.subplot(subr, subc, i+1)]
else:
	axes += [plt.figure(i)] # 确定图排布
    
i_fig = 0
if args.pos is not None:
	traj.plotPos(axes[i_fig], plot_axis=args.pos, wp_style='rx')
	i_fig += 1
... # 余图同理，在确定排布的图内依次绘制子图

if args.output is not None:
	plt.savefig(fig, args.output)
else:
	plt.show()
```

与回调函数不同，上述主函数中图的信息承载于args内，且判别图排布的方式不同：主函数利用`args.sub`直接判别是否用子图，以及子图的排布；而回调函数则通过子图数来选择排布方式。

```python
# 回调函数内
if self.n_plots<=3: 
      self.plot_r = self.n_plots
      self.plot_c = 1
    else:
      self.plot_r = 2
      self.plot_c = int((self.n_plots+1)/2)
```

而图的建立方式比较类似，都是先建立图对象`plt.figure()`再依次将子图添加在轴数组中（这也是为什么可以通过axes清理子图，axes内装的就是子图对象）：

```python
# 回调函数内：
self.axes = []
if self.n_plots>1:
	for i in range(self.n_plots):
		self.axes.append(plt.subplot(self.plot_r, self.plot_c, 1+i))
else:
	self.axes.append(self.fig.gca())
# 主函数内：
axes = []
for i in range(n_plots):
	if use_subplot:
		axes += [plt.subplot(subr, subc, i+1)]
	else:
		axes += [plt.figure(i)]
    
```

至此CPC代码主要内容分析完毕。

