# SUPER: Safety-assured high-speed navigation for MAVs(MARS, 2025)

关键：①集感知-规划-控制一体的系统，可避障的同时实现高机动规划，②具有安全保障算法。

- ①整个模型结构并不少见，即利用激光雷达（50HZ）建立局部栅格地图和IMU定位（200Hz），根据地图进行规划（10Hz），规划的结果交由MPC进行跟踪（100Hz）。感知部分直接使用Fast-lio2。

- ②特别的是安全保证算法，即如果目标点在未知区域时同时规划两个轨迹，分别保证最优和最安全。这依赖于两个重要技术难点：a.局部地图的生成和b.规划方法。
  - 对a而言，可以与ego planner V2使用的环形缓冲区RingBuffer算法实现的局部地图相对照，显然此种方法还需要区分已知区域和未知区域，需要利用类似Raycast方法来判断非点云栅格是否已知；
  - 规划方法就在于能否实现在线重规划，以及规划如何尽可能保证时间最优。由于遇到未知区域时需要规划两条路线，即使重合部分不做考虑，要尽可能在保证最优的条件下压缩时间是非常困难的。

优先关注软件方面：

## 建图

滑动窗口策略+清除机制+懒惰更新策略。

- 滑动窗口策略：？

- 此处的清除机制和ego2的衰减机制同构，即每个栅格最后一次刷新的时间超过阈值后进行清除，但实现的方法大不相同：ego2基于概率图的置信度定时递减，而super则直接计算每个栅格的时间戳，因此ego2的方法要更加稳健。
- 懒惰更新策略：？

## 规划

状态机（fsm）的重规划频率为10Hz，且同时完成两条轨迹的重规划。重规划的判断为更新目标点，若目标点不在视距H内则用投影到H球面作为目标点，两条轨迹若跟新失败，则采用前一帧轨迹。

轨迹的规划采用飞行走廊法（corridor）：

- 通过在A*无碰撞轨迹上利用视域划分出点截成各个走廊的种子；
  - 考虑到无人机的几何形态，地图需要增设膨胀，此处提出了配置空间迭代区域膨胀（CIRI）代替传统的快速迭代区域膨胀（FIRI），可以直接由点云获取走廊；
  - 如果无人机视域（FOV）有限，则将走廊和FOV取交作为新走廊；
  - 两走廊重叠处，希望轨迹衔接时尽可能远离走廊边缘，因此把控制点Q到重叠部分中心的距离$v=\sum^M_{i=1}L_\mu(||q_i-a_i||_2^2)$作为代价引入代价函数，i指走廊编号，$a_i$指走廊$p_i\cap p_{i+1}$的中心点，$L_\mu$是一个$c^2$障碍函数。
- 对每个走廊，生成末态0速度的备份轨迹，使其始末点都在走廊内；
  - 为了速度，备份轨迹起点应该尽量晚，但起点时间$t_s$又必须大于当前时间$t_c$且小于预期轨迹出走廊时间$t_o$，因此设$t_s=(t_o-t_c)\frac{1}{1+e^{-\eta}}+t_c$，将$v=-t_s$加入代价函数中；

轨迹参数化方法采用MINCO轨迹，每一个飞行走廊对应MINCO的一段piece，优化问题也采用GCOPTER一脉的无约束优化问题以及与无约束适配的L-BFGS数值计算法。优化问题的形态如下：
$$
\min_{Q,T}\int^{t_M}_0||p^{(s)}(t)||^2_2dt+\rho_Tt_M+\rho_uv\\
其中：
p(t)=C(Q,T,s_0,s_f)\beta(t),\quad
t_M=\sum^M_{i=1}T_i,\quad
||\dot p||^2_2\leq v^2_{max},||\ddot p||^2_2\leq a^2_{max},\quad
p_i\in P_i\\
v=\begin{cases}\sum^M_{i=1}L_\mu(||q_i-a_i||_2^2),\quad正常轨迹\\-[(t_o-t_c)\frac{1}{1+e^{-\eta}}+t_c],\quad备份轨迹\end{cases}
$$

## 控制

控制策略是OMMPC，该方法通过将其状态从状态歧管映射到沿轨迹$s(t)$的每个点的局部坐标来线性化，从而使该状态最小化，从而导致一个线性化的系统.

## 规划实现

论文中对于许多细节没有作进一步的解释，例如

- 状态机如何实现非rest-to-rest的预期轨迹规划和预期轨迹与备份轨迹的切换的；
- 局部地图如何建立？栅格清除具体机制是什么？
- CIRI的实现方法是什么？
- 飞行走廊如何设定？如何在A*基础上获取最远视点？
- 有约束的优化问题如何转换为无约束？

### 总体结构与状态机

> 由于在阅读ego已经有了大工程阅读基础，因此此处不作完全的细节展开

SUPER是基于CPP的，其启动文件以Python和Launch混用，此外结构和ego有诸多相似之处，反映了一个完整系统的普适架构。特别注意的是，SUPER将mission_planner和super_planner二分，后者本身更接近ego。

SUPER的主体就是super_core、traj_opt、utils三部分：

- core集成了建图、状态机和规划管理器；
- opt则清晰地分为备用、预期轨迹优化以及一个yaw角优化器；
- utils则塞入各种容器、数学方法的定义，例如轨迹的容器，minco优化器，lbfgs求解器等等，与ego无异。

SUPER的状态机结构更加优秀，其将重规划和状态机分离，状态机执行函数内状态单向进行，而反向切换都由外部执行函数切换，而切换函数同时表明了切换状态的主体，结构清晰。

1. 状态机状态有：[init wait_goal ~~yawing~~ generate_traj follow_traj emer_stop]，在`void Fsm::callMainFsmOnce()`内执行：

- fsm需要启动时间，当前时间，当前状态，里程和目标点；时间信息用于撰写**日志（log）和计时**。

  - 里程通过`planner_ptr_->getRobotState(robot_state_);`获取，若空或滞后0.1s则视作无里程；
  - 目标点通过`void Fsm::setGoalPosiAndYaw(const Vec3f &p, const Quatf &q)` 设置，改变标识`started_`和`gi.new_goal`；
  - 在进入状态前就先检查一遍，并输出是否缺少odom和goal。

- init检查里程和目标点，若兼备进入wait_goal；

- wait_goal检查目标点，若有则进入generate_traj并重置可视化路径；至此和ego一致；

- generate_traj检查目标点未抵达，则进行轨迹生成；若目标点无效，则跳过；若生成成功则发布轨迹并进入follow_traj；

  - 轨迹生成调用的是`planner_ptr_->PlanFromRest`，注意其在检测目标是否有效之前（因为有效与否是`PlanFromRest`来设置的）；
  - 目标是否抵达用`bool Fsm::closeToGoal(const double &thresh_dis)`判断与目标点距离；
  - 生成成功使用`planner_ptr_->PlanFromRest`返回的标识`ret_code`判断；
  - 生成成功后改变`gi.new_goal`，等待下一个新点。

- follow_traj调用`publishCurPoseToPath();`发布控制指令；

  > 另外emer_stop实现比较简单粗暴，即直接进入wait_goal状态。

2. 状态机内的重规划设置了四重标识条件[!stop follow_traj !finish_plan !plan_from_rest_]，同时满足后调用`planner_ptr_->ReplanOnce`利.用返回的标识`ret_code`切换状态：

- FAILED无操作，等待下一次重规划；
- EMER成功但急停，进入emer_stop；
- NEW_TRAJ需要新轨迹，进入generate_traj；
- SUCCESS或FINISH是正常重规划，与generate_traj相同改变`gi.new_goal`并发布多项式轨迹，由于重规划一直处于follow_traj内，因此无需切换状态。



### 规划器

优秀的状态机写法令人赏心悦目，接下来就是状态机中调用到的很不赏心悦目的规划函数，分别是轨迹生成函数`PlanFromRest()`和`ReplanOnce()`。在管理器的初始化中并没有使用订阅和发布器，只是初始化了各项容器变量。

无论任何算法，产生一条轨迹都需要起点约束和终点约束，分别对应无人机当前状态和状态机下达的目标指令。函数在状态机中调用，传入目标点约束，再在规划器内部获取无人机状态，然后就可以调用生成函数：

```python
ExpTraj exp_traj_info;
last_exp_traj_info_.setEmpty();
RET_CODE exp_ret_code = generateExpTraj(last_exp_traj_info_, exp_traj_info);
...# 根据返回值执行if判断，整个super高度依赖各个执行函数的返回值，这相比单纯的判断非空更加灵活可塑。
back_traj_info.setEmpty();
RET_CODE back_ret_code = generateBackupTrajectory(exp_traj_info, back_traj_info);
...# 根据返回值执行if判断
```

在重规划中亦是如此：

```python
ExpTraj exp_traj_info;
TimeConsuming t_exp("t_exp", false);
RET_CODE exp_ret_code = generateExpTraj(last_exp_traj_info_, exp_traj_info);
time_consuming_[GENERATE_EXP_TRAJ] = t_exp.stop();
...
BackupTraj back_traj_info;
TimeConsuming t_back("t_back", false);
RET_CODE back_ret_code = generateBackupTrajectory(exp_traj_info, back_traj_info);
time_consuming_[GENERATE_BACK_TRAJ] = t_back.stop();
...
```

轨迹生成函数的引用传参只传了前一条轨迹和本条轨迹地址，而其他信息（始终点信息）都是通过调用规划器类成员变量（全局）直接获得的（虽然前一条轨迹其实也可以写成全局变量，这样只需要传本轨迹引用）。

生成新轨迹和重规划的主要区别有二：状态机切换不同，是否需要前一条轨迹。后者是通过预期轨迹生成中`last_exp_traj_info_`是否非空判断的，而备份轨迹都由预期轨迹生成，因此无差别；具体来说，前一条轨迹的作用是确定起点状态。这样就相当于所有的轨迹生成都可以看做重规划。

> 回忆ego的连续规划方式，采用全局规划和局部规划两种，先生成一个到目标点的全局规划路径，再沿着路径进行局部规划，重规划针对的是全局路径，而真正发出控制指令的是局部鬼狐；当距离目标点一定距离内，规划到下一个目标点保证连续性。
>
> 这个算法的一大缺点是：全局轨迹始终是rest2rest的，甚至局部规划也将所沿轨迹的最终目标点视作0速度；即使在抵达目标之前提前进行重规划（这部分代码还并非ego原有的），轨迹之间的衔接性也很不好。甚至由于前一条轨迹的时间、全局时间、下一条轨迹时间出现错乱导致规划出现回头路。

为了解决形如ego的问题，在生成轨迹时除了必要的图信息（SFC，安全走廊）、始终点约束、全局时间和轨迹时间（WT，TT）外，生成函数还设立了引导路径和其对应的轨迹时间（guide path，guida t），当前一条轨迹非空，前一条轨迹就作为引导路径。

同时，对重规划需要对各种状态作一遍早退检查：规划的目的是对同一个目标点，从旧路径中产生新的路径，但如果已经很接近最终目标点就不用生成了，可以作为早退机制（`ret_code = NO_NEED`）。

- 时间：时间判断是否抵达终点，调用`replan_state_TT`和前路径总时长比较；但所谓“前一条路径”其实有两个：`cmd_traj_info_`和`last_exp_traj`，分别做判断。
- 距离：同ego的touch_goal，即当前目标是最终点`!gi_.new_goal`且已抵达，再判断一遍到目标点距离；
- 走廊：或调用前一条轨迹的两个类内方法：`last_exp_traj_info.getSFCSize() == 1 && last_exp_traj_info.connectedToGoal()`，指前一条轨迹末端抵达终点且只剩下一个走廊，即终点在当前走廊内，无需规划。

通过早退检查意味着确定开始规划，就需要满足要求：引导轨迹无碰撞，引导轨迹作为一部分。

- 无碰撞采用时间遍历，利用`eval_t`搜索判断，同时步长小于0.8分辨率时跳过提高效率；
- 引导轨迹的关键点在于：新的轨迹从旧轨迹的何处开始？`replan_state_TT=replan_process_start_TT + cfg_.replan_forward_dt`表明默认在重规划开始后一小段时间开始规划，即`cfg_.replan_forward_dt`作为默认的引导轨迹段；
- 在确定了默认引导段后，根据碰撞检测是否后退。

引导路径至此设置完成，接下来就是在引导轨迹基础上规划新的轨迹：

- 新的轨迹的起始点：刚刚确定的引导轨迹的终点；
- 新轨迹的终点：利用预设的临时轨迹最大范围，减去引导轨迹长度即可；
  - 如果发现新轨迹范围很小，不用生成轨迹，反之新轨迹视作引导路径的拓展；
  - 但如果终点距离引导轨迹很近，那么直接在路径中加入目标点即完成拓展，而不需要几何轨迹；
  - 几何轨迹就需要真正的获取A*轨迹，再
  - 


