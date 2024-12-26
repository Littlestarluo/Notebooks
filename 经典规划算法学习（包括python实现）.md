# 经典规划算法学习（包括python实现）
└── Search-based Planning
    ├── Breadth-First Searching (BFS)√
    ├── Depth-First Searching (DFS)√
    ├── Best-First Searching√
    ├── Dijkstra's√
    ├── A\* √
    ├── Bidirectional A\* √
    ├── Anytime Repairing A\* √
    ├── Learning Real-time A\* (LRTA\*)√
    ├── Real-time Adaptive A\* (RTAA\*)√
    ├── Lifelong Planning A\* (LPA\*)√
    ├── Dynamic A\* (D\*)
    ├── D\* Lite
    └── Anytime D\*
└── Sampling-based Planning
    ├── RRT
    ├── RRT-Connect
    ├── Extended-RRT
    ├── Dynamic-RRT
    ├── RRT*
    ├── Informed RRT*
    ├── RRT* Smart
    ├── Anytime RRT*
    ├── Closed-Loop RRT*
    ├── Spline-RRT*
    ├── Fast Marching Trees (FMT*)
    └── Batch Informed Trees (BIT*)
    

## 搜索算法
### 搜索策略

搜索算法通过遍历邻接点寻找目标而得到路径，当选择邻接点的策略不同、遍历后的处理不同，得到的不同的算法。
选择邻接点的策略，也可以理解为优先性评估，决定了当前点的邻接点以怎样的顺序进入搜索队列，这几乎决定了搜索的倾向性。我们可以用这种方式来描述算法，写出通式为：

```python
while self.OPEN:
	_, s = heapq.heappop(self.OPEN) # 提取最优先无索引待搜索点，待搜索点集为OPEN
	self.CLOSED.append(s) # 将该点s标记为已搜索点，即插入已搜索点集CLOSE
    if s == self.s_goal: # 终止条件为找到目标点
    	break
    for s_n in self.get_neighbor(s): # 遍历s的邻接点，get_neibhbor基于存储的图
    	new_cost = self.g[s] + self.cost(s, s_n) # 计算路径代价，未计算的设为无限，可以达到只要更新的路劲更小的计算，即既不会漏掉未计算的点，也能当同一个点有代价更小路劲时选择更小路劲（更新父节点）
        if s_n not in self.g:
            self.g[s_n] = math.inf
        if new_cost < self.g[s_n]:
            self.g[s_n] = new_cost
            self.PARENT[s_n] = s # 加入
            prior = prior_function(s_n) # 优先评估算法
            heapq.heappush(self.OPEN, (prior, s_n)) # 优先队列，prior最小的优先pop
    
return self.extract_path(self.PARENT), self.CLOSED # parent集实现了路劲树，即可得到路劲
```
优先评估算法不同即搜索策略不同。对于深度优先搜索，每个新点的邻接点优先级都要最先，这样每一次循环总是向树深处，到底后再递归地返回上一层，表示为：
```python
prior = self.OPEN[0][0]-1 if len(self.OPEN)>0 else 0 # DFS
```
广度优先搜索则要保证每个新点的邻接点都在最后，这样每次需要把一整层的点遍历后再到下一层，即
```python
prior = self.OPEN[-1][0]+1 if len(self.OPEN)>0 else 0 # BFS
```
深度优先搜索（DFS）和广度优先搜索（BFS）是最为基础的搜索算法，几乎可以定义了搜索本身，却不能保证路径的最优性，因此有Dijkstra算法，使得搜索向路劲最优前进，即：
```python
prior = new_cost #Dijkstra
```
然而，上述三种算法都基于遍历，复杂度与图的变平方成正比，因此可以引入启发式搜索，即提供与目标点有关的信息引导搜索。最基础的启发式算法就是最优有限算法BF，引入了启发式因子h：
```python
prior = h # BF
h = heuristic(s_n)
def heuristic(s)
	if heuristic_type == "manhattan":
		return abs(goal[0] - s[0]) + abs(goal[1] - s[1]) # 当前点s到目标点的曼哈顿距离
	else:
		return math.hypot(goal[0] - s[0], goal[1] - s[1]) # 欧氏距离
```

同样的，BF不能保证路劲最优，因此自然可以将代价因子和启发式因子结合，这就是A*算法：
```python
prior = f # = g + h, Astar
g = new_cost # = self_g[s_n]
h = heuristic(s_n)
```

### 搜索后处理

A\*算法成为了高效有力的算法，平衡了搜索的效率和路径的质量，成为搜索算法的基石，因此之后对搜索算法的改进基本都是基于A\*算法上的。

1. bi directional Astar【双向Astar】双向搜索，停止条件为OPEN集合中有共同点； 

2. repeat Astar【重复Astar】重复搜索，提取最优路径；

3. AR Astar【任时修复Astar】重复搜索，但是通过启发式系数e降低启发因子大小，直到达到目标性能；

4. LRT Astar【实时学习Astar】重复搜索，但每次只搜索最多N个点，每次搜索后更新所有已搜索点的h值；然而h值更新的是值是代价和启发式因子之和的最小值：

```python
def iteration(self, CLOSED):
	h_value = {}
	for s in CLOSED:
		h_value[s] = float("inf")  # 已搜索点h设为无穷，未搜索点设为静态的h值（欧氏距离）
	while True:
		h_value_rec = copy.deepcopy(h_value)
		for s in CLOSED: # 计算每个已搜索点为起点时，邻接点的最小h值，并作为自己的h值
			h_list = []
			for s_n in self.get_neighbor(s):
				if s_n not in CLOSED:
					h_list.append(self.cost(s, s_n) + self.h_table[s_n])
				else:
					h_list.append(self.cost(s, s_n) + h_value[s_n])
			h_value[s] = min(h_list) # 只有邻接点h变小后才可能改变
		if h_value == h_value_rec:  # 停止条件是h值表收敛
			return h_value
```
这种策略使得：遍历已搜索点的所有邻接点时，如果邻接点都是已搜索的，h将会一直是无穷（else情况）；**当且仅当**搜索到未搜索点时（if情况），h值才有小于无穷的值更新。

这一逻辑具有深刻的意义，我们未搜索点记为s_open，将cost(s,s_n)视为递推公式，可以发现所有非无穷h值点s的h值**总是**等效为s到某个未搜索点的路径代价加上该搜索点的静态h值，动态h值此时可以看做s到搜索边的一个点再到目标点的折线距离，即：

```python
h_value[s]=cost(s,s_n)+h_value[s_n]
h_value[s]=cost(s,s_n)+cost(s_n,s_n_n)+h_value[s_n_n]
h_value[s]=cost(s,s_n)+cost(s_n,s_n_n)+...+heuristic(s_open)
h_value[s]=cost(s,s_open)+heuristic(s_open)
```

因此，迭代本质上优化的是```s_open```的选取和到```s_open```的最优路径。由于三角不等式，最终的结果应该是：```s_open```和到```s_open```的路径都位于s到目标点的直线上，即得到的结果是```h_value[s]=heuristic(s)```，然而关键在于：已搜索的域内路径可以实现避障，因此每个点的启发式的值不再是直接与目标点的距离，而是接近真正的到目标点的路径距离。

因此，每一次迭代后，下一个循环的Astar搜索过程都会使用经过优化的启发式值，直观上避免了撞墙导致的冗余搜索。

5. RTA Astar【实时适应Astar】重复搜索，但每次只搜索最多N个点，每次搜索后更新所有已搜索点的h值；然而h值更新时使用虚拟启发式因子v值计算当前最优点为起始点，重新计算所有已搜索点的f值作为h值：

```python
for (_, x) in OPEN.enumerate():
	v_open[x] = g_table[PARENT[x]] + 1 + self.h_table[x] # v值其实是f值，注意由于OPEN内的点还没计算g_tabel，因此通过父节点推算
s_open = min(v_open, key=v_open.get) 
f_min = v_open[s_open] # v值实现为fmin，对应最优未搜索点
for x in CLOSED:
	h_value[x] = f_min - g_table[x] # 已搜索点的h值实际是以fmin对应点为起点重算的f值
```

和LRT Astar算法一样，我们可以写出实际各个已搜索点的启发值的总表达式：
```python
h_value[x] = g_table[PARENT[s_open]] + 1 + self.h_table[s_open] - g_table[x]
h_value[x] = cost(s_0,s_open) - cost(s_0,x) + heuristic(s_open)
h_value[x] ≈ cost(s_open,x) + heuristic(s_open)
```
可以看到，与LRT Astar的目的一致，但是思路不同，最大的区别在于启发值中对当前点到```s_open```的代价只是一个估计值，而非对应实际路径得到的代价，更无法保证最优；但是，RTA Astar只需要计算所有未搜索点内最优点，就可以直接得到每个已搜索点的优化启发值，而不需要对所有未搜索点多次迭代优化，在效果相似的同时，实时性大大加强。

6. LP Astar【长效规划Astar】引入了动态障碍物监测机制，通过两个代价列表来实现判断：

```python
def ComputeShortestPath(self):
	while True:
		s, v = self.TopKey()
		if v >= self.CalculateKey(self.s_goal) and self.rhs[self.s_goal] == self.g[self.s_goal]: # 终止条件，当前最优点比目标点更优先，且目标点在当前路径和最优路径下的代价相等，即搜索到终点且到终点的路径相同
			break 
		self.U.pop(s)
		self.visited.add(s)
		if self.g[s] > self.rhs[s]: # 若搜索到更优路径，更新g值
			self.g[s] = self.rhs[s]
		else: # 若最优路径不再最优，说明原路径有障碍物，将g值重置为无穷，放弃原路径（即当前点s）重新搜索s
			self.g[s] = float("inf")
			self.UpdateVertex(s)
		for s_n in self.get_neighbor(s): # s确定后正式搜索
			self.UpdateVertex(s_n)
```
判断障碍物来源于预设：同一个点，当最优路径从增大，则说明出现障碍物影响。因为g总是由rhs赋值而来，而rhs总是由搜索计算而来，一旦g路径代价小于当前的rhs，就说明搜索计算出的rhs大于搜索计算前的rhs，说明路径发生了变化；由于路径计算方法与Astar一致，并未改变，因此可以认定为出现障碍，于是放弃当前搜索点重新搜索。

重规划时，要实现避免重复计算和优先处理受影响节点，障碍物变化时需要外部算法（例如建图算法）去提供障碍物的位置，让LPAstar去更新障碍物邻接点并加入队列U且优先弹出处理，当g和rhs重新一致后就不用继续计算了。

Dstar

Lite D

A Dstar
