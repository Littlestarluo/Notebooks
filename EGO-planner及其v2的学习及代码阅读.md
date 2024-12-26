# EGO-planner及v2的学习和代码阅读

EGO-planner（基于梯度的无欧式距离场局部四旋翼无人机规划器，以下简称ego）是为了解决计算距离场耗时巨大的问题而诞生的规划器，需要输入目标点坐标（路点，waypoint）和环境信息（图），生成一条无碰撞的B样条轨迹；v2则将B样条更换为MINCO（最小控制模型，下称minco）。

ego的核心思想是：

① 先利用路径模型（B样条或minco）生成抵达路点的、无视障碍的初始路径，再根据环境用Astar算法生成一条无碰撞路径；
② 判断初始路径中处于障碍物内的点，作他们沿路径的法平面与无碰撞路径相交得到参考障碍物表面点，使用优化算法将路径点从障碍物内推到安全位置；
③ 由于初始路径的点改变，其点所约束的路径也改变到安全区域，同时保持平滑，就得到了初步的轨迹，经过细化后可以作为实际规划路径输出。

借助ego的代码，不仅可以了解ego思想的实现细节，还能够更好地把握一个完整、高效的规划系统是如何构建的。他的代码分为以下几个部分：

- 建图：建立栅格地图和膨胀地图

* 搜索：基于图，用Astar生成可行路径

- 轨迹优化：生成minco轨迹，并细化之

- ego管理器：ego状态机，处理路点的接受和发布，控制规划方式和时序

- 轨迹可视化：将轨迹有关的各种信息处理后发布

在egoV2中，加入了无人机集群的处理，因此还有两个部分：

- 无人机探测：检测、定位其他无人机

- 集群桥接：和其他无人机的通信（？）

## 建图（plan_env）

grid_map.cpp完成了建图功能，程序主体就是地图初始化函数```void GridMap::initMap(ros::NodeHandle &nh)```，程序的其余部分就是initMap中各个发布器、订阅器的回调函数，最后是这些回调函数中使用的功能函数，形成非常清晰的三级结构。

ROS中，以此节点为例，一个节点的建立一般来说是由main函数cpp，调用cpp，启动程序launch这三级文件构成的（忽略头文件）：
```c++
//plan_env/grid_map.cpp内
#include "plan_env/grid_map.h"
void GridMap::initMap(ros::NodeHandle &nh)
{
	node_ = nh;
    ...
}
```
```c++
// 某个plan.cpp内
#include <ros/ros.h>
#include "plan_env/grid_map.h"///plan_env/grid_map.cpp通过这个头文件链接
GridMap::Ptr   grid_map_test_ ; 
int main(int argc, char** argv)
{
    ros::init(argc, argv, "gridmap_test_node");
    ros::NodeHandle nh_("~");
    grid_map_test_.reset(new GridMap) ;// 程序将整个节点用指针封装（如上GridMap::Ptr），因此需要更新，reset函数提供了释放空间再重新指向新空间的功能，新空间就是指new申请的类型确定的新内存空间。
    grid_map_test_->initMap(nh_) ; // 实现，nh句柄传入initMap中
    ros::spin();
    return 0;
}// https://blog.csdn.net/chunchun2021/article/details/134535140
```
```xml
<!-- run_sim.launch内 -->
<launch>
    
<node pkg="ego_planner" name="gridmap_test_node" type="gridmap_test_node" output="screen">
    <remap from="~grid_map/odom" to="/mavros/local_position/odom"/>
    ...
    <param name="grid_map/resolution"      value="0.15" /> 
    ...
</node>

<node pkg="odom_visualization" name="drone_odom_visualization" type="odom_visualization" output="screen">
    ...
</node>
    
<node name="rviz" pkg="rviz" type="rviz" args="-d $(find ego_planner)/launch/gridmap_test.rviz" required="true" /> 
<!-- 有时rviz用单独launch文件启动，实现同rvis多功能轮流运行，此时需要先启动rvis再启动以上代码所在文件 -->

</launch>
```
```bash
source devel/setup.bash
roslauch ego_planner run_sim.launch
```

可以看到，节点本身与功能是在包（packge，即ego_planner本身）内的cpp定义的，但实现需要主函数```int main()```唤起，而cpp内设定的节点参数由launch给出。其中，.cpp文件调用.h文件中加入的其他.h文件及定义的变量、函数和类，而.launch文件通过
```xml
<include file="$(find ego_planner)/launch/include/run_in_sim.xml">
        <arg name="drone_id" value="0"/>
		...
</include>
```
来从.xml文件（或.rviz或.yaml）中读取节点参数信息，通过
```xml
<include file="$(find moving_obstacles)/launch/obstacle_run.launch"/>
```
来调用其他.launch文件。因此形成了node（.cpp和.h）、main（.cpp和.h）、launch（.launch和.xml等）三层次。

### 图初始化GridMap::initMap()

GridMap的存在形式和实现方式。GridMap是一个类，所有的栅格数据、功能函数乃至节点本身都涵盖在内，而实现的时候我们只需要采用指针，就可以直接地调用内部的所有数据，而内部函数调用时也因是类内调用而方便许多。

```c++
void GridMap::initMap(ros::NodeHandle &nh)
{
	node_ = nh;//①
    
	node_.param("grid_map/pose_type", mp_.pose_type_, 1);//②
	...
	if (mp_.inf_grid_ > 4){...}//③
	...
	md_.ringbuffer_origin3i_ = Eigen::Vector3i(0, 0, 0);//④
	...
		depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "grid_map/depth", 50));
		extrinsic_sub_ = node_.subscribe<nav_msgs::Odometry>(
			"/vins_estimator/extrinsic", 10, &GridMap::extrinsicCallback, this);//⑤
	...
    map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy", 10);//⑥
    ...
}
```

如上可以视为函数的大致结构，即代码中标注的6个部分。

首先作为节点，函数最基础的要求就是完成上节所属的三级结构，即【①】能传入ROS句柄```ros::NodeHandle```，用临时变量node_接受；【②】定义节点参数```node_.param```，从而可以由launch文件传参至此。可以看到与节点有关的操作都是通过句柄```node_```实现的。

```MappingParameters mp_; MappingData md_;```就是在GridMap类中的地图数据，他们是两个结构体，一下简称mp和md。mp\_包含的是建图接受到实际数据前就需要预先知道的数据，GridMap从launch文件中接收到的参数就通过```node_.param``传入mp\_内【①】，需要做计算处理或判断的mp参数也在函数内运算【②】；而md\_则是整个运算过程中的容器，GridMap的订阅器接受到传感器数据后通过回调函数【⑤】将其塞入md\_中，md在函数中首先利用mp进行初始化设立初值【④】。

一般来说，一个订阅器对应一个话题，对应一个处理该话题数据类型的回调函数。而程序八个回调函数分别属于3个订阅器，两个同步器，3个计时器。计时器直接调用ROS系统的时间信息，而同步器通过指针传参处理两个订阅器的话题数据，表现为如下结构：
```c++
depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "grid_map/depth", 50));
...
pose_sub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_, "grid_map/pose", 25));
sync_image_pose_.reset(new message_filters::Synchronizer<SyncPolicyImagePose>(
        SyncPolicyImagePose(100), *depth_sub_, *pose_sub_));
sync_image_pose_->registerCallback(boost::bind(&GridMap::depthPoseCallback, this, _1, _2));
```

可以看到回调函数```GridMap::depthPoseCallback```要处理两个话题的信息，而订阅器只能订阅一个，因此用同步器来连接两个订阅器，然后注册回调函数。值得注意的是，这些消息过滤器都是通过过滤器的智能指针```share_ptr```实现的，由于指针可以随时释放内存、占用内存，使内存分配极为灵活，其定义类似：
```c++
//GridMap private
shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;//智能指针<指针类型>，过滤器::过滤器类型<消息类型>，传感器消息命名空间::（深度）图像
```

最后，在【⑥】处发布```grid_map/occupancy```和```grid_map/occupancy_inflate"```两个话题，即栅格地图和膨胀地图。

至此initMap的结构和功能已经明确了：建立节点，确定所有的初始参数，设立所有的输入输出接口，设置标识（这部分被整合在了mp中）。而参数的具体功能则需要在接口的实现，即回调函数中明确。

### 回调函数的结构与功能

建图一共使用了2种传感器的数据，分别是深度相机、VINS，同时ego还提供了另一种方案（深度和里程计）和可补充的独立信息（激光雷达点云和里程），因此我们以深度相机、VINS及其发布的带时间戳的位置、里程、深度为基础解释回调函数的功能和流程。值得注意的是，真实传感器发布的数据类型命名往往各不相同，与程序对接时就需要修改参数。

#### 深度图回调depthPoseCallback()

我们根据建图的过程，首先是最核心的两个话题数据```grid_map/depth```和```grid_map/pose，在initMap中用参数```mp_.pose_type_ ```区别了带有时间戳的pose信息和历程信息odom，分别设立各自的订阅器和同步器。我们以pose为例，这个回调函数向我们展示了回调函数最为经典的使用方法：数据传递和标识设置。

```c++
void GridMap::depthPoseCallback(const sensor_msgs::ImageConstPtr &img,
const geometry_msgs::PoseStampedConstPtr &pose)
```

数据传递即要把指针所指的内容赋值到md内去，从而在类内计算；当数据类型不同时需要数据类型转换。

显然，点消息是最为简单的，```pose```和md的结构天差地别，但是其内部的数据无非是三维坐标和四元数7个变量，与```md_.camera_pos_```和```md_.camera_r_m_```完全对应，读指针赋值即可。

然而，图像数据则不一样，首先图像的数据量远超点，要做到深拷贝需要特殊手段；其次图像往往需要经过解码和尺寸整定才可使用，因此函数使用的是利用OpenCV库的智能指针实现解码和深拷贝。

标识设置就是控制整个程序的时序，由于回调函数基本上就是数据传输的接口函数，因此一个数据是否收到并处理完毕至关重要，下一步程序必须要收到相关的全局标识符才能执行。而类则轻易地实现了标识符的功能：
```c++
md_.occ_need_update_ = true; //栅格地图需要更新
md_.flag_have_ever_received_depth_ = true; //深度已收到
```

十分明显，这两个标识符和栅格地图更新```void GridMap::updateOccupancyCallback(const ros::TimerEvent & /*event*/)```紧密相连，可以直接看做实现了一个小状态机来控制时序，这也得益于ROS节点的功能。```checkDepthOdomNeedupdate()```通过后地图更新程序就启动了。

#### 图更新回调updateOccupancyCallback()

地图更新是ego中需要大量计算的环节，也是ego之所以为ego的成家之本，因此地图更新中加入了时间监控器```t = ros::Time::now();```并最后计算输出四个环节时长。同时由于ego的避障使用的是栅格地图，因此在处理坐标的时候往往需要double类型的实际值vector3d和整型的栅格索引值vector3i成对处理，并且能正确地互相转换。

ego使用局部地图进行避障，但由于路径规划的要求，坐标仍然需要全局坐标，因此同地图局部更新实现计算量的缩减。对于ego，其采用的是传统的方案，但2023年末发布的egoV2对其做了一次重大优化，即环形缓冲区（Ringbuffer）。由于缓冲区的坐标来自全局，因此

①更新地图首要就是更新环形缓冲区的范围```GridMap::moveRingBuffer()```，使其跟上机体速度；

②其次就需要更新缓冲区内栅格地图本身，这就需要对来自相机的数据，相机数据都是机载坐标系，需要转换成全局坐标系```GridMap::rojectDepthImage()```；

③然后利用相机中位于缓冲区内```GridMap::raycastProcess()```的点建立地图；

④最后构析之膨胀之```GridMap::clearAndInflateLocalMap```，传出栅格及膨胀地图。

- **moveRingBuffer()**

这其中，①的实现依靠索引和实际值的转换，即：固定大小的是栅格地图，即索引范围，但判断点是否在内需要的是实际值，因此①设计了大量的索引与实值的互相转化，好在EIGEN提供了转化的方式：

```c++
Eigen::Vector3d ringbuffer_lowbound3d_new = ringbuffer_lowbound3i_new.cast<double>() * mp_.resolution_;//即强制类型转换，设定分辨率（resolution）
```
现在，所有的数据都存入了md内，因此函数计算的局部变量都需要从md和mp赋值。md_.camera_pos_)提供相机位置，mp_.local_update_range3i_提供缓冲区范围（三维相同），mp_.inf_grid_提供缓冲区膨胀范围（三维相同），而实值也同步进行。

旧的缓存区需要删除，先让新旧的缓存区起始点三轴比较，确定每个轴的边界方向（正负，即上下界），然后通过```GridMap::clearBuffer(char casein, int bound)```来清除所有旧栅格。显然，首先比较确定清除相对于边界的方向比直接求解是否在边界中容易得多，以x轴为例，当缓冲区向正方向移动时，需要清理的是x轴的负半轴方向，即下界，因此边界结尾新旧的下边界（lowbound），为：
```c++
if (center_new(0) > md_.center_last3i_(0))
    clearBuffer(1, ringbuffer_lowbound3i_new(0));
...
for (int x = (casein == 0 ? bound : md_.ringbuffer_lowbound3i_(0)); x <= (casein == 1 ? bound : md_.ringbuffer_upbound3i_(0)); ++x)
    ...
    md_.count_hit_[id_buf] = 0;
	...
//for (int x = md_.ringbuffer_lowbound3i_(0)); x <= ringbuffer_lowbound3i_new(0); ++x)
```

可以看到，```GridMap::clearBuffer```清除的过程是通过三层for后改变md内标识实现的，然而还有判断其膨胀区的清理。对膨胀区来说，需要清理意味着障碍物的重要性降低，因此可以通过降低其障碍物概率实现，为了使地图平滑，就用以下方法更新：

```c++
if (md_.occupancy_buffer_inflate_[id_buf_inf] > GRID_MAP_OBS_FLAG)
    changeInfBuf(false, id_buf_inf, id_global);
...
inline void GridMap::changeInfBuf(const bool dir, const int inf_buf_idx, const Eigen::Vector3i global_idx)
{
  int inf_grid = mp_.inf_grid_;
  if (dir)
    md_.occupancy_buffer_inflate_[inf_buf_idx] += GRID_MAP_OBS_FLAG;
  else
    md_.occupancy_buffer_inflate_[inf_buf_idx] -= GRID_MAP_OBS_FLAG;//即若>32767,则减去32767，有符号int最大值，无符int的一半，因此处理后则呢么都小于32767的门槛了

  for (int x_inf = -inf_grid; x_inf <= inf_grid; ++x_inf) for ...for ...
      {
        Eigen::Vector3i id_inf(global_idx + Eigen::Vector3i(x_inf, y_inf, z_inf));
		nt id_inf_buf = globalIdx2InfBufIdx(id_inf);
        if (dir)
          ++md_.occupancy_buffer_inflate_[id_inf_buf];
        else
        {
          --md_.occupancy_buffer_inflate_[id_inf_buf];
          if (md_.occupancy_buffer_inflate_[id_inf_buf] > 65000) // An error case
              ...
```

即大于门槛使直接减去门槛，然后将膨胀范围内都自减达成平滑效果，最后检测错误（负概率），如此就实现了障碍栅格的删除。

接下来就是环形缓冲区之所以为环形的关键之处。从前面确定缓冲边界我们可以看到，其实缓冲区是用中心店和边界距直接算出的立方体，而“环形”指的是其数据存储的模式。因为当地图很大、进行时间很长时，缓冲区如果直接使用全局索引会导致索引值暴涨，这在内存中相当于又建了一个全局图，这对于计算和存储而言都是沉重的负担。因此，我们设定缓冲区有一个固定大小，但缓冲区所使用的索引又必须和全局对应，因此我们使用取余的方式，让缓冲区的大小当做弧度制的2π，就可以保证索引始终在最大值内，在数据上也就是保证内存限度。由于取模算法本身仍然是通过循环加减，因此程序直接用循环加减解决。

- **projectDepthImage()**

缓冲区经历区域更新、清理和索引整定后，就完成了更新地图的范围，接下来的步骤就是提取深度图的坐标。提取深度图的坐标原理非常简单：遍历深度图每个点，由深度得到相对相机的坐标，再用线性变换```camera_r * proj_pt + md_.camera_pos_```得到点的实际坐标。然而，这其中关键就在于如何计算坐标。深度相机通常会使用深度滤波器优化输出的数据，这使得数据来源有所不同：
```c++
for (int v = 0; v < rows; v += skip_pix)//先由mp获取图像长col宽row，skip_pixel_为单位遍历
//for (int v = mp_.depth_filter_margin_; v < rows - mp_.depth_filter_margin_; v += mp_.skip_pixel_)
{
    row_ptr = md_.depth_image_.ptr<uint16_t>(v);//这个名称具有巨大的误导性，实际上指的是prt(0,v)
    //row_ptr = md_.depth_image_.ptr<uint16_t>(v) + mp_.depth_filter_margin_;
    for (int u = 0; u < cols; u += skip_pix)
    {
        Eigen::Vector3d proj_pt;
        depth = (*row_ptr) / mp_.k_depth_scaling_factor_;//缩放因子
        row_ptr = row_ptr + mp_.skip_pixel_; //prt(u++,v)
        if (depth < 0.1)
            continue;
        proj_pt(0) = (u - mp_.cx_) * depth / mp_.fx_;//光学知识之相似变换，像素间距和实际间距与焦距f和深度d成正比
        proj_pt(1) = (v - mp_.cy_) * depth / mp_.fy_;
        proj_pt(2) = depth;//深度就是相机系的z
        proj_pt = camera_r * proj_pt + md_.camera_pos_;
        md_.proj_points_[md_.proj_points_cnt_++] = proj_pt;//计数为raycast做准备
    }
}
```

```c++
md_.last_camera_pos_ = md_.camera_pos_;
md_.last_camera_r_m_ = md_.camera_r_m_;
md_.last_depth_image_ = md_.depth_image_;//更新数据
```

- **raycast()**

下一个环节就是界定点的范围。我们先利用```md_.proj_points_cnt```_遍历判断缓冲区内点，再利用判断出的```md_.cache_voxel_cnt_```遍历出栅格地图。先来看第一个循环内：
```c++
md_.raycast_num_ += 1;//计数
int pts_num = 0;
for (int i = 0; i < md_.proj_points_cnt_; ++i){
    pt_w = md_.proj_points_[i];
    if (!isInBuf(pt_w)){//索引是否在bound3i内
        pt_w = closetPointInMap(pt_w, md_.camera_pos_);//找到最近地图边界点，以此形成缓冲区边缘
        pts_num++;	vox_idx = setCacheOccupancy(pt_w, 0);//0：自由空间（外部默认），1：occupied（区内默认）
    }
    else{
        pts_num++;	vox_idx = setCacheOccupancy(pt_w, 1);
    } 
```

> 一个次要的点是，程序中有避免重复的算法：
> ```c++
> if (vox_idx != INVALID_IDX){//有实值
> 	if (md_.flag_rayend_[vox_idx] == md_.raycast_num_)//若标志（体素索引）已经等于计数器
> 		continue;
> 	else
> 		md_.flag_rayend_[vox_idx] = md_.raycast_num_;
> }
> ```
>
> ```md_.raycast_num_```是体素（voxel）处理计数器，也表明当前ray过程的体素是第几个，一半情况下先计数再更新体素标识，但如果```md_.flag_rayend_[vox_idx]```唤出的当前ray过程体素索引已经等于计数器，就说明此体素已经被计算了。
>
> 此后在traverse过程也同理，即```if (md_.flag_traverse_[vox_idx] == md_.raycast_num_)```

至此就正式开始raycast过程。Raycast利用```raycaster.setInput()```和```while (raycaster.step(ray_pt))```实现，模拟“用光线（ray）拍摄（cast）”的思想，使得判断障碍的过程变为模拟激光雷达，因此在```while (raycaster.step(ray_pt))```内，```ray_pt```就是模拟光线的坐标，在```raycaster.step()```内实现光线更新，在while内将光线所到之处由默认的1：occupied转为0：free，直到检测穿越已计算点（通过避免重复算法，即```if (md_.flag_traverse_[vox_idx] == md_.raycast_num_)```
）。接下来可以看到实现方法：
```c++
raycaster.setInput(pt_w / mp_.resolution_, md_.camera_pos_ / mp_.resolution_);//d转i，除分辨率
while (raycaster.step(ray_pt)){
    Eigen::Vector3d tmp = (ray_pt + Eigen::Vector3d(0.5, 0.5, 0.5)) * mp_.resolution_;//有趣的是(0.5, 0.5, 0.5)才是每个体素的中心，因此总是需要这个偏置向量保证i-d转换顺畅
    pts_num++;
    vox_idx = setCacheOccupancy(tmp, 0);//由默认的1置0
    if (vox_idx != INVALID_IDX){//查重，与上一个不同的是此处的重复具有实际意义（障碍）
        if (md_.flag_traverse_[vox_idx] == md_.raycast_num_)
        ...
```

第二个循环是遍历体素，因为栅格地图并非直接由raycast的结果确定，而是基于概率的，比如前面提到的GRID_MAP_OBS_FLAG=32767。因此需要根据raycast得到的数据综合出，raycast的主要工作也就是计数和传递栅格信息```setCacheOccupancy(tmp, 0)```，而```setCacheOccupancy(tmp, 0)```的作用是给tmp点计数是否打击（hit或miss），经过整轮raycast后每个点大概率都既有hit计数也有miss计数，因此需要通过概率来确定，实现如下：
```c++
for (int i = 0; i < md_.cache_voxel_cnt_; ++i)
{
    int idx_ctns = globalIdx2BufIdx(md_.cache_voxel_[i]);//缓冲区内，先转到环区索引
    double log_odds_update =
        md_.count_hit_[idx_ctns] >= md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns] ?
        mp_.prob_hit_log_ : mp_.prob_miss_log_;//此点raycast击中概率是否大于50%？更具结果整定为70和35（由mp决定）
    //程序内其实只用到了hit和总数，而没有单独的miss
    md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;//击中计数置零，注意清理旧缓存区时也给这些数据置零了
    md_.occupancy_buffer_[idx_ctns] =
        std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update, mp_.clamp_min_log_),
 mp_.clamp_max_log_);//此栅格的最终结果
  }
```

关于栅格最终认定的算法需要展开描述，首先最外层的min是为了保证取值，而```mp_.clamp_min_log_```是概率```md_.occupancy_buffer_[]```的初始值，并不为零而是接近临界以灵活变化，而```md_.occupancy_buffer_[idx_ctns] + log_odds_update```就是概率的更新值了，因此该代码实现了概率更新值保持在上下限内。

- **clearAndInflateLocalMap()**

最后一个部分非常简单，甚至可以列出所有代码。在此遍历所有参与计算的体素，在环区内，判断是否膨胀。
```c++
for (int i = 0; i < md_.cache_voxel_cnt_; ++i){
    Eigen::Vector3i idx = md_.cache_voxel_[i];
    int buf_id = globalIdx2BufIdx(idx);
    int inf_buf_id = globalIdx2InfBufIdx(idx);

    if (md_.occupancy_buffer_inflate_[inf_buf_id] < GRID_MAP_OBS_FLAG && //未膨胀
        md_.occupancy_buffer_[buf_id] >= mp_.min_occupancy_log_)//障碍物，需要设置膨胀
		changeInfBuf(true, inf_buf_id, idx);
    if (md_.occupancy_buffer_inflate_[inf_buf_id] >= GRID_MAP_OBS_FLAG && //已膨胀
        md_.occupancy_buffer_[buf_id] < mp_.min_occupancy_log_)//非障碍，需要取消膨胀
		changeInfBuf(false, inf_buf_id, idx);
  }
```

可以看到，膨胀地图仍然是概率的，但通过`getInflateOccupanc()`访问时：
```c++
inline int GridMap::getInflateOccupancy(Eigen::Vector3d pos){
  if (!isInInfBuf(pos))
    return 0;
  if (mp_.enable_virtual_walll_ && (pos(2) >= mp_.virtual_ceil_ || pos(2) <= mp_.virtual_ground_))
    return -1;
	return int(md_.occupancy_buffer_inflate_[globalIdx2InfBufIdx(pos2GlobalIdx(pos))]);
}
```
可以看到先检测是否在缓冲区、z轴地面和天花板间，然后直接强制类型转换，实际上就是四舍五入为0和1.

到此就完成了**updateOccupancyCallback()**的全部流程实现。到此我们也可以看到，整个建图流程中其实只用到了深度图信息，而整个建图过程也主要调用此函数。其他的回调函数则占据了辅助的功能，相对重要的是两个计时器，他们分别执行了可视化和地图衰减两个进程；剩下的回调函数就是单纯的数据处理，他们通过写入md间接地影响建图。

#### 雷达点云回调cloudpointsCallback()

然而，无人机应用最广泛的还是激光雷达。和深度图不同，无需通过raycast步骤“模拟”激光雷达，而可以直接认为每一个点云都对应一个实在的障碍物，因此使用激光雷达的情况下整个建图直接通过此回调函数就可以实现，而除了缓冲区移动函数外无需引用其他函数。

点云回调函数的过程可以看做深度回调的简化版：检查里程和点云都有信息后，移动缓存区，遍历判断点云是否在缓存区内，筛选后将点所在栅格概率置最大，然后膨胀之。

值得注意的是，点云默认使用全局坐标，但对于大多数激光雷达而言这都是奇怪的过分的设置，不过只需要加上偏移量即可。

#### 两个定时器

```GridMap::fadingCallback```对occupied格子的障碍概率进行低频定时衰减，对长时间没有被传感器刷新的静态障碍物就会因衰减而倾向被忽略，衰减的定量操作如下：
```c++
const double reduce = (mp_.clamp_max_log_ - mp_.min_occupancy_log_) / (mp_.fading_time_ * 2); // function called at 2Hz
const double low_thres = mp_.clamp_min_log_ + reduce;//避免减到取值外
```
对障碍概率大于low_thres的格子，概率都减去reduce，然后与```mp_.min_occupancy_log_```判断；如果本来是障碍物，衰减后却概率低于障碍阈值，则```changeInfBuf(false, inf_buf_idx, idx);```.

第二个定时器是```GridMap::visCallback```，其实就是栅格和膨胀地图的发布器。两个发布器是同构的，因此以栅格地图发布器为例：
```c++
void GridMap::publishMap(){
    if (map_pub_.getNumSubscribers() <= 0)
    	return;
    Eigen::Vector3d heading = (md_.camera_r_m_ * md_.cam2body_.block<3, 3>(0, 0).transpose()).block<3, 1>(0, 0);//heading是相机方向向量，即相机旋转矩阵和相机到机体矩阵相乘
    pcl::PointCloud<pcl::PointXYZ> cloud;
    double lbz = mp_.enable_virtual_walll_ ? max(md_.ringbuffer_lowbound3d_(2), mp_.virtual_ground_) : md_.ringbuffer_lowbound3d_(2);//地面ground
    double ubz = mp_.enable_virtual_walll_ ? min(md_.ringbuffer_upbound3d_(2), mp_.virtual_ceil_) : md_.ringbuffer_upbound3d_(2);//天花板ceil，这是单独计算了为了z轴上下界
    if (md_.ringbuffer_upbound3d_(0) - md_.ringbuffer_lowbound3d_(0) > mp_.resolution_ &&
       (md_.ringbuffer_upbound3d_(1) - md_.ringbuffer_lowbound3d_(1)) > mp_.resolution_ && 
       (ubz - lbz) > mp_.resolution_)//检测缓冲区范围大于分辨率单位
    	for (double xd = md_.ringbuffer_lowbound3d_(0) + mp_.resolution_ / 2; //这个mp_.resolution_ / 2相当于0.5，用来方便i-d转化
             xd <= md_.ringbuffer_upbound3d_(0); 
             xd += mp_.resolution_)
    		for (double yd = md_.ringbuffer_lowbound3d_(1) + mp_.resolution_ / 2; 
 yd <= md_.ringbuffer_upbound3d_(1); 
 d += mp_.resolution_)
    			for (double zd = lbz + mp_.resolution_ / 2; 
     zd <= ubz; 
     zd += mp_.resolution_){
					Eigen::Vector3d relative_dir = 
        (Eigen::Vector3d(xd, yd, zd) - md_.camera_pos_);//此点的相对向量
    				if (heading.dot(relative_dir.normalized()) > 0.5)//是否在相机前方，点积判断角度
    					if (md_.occupancy_buffer_[globalIdx2BufIdx(pos2GlobalIdx(Eigen::Vector3d(xd, yd, zd)))] >= mp_.min_occupancy_log_)//最终occupied判断
   							 cloud.push_back(pcl::PointXYZ(xd, yd, zd));//若occupied，作为点云发布
    //以下是点云消息设置
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = mp_.frame_id_;
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    map_pub_.publish(cloud_msg);//发布
}
```

虽然代码形态非常复杂，但逻辑异常简单，遍历所有点，根据栅格概率判断是否occupied，是则加入点云，最后发布点云。

至此建图程序的基本功能就实现了，剩下的也只是其他传感器的回调函数。

## 规划（planner_manage）

planner_manage是整个程序的主体，main所在地，所有信息的交汇点，ego主体实现之处。规划器比起建图器来说是一个更为庞大的个体，他的节点启动、状态机、主要程序和相对独立的部分都分开为四个.cpp文件。我们可以按照程序启动的顺序依次讲解。

ego_planner_node.cpp节点文件内就是主函数，呈现为建图一节开头例子中的经典形态：
```c++
...
  EGOReplanFSM rebo_replan;
  rebo_replan.init(nh);//实现函数
...
```
可以看到首先启动的是EGOReplanFSM，状态机是整个规划部分的主体和中心，也是最庞大的.cpp文件。```EGOReplanFSM::init```函数中启动planner_visualization和planner_manage：

```c++
...
visualization_.reset(new PlanningVisualization(nh));
planner_manager_.reset(new EGOPlannerManager);
planner_manager_->initPlanModules(nh, visualization_);
...
```
其中```visualization_```来自tra_utils包，执行可视化操作，而```planner_manager_```自然来自planner_manager.cpp，句柄nh传递到两个模块中，同时```visualization_```也和句柄传递到```planner_manager_```内，再加上状态机本身，可以看到携带句柄的节点有三个了。

traj_server是一个单独的.cpp，它的内部也有一个主函数，负责订阅轨迹和时间信息，再发布控制指令。因此，可以说实质上规划功能是两个部分组成的：状态机和轨迹服务器。

### 状态机（EGOReplanFSM）

#### 状态机初始化EGOReplanFSM::init()

```EGOReplanFSM::init```与建图部分的init算法几乎同构，但作为状态机，其显然增加了许多标识。因此仍然可以分为三部分：①导入句柄，②参数配置，③初始化回调函数，④初始化标识。

导入句柄部分①已经在上文有所展示，即将句柄传入```visualization```_以及```planner_manager_```使其启动。

而参数配置②导入的除了设置参数外，值得注意的是waypoints（下称路点）的初始化：
```c++
nh.param("fsm/waypoint_num", waypoint_num_, -1);
    for (int i = 0; i < waypoint_num_; i++){
      nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);//初设为-1，便于错误检测
      ...
```

在标识作用前空谈无用，④按下不表。因此主要的工作就集中在回调函数③中，具体来说集中在规划的2个定时器，4个订阅器和5个发布器。

- 发布器发布minco轨迹、多项式轨迹、数据展示、心跳和对地高度测量。其中心跳heartbeat_msg和数据展示data_disp_都是带时间戳到状态信息，因此实际的数据输出就是两个轨迹和对地高度测量3种。

- 两个定时器分别负责执行操作和碰撞检测，因此```exec_timer_```对应```execFSMCallback()```，```safety_timer`_``对应```checkCollisionCallback()```，这就是两个主循环了。

- 订阅器主要订阅的是指令和数据。指令指的是强制停止```mandatory_stop```，而数据就是里程、minco轨迹和目标点。其中目标点需要依靠```target_type_```条件判断用预设目标（PRESET）还是手动目标（MANUAL）。两个模式的区别非常大，调用的函数和状态机流程都完全不同，例如在init内，PRESET就直接可以开始`readGivenWpsAndPlan();`启动读点了。
  - 因此定时器和订阅器的7个回调函数就是功能区了。

#### 状态机回调execFSMCallback()

```execFSMCallback()```就是整个规划过程的状态机。状态机需要执行推送时间，每500循环发布一次状态，以及核心的状态切换与每个状态的功能函数调用。状态机一共有七个状态，如下：

```c++
void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e){
    exec_timer_.stop(); // To avoid blockage
    std_msgs::Empty heartbeat_msg;
    heartbeat_pub_.publish(heartbeat_msg);

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 500){
      fsm_num = 0; printFSMExecState();
    }

    switch (exec_state_){
        case INIT:
            changeFSMExecState()//此处列出该状态唤起的所有函数
        case WAIT_TARGET:
            changeFSMExecState()
        case SEQUENTIAL_START:
            changeFSMExecState()
        case GEN_NEW_TRAJ:
            planFromGlobalTraj(10),changeFSMExecState()
        case REPLAN_TRAJ:
            planFromLocalTraj(1),changeFSMExecState()
        case EXEC_TRAJ:
            mondifyInCollisionFinalGoal(),planNextWaypoint(),changeFSMExecState()
        case EMERGENCY_STOP:
            callEmergencyStop(),changeFSMExecState()
	}
	data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);

  force_return:;//goto标识，来自状态机内的goto代码，若状态不改变则直接跳出switch
  exec_timer_.start();
```

首先可以发现```exec_timer_```计时完全绕过了状态机，说明执行时间是不包含状态切换时间的，这是为了避免多个程序同时调用```exec_timer_```出现堵塞。然后可以看到状态和每个状态的调用函数。毋庸置疑，每个状态都有切换状态的功能，因此都有```changeFSMExecState()```，由这个函数遍历状态列表。

前三个状态```INIT```，```WAIT_TARGET```，```SEQUENTIAL_START```是纯粹的时序状态，单纯根据标识来等待和执行状态切换，其时序依次串联的，最后从```SEQUENTIAL_START```通向```EXEC_TRAJ```。以```INIT```为例：
```c++
case INIT:{
	if (!have_odom_)
		goto force_return;
	changeFSMExecState(WAIT_TARGET, "FSM");
	break;
}
//WAIT_TARGET： if (!have_target_ || !have_trigger_)
case SEQUENTIAL_START：{
    if (planner_manager_->pp_.drone_id <= 0 || 
       (planner_manager_->pp_.drone_id >= 1 && 
        have_recv_pre_agent_))
        if (planFromGlobalTraj(10))
            changeFSMExecState(EXEC_TRAJ, "FSM");
    ...
```
可以看到就是一个条件判断，未满足条件就不切换状态，反之到下一个状态。`INIT`需要里程信息就绪`have_odom_`；`WAIT_TARGET`需要目标以及触发`have_target_ &&have_trigger_`；而S`EQUENTIAL_START`比较复杂，无人机ID小于0时通过，而大于等于1时需要`have_recv_pre_agent_`，最后进行全局规划，若成功则进入下一个状态。

而`GEN_NEW_TRAJ`，`REPLAN_TRAJ`和`EXEC_TRAJ`则是具体的规划状态，对应生成、重规划和执行轨迹；前两个比较次要，只是控制`planFromGlobalTraj()`生成轨迹的个数，若成功就转到`EXEC_TRAJ`。所有`planFromGlobalTraj()`都需要在`EXEC_TRAJ`转态内执行，因此又可以说规划主要靠`planFromGlobalTraj()`和`EXEC_TRAJ`内的`mondifyInCollisionFinalGoal()`,`planNextWaypoint()`这三个函数实现。

最后提供了一个故障状态EMERGENCY_STOP用于紧急停止，先再次判断`flag_escape_emergency_`避免重复判断，通过后转进`callEmergencyStop(odom_pos_)；`用于调用位于`planner_manager`内的`EmergencyStop()`。

至此状态机3+3+1的结构也就完全了，那么规划的主体我们也得以窥见，也就是`EXEC_TRAJ`：
```c++
case EXEC_TRAJ:{
    ...//参数计算
    if (mondifyInCollisionFinalGoal()) 
        // case 1: 目标点在障碍物内，跳过
        
    else if ((target_type_ == TARGET_TYPE::PRESET_TARGET) &&
              (wpt_id_ < waypoint_num_ - 1) &&
              (final_goal_ - pos).norm() < no_replan_thresh_) 
        // case 2: 未达目标，分配下一个路点
    	wpt_id_++; planNextWaypoint(wps_[wpt_id_]);
    
    else if ((t_cur > info->duration - 1e-2) && touch_the_goal){ //-1e-2是处理相等情况
        // case 3: 抵达目标
        have_target_ = false;
        have_trigger_ = false;
     	if (target_type_ == TARGET_TYPE::PRESET_TARGET)//为下一循环做准备，重置路点索引
    		wpt_id_ = 0; planNextWaypoint(wps_[wpt_id_]);
    	changeFSMExecState(WAIT_TARGET, "FSM");    // 规划任务完成，回到等待状态
    }
    
    else if (t_cur > replan_thresh_ || (!touch_the_goal && close_to_current_traj_end)) 
        // case 4: 重规划
    	changeFSMExecState(REPLAN_TRAJ, "FSM");
    break;
}
```

`EXEC_TRAJ`的主体就是这四种情况的条件判断，剩下的部分就是计算条件判断所需要的标识。

- case 1很好理解，我们也可以轻易看到障碍判别是如何进行的：整体逻辑是从轨迹末端开始循环判断是否在障碍物中，若在则从目标点退一步再次检测，若有非障碍点则将目标点重设为此点，若无则整条轨迹都在障碍中，发布错误提示。而其实现就是t_step利用分辨率进行，轨迹的尽头用duration索引，pt从duration依次减去t_step直到出发点。

```c++
//其实冗长之处在于唤出信息其实是从planner_manager中调用的，可以看到在建图部分我们熟悉的访问函数`getInflateOccupancy()`在此处就是如此调用的，在此做了省略用“.”代替引用。		
	double t_step = .getResolution() / .max_vel_;
	for (double t = .duration; t > 0; t -= t_step){
        Eigen::Vector3d pt = .getPos(t);
        if (!.getInflateOccupancy(pt))
          if (planNextWaypoint(pt)) // 重设目标点为找到的非障碍点
            return true;//这里省略了ROS_INFO
        if (t <= t_step)//如此判断和之前的low_thres一样，避免减到0溢出
          ROS_ERROR(...);
      }
    }
	return false;
```

- case 2的判断除了单纯没到目标点之外，还要求到目标点的距离小于重规划阈值、且目标是预设模式，也就是说对实时发布路点的情况是不会用到这个情况的。

- case 3的为抵达判断加了两重保险，然后重置标识，回到等待状态。值得注意其考虑了TARGET的两种情况。
```c++
double t_cur = min(.duration, .toSec()-.start_time);//时间判断
bool touch_the_goal = ((local_target_pt_ - final_goal_).norm() < 1e-2);//距离判断
```

- case 4即时间长于阈值后执行重规划。对于PRESET模式来说，和目标点的距离要大于阈值才定时重规划。而对于MANUAL模式，case 4就是主要的情况，只要还没到目标且离目标有一定距离，就一直保持重规划而非定时（定时和未达目标是或关系）。 

到此，所有发点操作都仅限于PRESET模式下的，这是因为PRESET需要从配置文件读取，也就是在程序中直接控制读取，无法实现异步，因此需要在状态机中明确；而MANUAL模式下，系统读取路点话题发送过来的路点，然后在路点回调函数直接进入`planNextWaypoint()`，而作为回调函数，发点都是异步的，因此在回调函数中明确执行前后应处状态即可：
```c++
void EGOReplanFSM::waypointCallback(const quadrotor_msgs::GoalSetPtr &msg){
    if (msg->drone_id != planner_manager_->pp_.drone_id || msg->goal[2] < -0.1)
        return;
    ROS_INFO("Received goal: %f, %f, %f", msg->goal[0], msg->goal[1], msg->goal[2]);
    Eigen::Vector3d end_wp(msg->goal[0], msg->goal[1], msg->goal[2]);
    if (planNextWaypoint(end_wp))//异步调取，不在WAIT状态时执行，转到REPLAN
        have_trigger_ = true;
}
```


于是洋葱层层剥开，实际执行规划的就是`planNextWaypoint(vetor3d)`（MANUAL）和`planFromGlobalTraj(num)`（PRESET）。以MANUAL模式为例，发现`planNextWaypoint`还是一个洋葱：

```c++
bool EGOReplanFSM::planNextWaypoint(const Eigen::Vector3d next_wp){
    std::vector<Eigen::Vector3d> one_pt_wps;
    one_pt_wps.push_back(next_wp);//再套一个vector送进规划

    bool success = false;
    success = planner_manager_->planGlobalTrajWaypoints(//这个才是关键
        odom_pos_, odom_vel_, Eigen::Vector3d::Zero(),
        one_pt_wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (success){
		final_goal_ = next_wp;
		...//display 
		have_target_ = true;
		have_new_target_ = true;
		if (exec_state_ != WAIT_TARGET)// FSM，通往REPLAN_TRAJ
			...
		visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
		ROS_ERROR("Unable to generate global trajectory!");
    return success;
  }
```

可以发现，`planner_manager_->planGlobalTrajWaypoints(...)`才是真正的规划函数，剩下的都是状态切换部分和展示部分。而PRESET模式下，`planFromGlobalTraj(num)`调用`callReboundReplan()`，后者调用的`getLocalTarget`和`reboundReplan`也进入了planner_manager部分。

MANUAL和PRESET模式都是极为粗糙的给点方式，无论是仿真还是实机的实践中我们都需要中途给点，也就是说：存在ROS节点不定时地发出目标点，而EGO中需要有相应的订阅器和回调函数处理。要实现这个功能，必须提供一个新的模式，对应一个新的`TARGET_TYPE`，并且需要自定义编程将其嵌入状态机中；ego高结构化特性为自定义模式提供便利，只需要调用`planFromGlobalTraj(num)`或`planNextWaypoint(vetor3d)`即可。

那么可以在此总结状态机发点的控制方法：MANUAL模式下，初始化和等待状态通过后，每收到路点就执行一次，并重规划直到抵达终点；PRESET下则需要经历SEQUENTIAL和REPLAN等状态转换读点和重规划，直到抵达终点。而每个状态内最终都需要调用管理器内的函数实现功能，状态机的函数实现了接口的功能。

#### 检碰回调checkCollisionCallback()

回调函数中，只有两个比较复杂：安全定时器和minco轨迹回调，其余都是简单的数据变换和标志改变，`RecvBroadcastMINCOTrajCallback()`又是无人机集群用于接受其他无人机轨迹消息的，因此按下不表。

在代码的注释可以看到分为：检查高度，检查深度丢失，检查轨迹。前两个检测非常简单，例如`if (map->getOdomDepthTimeout())`，因此主要的工作在于轨迹检查。轨迹检查分为①寻找轨迹起点，②检查碰撞，③危险处理。

zero 首先注意轨迹的数据是如何调出的，这可以看做整个工程各处常省略部分的缩影：
```c++
LocalTrajData *info = &planner_manager_->traj_.local_traj;//指针
auto map = planner_manager_->grid_map_;
const double t_cur = ros::Time::now().toSec() - info->start_time;
PtsChk_t pts_chk = info->pts_chk;//读指针，PtsChk_t是一个二维数组，表示轨迹段和内点
double t_temp = t_cur;
int i_start = info->traj.locatePieceIdx(t_temp);
```

①寻找轨迹起点就是通过遍历`pts_chk`从0开始查找t_cur处的轨迹；值得注意的是他提供了双层循环使用goto中断的范本：
```c++
for (; i_start < (int)pts_chk.size(); ++i_start){
    for (j_start = 0; j_start < pts_chk[i_start].size(); ++j_start)
    	if (pts_chk[i_start][j_start].first > t_cur)//第一次出现大于t_cur的点，即是起点
    		goto find_ij_start;//跳出双层循环
}
find_ij_start:;
```

②检查碰撞同样是遍历轨迹通过：
```c++
Eigen::Vector3d p = pts_chk[i][j].second;
bool dangerous = false;
dangerous |= map->getInflateOccupancy(p);// |=是自或，和+=、-=类似
//注意，省略了集群无人机避障部分
```

③危险处理主要是ROS_ERROR信息，并要求重规划（转换状态）。

### 管理器（planner_manager）

规划管理器肩负着规划的主要功能实现，在状态机部分我们已经得到了5个与管理器连接的部分：模块初始化initPlanModules()、MANUAL模式的全局规划`planGlobalTrajWaypoints()`、PRESET的局部目标获取`getLocalTarget`和反弹重规划`reboundReplan`，急停模块从调用的`EmergencyStop()`。这5个函数就是管理器的主体，剩下的checkCollision()用于集群避碰按下不表，`computeInitStateset()`和`LocalTrajFromOpt()`都是管理器内部被调用的功能函数。

#### 管理器初始化initPlanModules()

数据传输和时序控制都整合到了状态机内，因此初始化函数中没有回调函数也不用标识设置，从四部分分为两部分：句柄专递，参数配置。
> 四部分：句柄专递，参数配置，回调函数初始化，标识设置

参数配置大同小异；而句柄启动了栅格地图和轨迹优化，同样是用reset并传递句柄nh，**特殊之处**是：管理器、轨迹优化、栅格地图、可视化（还有暂时忽略的集群）都要相互连接指针，才能互相引用数据：

```c++
void EGOPlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis)
...
grid_map_.reset(new GridMap);
grid_map_->initMap(nh);

ploy_traj_opt_.reset(new PolyTrajOptimizer);
ploy_traj_opt_->setParam(nh);
ploy_traj_opt_->setEnvironment(grid_map_);//轨迹优化与栅格地图连接
ploy_traj_opt_->setSwarmTrajs(&traj_.swarm_traj);//轨迹优化和集群连接
ploy_traj_opt_->setDroneId(pp_.drone_id);//集群设置

visualization_ = vis;//管理器和可视化连接，vis是和句柄一起传入的指针，在状态机init中启动vis并传入的
```

#### MANUAL分支

先从MANUAL的情况入手，路点回调`waypointCallback()`将收到的点传入“接口函数”`planNextWaypoint()`，再塞入管理器的`planGlobalTrajWaypoints()`内。因此MANUAL下所有的路点最终都抵达此处参与规划，所有的轨迹都从此函数开始。

由于没有状态机而无需标识改变，因此函数从四部分变为三部分：轨迹生成（功能），数据准备，数据检查。功能上planGlobalTrajWaypoints()是通过调用轨迹优化来实现规划的，即：
```c++
poly_traj::MinJerkOpt globalMJO;
globalMJO.reset(headState, tailState, waypoints.size());//初始化minco曲线内存
globalMJO.generate(innerPts, time_vec);//生成minco曲线
...//minco曲线检查
auto time_now = ros::Time::now();
traj_.setGlobalTraj(globalMJO.getTraj(), time_now.toSec());//实例化轨迹
```

`poly_traj::MinJerkOpt`是一个minco对象，而最终我们使用的轨迹是`TrajContainer traj_`，因此本段代码的全部内容就是生成一个minco曲线，并将其实例化一个轨迹类`traj_`。为了区分这个过程，minco用曲线指代，而traj才用轨迹。

函数需要提供的信息是：起始状态，路点数量，内点，路点时间；而函数传入的是起始点的位置、速度、加速度（即起点状态），终点的速度、加速度（终点状态），以及路点集合。状态和路点都能轻易实现，关键就落在了内点生成与时间计算上。

```c++
if (waypoints.size() > 1){
	innerPts.resize(3, waypoints.size() - 1);
	for (int i = 0; i < (int)waypoints.size() - 1; ++i)
		innerPts.col(i) = waypoints[i];
...//else
```

内点的生成就是每个路点间插一个点，因此为路点数减1，再把路点除终点都导入内点中，此后就都以内点坐标作为计算依据。
```c++
double des_vel = pp_.max_vel_ / 1.5;//参考速度是最大速度的2/3
Eigen::VectorXd time_vec(waypoints.size());
	for (int j = 0; j < 2; ++j)
		for (size_t i = 0; i < waypoints.size(); ++i)
			time_vec(i) = (i == 0) ? 
			              (waypoints[0] - start_pos).norm() / des_vel: 
			              (waypoints[i] - waypoints[i - 1]).norm() / des_vel;//仍然是避免索引减溢出
```

路点时间则直接用路点间距除以参考速度求得，作为minco的生成初始化。

> 之所以把这两部分简单的代码提出，是为了展示一些经典代码写法。内点所使用的数据类型是`Eigen::MatrixXd`，即动态双精度浮点矩阵，因此其大小设定是通过`.resize`实现的；而路点的代码内曲折地进行了一次非零判断，也是为了避免索引减到负值导致溢出。
检查则针对minco曲线的最大速率，因为minco生成的速度会在参考速度上下浮动，`globalMJO.getTraj().getMaxVelRate()`可以获取曲线中最大速率，再加上首位速度即可形成约束检查，若有超过约束，则将dev_vel再缩小为2/3.

> 此处有一个逻辑疑点，`for (int j = 0; j < 2; ++j)`内有一个`if(j==2)`的支路，不知用意。

至此轨迹的生成就完整了，返回`true`用于状态机内的判断。

#### PRESET分支

获取局部目标`getLocalTarget`和反弹重规划`eboundReplan`就是PRESET模式的规划函数。其中，`eboundReplan`调用了计算初状态`computeInitState`和从优化器设置局部轨迹`setLocalTrajFromOpt`两个函数，因此几乎是整个管理器的余下内容。这两个函数都是从状态机的调用反弹重规划`callReboundReplan`唤起，因此此处需要先介绍之前忽略的PRESET分支状态机。

- 在SEQUENTIAL和GEN状态下调用了全局坐标规划`planFromGlobalTraj`，而REPLAN状态调用的是局部轨迹规划`planFromLocalTraj`，这反映了**生成轨迹用全局，重规划针对局部轨迹**的算法。

  - 而`planFromGlobalTraj`的内容只有更新标识随机多项式初始化标志`flag_random_poly_init`的状态以及调用`callReboundReplan`直到成功；

  - `planFromLocalTraj`则需要从管理器`planner_manager_->traj_.local_traj`读点（依然通过指针info传递）再调用`callReboundReplan`。

  - > 两个函数都设定了最大尝试次数避免卡死。

- 因此“洋葱”来到了`callReboundReplan`上，而此函数更是异常简单：调用`getLocalTarget`和`eboundReplan`，并在他们又返回之后发布轨迹消息。一个典型的发布器在建图最后部分的代码的最后部分中有一个实例（点云发布），此处不作展示。

综上，PRESET在状态机内只是区分了点的来源，当拿到点后**都**是通过调用`getLocalTarget`和`eboundReplan`实现规划。

`getLocalTarget`是一个简单的函数，即给轨迹再做一次处理，但由于轨迹类内信息众多，使代码的可读性很差。三段处理分别是：遍历轨迹将轨迹局限在规划域（`planning_horizen`）内并整定之，时间判断是否抵达终点并设置标识，判断能否急停到终点并设置速度（不能急停需要置零）。第一个和第三个处理其实都是一些基于轨迹的约束。

`reboundReplan`是一个复杂的函数，与其调用的`computeInitState`一起组成整个管理器最庞大的代码块。幸而代码的注释提供了结构，即①初始化、②优化、③存储与展示。

- **初始化**

①初始化的主要功能就是a.设目标点 b.计算初始状态和 c.获取并可视化约束点。获取约束点是轨迹优化中的内容，暂时不表；目标点则轻易使用`ploy_traj_opt_->setIfTouchGoal(touch_goal)`设置抵达标志位，因此目前初始化的主要内容就是状态计算函数`computeInitState()`.

计算初始状态考虑2种情况：

`if (flag_first_call || flag_polyInit)`：其中`flag_first_call`已被置1，因此对应后者即多项式初始化情况，minco的初始化如下：
```c++
initMJO.reset(headState, tailState, piece_nums);
initMJO.generate(innerPs, piece_dur_vec);//初始化轨迹
poly_traj::Trajectory initTraj = initMJO.getTraj();//轨迹初始化

initMJO.reset(headState, tailState, piece_nums);
initMJO.generate(innerPs, piece_dur_vec);//实现轨迹
```

虽然和MANUAL模式的初始化类有所不同（不是globalMJO），轨迹的实现也不太一样，但流程也基本相同，那么主要内容仍然是装填首尾状态，计算内点和参考时间。

初始化轨迹往往是为后续的启发式算法服务的，无论是刚开始还是前一次规划失败，都力图生成一个与数据无关的、尽量随机且尽量简单的轨迹，一个很自然的想法便是：**单内点，且内点从起始到终点直线上偏移**，这样生成的轨迹尽量简单便于后续约束，同时当失败次数变多后可以调整内点位置改变初始解。所以问题的关键就是生成初始点到单内点的向量。

程序使用的是分量偏移的方法，即生成与直线路径方向垂直的两个向量，以直线终点为基础设置分别加入两分量的随机偏移量，偏移量与之前的规划失败次数正比，代码如下：
```c++
Eigen::Vector3d horizen_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();//(start_pt - local_target_pt)就是直线路径向量，与z单位向量叉积得到水平面向量1
Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizen_dir)).normalized();//与horizen_dir垂直的向量，horizen_dir，vertical_dir和(start_pt - local_target_pt)互相垂直，形成直角系
innerPs.resize(3, 1);//单内点
innerPs = (start_pt + local_target_pt) / 2 +//从直线的中点开始
(((double)rand()) / RAND_MAX - 0.5) *(start_pt - local_target_pt).norm() *
    horizen_dir * 0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989) +
(((double)rand()) / RAND_MAX - 0.5) *(start_pt - local_target_pt).norm() *
    vertical_dir * 0.4 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989);
//分量系数：随机值+直线距离+失败次数+固定参数
piece_nums = 2;
piece_dur_vec.resize(2);
piece_dur_vec = Eigen::Vector2d(init_of_init_totaldur / 2, init_of_init_totaldur / 2);//只有两段，十分好算
```

而实现曲线则是和MANUAL一样的过程规划。然而，内点设置逻辑不是在路点间插，而是先通过配置中的参考段距离预分配：
```c++
piece_nums = round((headState.col(0) - tailState.col(0)).norm() / pp_.polyTraj_piece_length);
if (piece_nums < 2)//不能减到0了内点，也同样设为piece_nums - 1个。
	piece_nums = 2;
```
`else`则说明第二种情况是在先前的轨迹基础上再优化，而非初始化。此时对策是：反馈并等待，如果抵达时限，则初始化一次实现曲线。

- **优化**

优化最瞩目的部分是依据`pp_.use_multitopology_trajs`分了多拓扑优化和单拓扑优化两种情况。

先从单拓扑入手：

```c++
poly_traj::Trajectory initTraj = initMJO.getTraj();//传入之前的初始化轨迹
int PN = initTraj.getPieceNum();
Eigen::MatrixXd all_pos = initTraj.getPositions();
Eigen::MatrixXd innerPts = all_pos.block(0, 1, 3, PN - 1);//算内点，轨迹已初始化，直接插点做内点
Eigen::Matrix<double, 3, 3> headState, tailState;
headState << initTraj.getJuncPos(0), initTraj.getJuncVel(0), initTraj.getJuncAcc(0);
tailState << initTraj.getJuncPos(PN), initTraj.getJuncVel(PN), initTraj.getJuncAcc(PN);//算状态
double final_cost;
flag_success = ploy_traj_opt_->optimizeTrajectory(headState, tailState,
  innerPts, initTraj.getDurations(), 
  final_cost);
best_MJO = ploy_traj_opt_->getMinJerkOpt();//取出轨迹
```

可以看到，轨迹优化完全同质：计算内点，计算首尾状态，传入优化函数，取到本地。由于已经有初始轨迹，因此内点就以初始轨迹为准插点，和MANUAL一样。特别的，这里直接调用了优化函数`optimizeTrajectory()`，同时还多了一个未曾涉及的变量代价`final_cost`。

接下来看多拓扑的情况下多了哪些代码：
```c++
std::vector<ConstraintPoints> trajs = ploy_traj_opt_->distinctiveTrajs(segments);
Eigen::VectorXi success = Eigen::VectorXi::Zero(trajs.size());
for (int i = trajs.size() - 1; i >= 0; i--)
```

trajs是一个轨迹数组，因此多拓扑优化处理多条轨迹，而遍历也依此进行：

```c++
double final_cost, min_cost = 999999.0;
...//for
ploy_traj_opt_->setConstraintPoints(trajs[i]);
ploy_traj_opt_->setUseMultitopologyTrajs(true);
if (ploy_traj_opt_->optimizeTrajectory(headState, tailState,
       innerPts, initTraj.getDurations(), final_cost)){
	success[i] = true;
	if (final_cost < min_cost){
        min_cost = final_cost;
        best_MJO = ploy_traj_opt_->getMinJerkOpt();
        flag_success = true;
	}
```

trajs并非真正的轨迹，而是多组轨迹约束点的集合，分别进行优化，并得到代价最小的轨迹作为`best_MJO`。整体的逻辑也相当简单。

- **展示**

```c++
  if (flag_success){
      static int count_success = 0;
      count_success++;
      setLocalTrajFromOpt(best_MJO, touch_goal);
      cstr_pts = best_MJO.getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
      visualization_->displayOptimalList(cstr_pts, 0);
      continous_failures_count_ = 0;
  }
  else{
  cstr_pts = ploy_traj_opt_->getMinJerkOpt().getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
  visualization_->displayFailedList(cstr_pts, 0);
  continous_failures_count_++;
  }
```


展示部分完成的两个主要功能：为失败和成功次数计数，用于之前提到的minco轨迹初始化；其次是调用`visualization_->displayFailedList(cstr_pts, 0);`，用约束点画图。

因此我们可以将管理器和轨迹优化部分的连接总结如下：在MANUAL模式下，通过`globalMJO`的方法生成曲线，再用`traj_.setGlobalTraj`导入轨迹；而PRESET模式则是利用`initMJO`初始化和生成曲线，用`optimizeTrajectory`优化轨迹，通过`getMinJerkOpt()`得到最优轨迹，再于函数`EGOPlannerManager::setLocalTrajFromOpt`中提取轨迹写入`traj_.setLocalTraj`。


### 服务器（traj_server）

轨迹服务器在面所有部分中隐形，这是因为它是一个仅PRESET模式下的后端处理，将轨迹内容转化为命令。作为一个单独的节点，其有着独立的主函数，轨迹和时间信息的回调函数和一个定时器和发布器就占据了全部内容。而轨迹本身已经有了位置、速度、加速度、jerk信息，使得定时器、发布器、回调函数的内容除了传递外只有yaw角计算。可以说轨迹服务器只是一个无情的发布机器。

```c++
void polyTrajCallback(traj_utils::PolyTrajPtr msg){
  if (msg->order != 5)//5次多项式
    ROS_ERROR("[traj_server] Only support trajectory order equals 5 now!"),return;
  if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size())//每段描述式均有6项系数（5次），否则报错
	ROS_ERROR("[traj_server] WRONG trajectory parameters, "),return;

  int piece_nums = msg->duration.size();
  std::vector<double> dura(piece_nums);//段长数组初始化大小
  std::vector<poly_traj::CoefficientMat> cMats(piece_nums);//多项式系数数组初始化大小
  for (int i = 0; i < piece_nums; ++i){//依次写入cMats和dura
    int i6 = i * 6;
    cMats[i].row(0) << msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
        msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
    ...
    dura[i] = msg->duration[i];
  }
  traj_.reset(new poly_traj::Trajectory(dura, cMats));//轨迹
  start_time_ = msg->start_time;
  traj_duration_ = traj_->getTotalDuration();
  traj_id_ = msg->traj_id;
  receive_traj_ = true;
}
```

在此出现了一个巨大的疑惑：轨迹早已在管理器内生成并调用优化，为什么又出现了遍历数据并重新生成`traj_`？回顾轨迹信息的数据类型历程：

- 管理器内：`ploy_traj_opt_`作为优化器类，调用优化函数得到`poly_traj::MinJerkOpt`，`poly_traj::MinJerkOpt` -> `traj_.setLocalTraj()`；

- 状态机内：唯一上述未提的函数`polyTraj2ROSMsg`执行`traj_.local_traj()` -> `traj_utils::PolyTraj`，后者是ROS消息类型，由此才可以发布和接收；

- 服务器内：接受消息，并`traj_utils::PolyTrajPtr` -> `traj_.reset()`

可以看到，多项式生成，检查后传入轨迹优化器，在全局变量`traj_`中得到轨迹是正常的路线，只是此处为了消息收发做了变换，而每一次变换都是拆开多项式系数和段间时间，再重新生成新数据格式的过程。

## 优化（traj_opt）

ego最核心，同时也是最庞大、最艰深的部分，ego标志性的无距离场优化与V2特别加入的minco轨迹类型都在此处。类定义和数学功能都被外置在.h和.hpp文件中，只在.cpp文件中保留了功能函数，每个文件都具有庞大的量。

先分析.cpp文件的调用结构：

- 轨迹优化**`optimizeTrajectory`**中调用`computePintsToCheck`，后者调用finelyCheckAndSetConstraintPoints；而`costFunctionCallback`、`earlyExitCallback`、`RealT2VirtualT`；
- 代价函数回调`costFunctionCallback`调用了`roughlyCheckConstraintPoints`、`allowRebound`、VirtualT2RealT、`initAndGetSmoothnessGradCost2PT`、`VirtualTGradCost`、`addPVAJGradCost2CT`共此文件6个函数，还有`jerkOpt_`的两个函数`generate`和`getGrad2TP`.
- `addPVAJGradCost2CT`又调用了`obstacleGradCostP`、`swarmGradCostP`、`feasibilityGradCostV`、`feasibilityGradCostA`、`feasibilityGradCostJ`、`distanceSqrVarianceWithGradCost2p`、~~lengthVarianceWithGradCost2p~~（此函数弃用故不计入）共6个本文件函数。
  - 阅读论文后就可以知道，这部分对应

- 其余的set函数都是辅助函数，都是在文件外用`poly_traj_opt_->set...`来调用。
  - 所以，整个优化器是以优化函数`optimizeTrajectory`为主干的树形结构。

以上大多是通过`poly_traj_opt_->`调用的函数，由于通过类指针维护，更多对实例操作都是使用`initMJO`、`globalMJO`、`traj_`的指针引用类内函数，而类定义都聚集在.h文件中，以命名空间分类。

- 多项式轨迹优化器**`ego_planner::PolyTrajOptimizer`**：.cpp文件内所有函数都属于此类，**`ploy_traj_opt_`**就是它的类指针调用.cpp内的函数，最小急动度优化器`poly_traj::MinJerkOpt`、约束点`ego_planner::TrajContainer`也在类中；


- 最小急动度优化器**`poly_traj::MinJerkOpt`**：globalMJO、initMJO、bestMJO、JerkOpt_都是此类指针，由于处于`ego_planner::PolyTrajOptimizer`内，也可以通过`ploy_traj_opt_`调出；
- 轨迹**`poly_traj::Trajectory`**：轨迹数据结构，不管是容器还是规划器，轨迹在其中的最终数据形态；类中包含了数据和一系列get函数用于调用轨迹的参数，因为轨迹的本质是点集，许多间接参数需要计算；

  - 轨迹在最底层是**段数组**`vector<Piece>`，其单元**段**`poly_traj::piece`同样包含了数据和一系列get函数，其数据正是已经反复出现的5次多项式的6个系数的**系数阵**`typedef Eigen::Matrix<double, 3, 6> CoefficientMat`以及每段的**时间**`double duration`。
  - 这也是为什么轨迹类型转化时只遍历每段的时间（dura）和系数（cMat）。

- 轨迹容器**`ego_planner::TrajContainer`**：位于优化工具traj_utils文件夹的plan_Containers.hpp内，内含同文件内的GlobalTrajData、LocalTrajData、~~SwarmTrajData~~结构体与结构体的构造函数`set...()`；

  - 结构体内除了主要的`poly_traj::Trajectory`外，还有时间信息、轨迹索引等附属信息。


回顾管理器一节，所有轨迹都经过了minco优化，然后喂入`traj_`内，区分只在于MANUAL模式需要全局规划，而PRESET只需要局部规划。至此整个优化的过程得以明确：最初无论何种模式，都是从一系列的三维点开始；而不管是优化的结果，还是在轨迹容器中的实际形态轨迹最终都储存为`poly_traj::Trajectory`。

对此整个优化过程，不妨从以往的自上而下换为自下而上，先明确每个优化器的功能和输入输出。

### MINCO优化器（MinJerkOpt）

**MINCO**，即最小控制，特指由微分平坦特性得到的以z隐形表达为基础的jerk最小优化。程序中minco优化都是通过最小急动度优化器**`poly_traj::MinJerkOpt`**实现的，优化器类的结构如下：

- 成员变量和赋值运算重载：minco的成员变量分为输入、中间两部分。输入即前文导入优化器时用的段内点数N，首尾状态headPVA、tailPVA（PVA分别指位置、速度、加速度）；中间变量是状态方程的A, b两个矩阵，T1-4点间时间的各次数组，梯度gdC（因为ego_planner就是基于梯度的轨迹规划器）。

  而运算符重载是为了完成深拷贝，代码具有典型性：
```c++
public:
	inline void operator=(const MinJerkOpt &mjo){//指针赋值时把成员变量等都赋值
		N = mjo.N;
		...
```

- 私有函数：5个加入代价函数`add...()`和1个解优化函数`solveAdjGradC()`，明显是依次传入代价项信息，最后再调用解优化开始计算。
  - `solveAdjGradC()`直接调用A的类 `BandedSystem`执行类内优化。
- 公有函数：最重要的reset和generate，此外都是各种信息的获取函数`get...()`，值得注意的是梯度信息分为了初始化和计算两个函数。

可以从generate函数直观地看到minco的求解过程：

#### 问题生成

整个优化问题从微分平坦特性和二次优化问题开始：
$$
由微分平坦特性，可将状态、控制写为隐变量的s次多项式z，其中s有限。特别的，旋翼无人机z可写为\\
z=[p_x(t),p_y(t),p_z(t),\varphi(t)]^T=z(t),z(t)每一行为关于时间的多项式，z即轨迹\\
在此基础上，对经典的带初末约束的二次问题：\min_{z^*} J=\int_{t_0}^{t_f} z^{(s)}Wz^{(s)},有定理1:最优解z^*次数为2s-1\\
\\(省略数学证明)
$$

minco是jerk（s=3）最小优化，目标解即4个5次多项式的24个系数;

$$
当上述问题拓展到分段轨迹\{[t_0,t_1],[t_1,t_2],...,[t_{M-1},t_M]\}时，有定理2:\\
该2s-1次解最优的充分且唯一性条件为：\\
满足中间和首尾约束，且z^*(t)在t_i时刻在2s-1-d_i上连续且可微\\
\\(省略数学证明)
$$
$d_i$指中间约束的次数，即$z,\dot z,...,z^{(d_i-1)}$共$d_i$项在$t_i$时刻的具体值$\bar z_i$，$i$指对应时刻。最常见的$t_i$时刻的路点约束即$d_i=1,z(t_i)=waypoint$，对应最优轨迹$z^*(t)于t_i处在4次导数上连续可微$。
$$
根据定理2，可以利用求解连续性得到每段最优解,即系数矩阵c_i。对每一个约束时刻t_i，最优条件可以表述为：\\
\begin{cases}
z_{i}(t_i)=\bar z_i &(中间约束)\\
z_{i}(t_i)-z_{i+1}(0)=0 &(上段末值等于下段初值，连续性)
\end{cases}\\
转换为矩阵形式，即为：\\
\begin{cases}
\beta(t_i)=[1,t_i,t_i^2,...,t_i^{2s-1}]^T_{2s×1} &多项式向量，使用相对时间，t_i=T_i-T_{i-1}\\
{E_i^{d_i}}=[\beta(t_i),\dot \beta(t_i),...,\beta^{(d_i-1)}(t_i)]^T_{d_i×2s}&多项式阵，包括时间值和求导的系数，指代终值\\
{D_i}=[z(t_i),\dot z(t_i),...,z^{(d_i-1)}(t_i)]_{d_i×1}&中间约束\\
{F_i}=[-\beta(0),-\dot \beta(0),...,-\beta^{(2s-1-d_i)}(0)]^T_{(2s-di)×2s}&多项式阵，同上，指代初值
\end{cases}\\
\Rightarrow\begin{cases}
E_ic_i=D_i \\
E_ic_i-F_ic_{i+1}=0
\end{cases}\Rightarrow
\begin{bmatrix}E_i^{d_i}&0\\E_i^{2s-d_i}&F_i\end{bmatrix}
\begin{bmatrix}c_i\\c_{i+1}\end{bmatrix}=
\begin{bmatrix}D_i\\0\end{bmatrix}
\Rightarrow 
\begin{bmatrix}M_0\\M_i\\ \vdots \\M_M\end{bmatrix}
\begin{bmatrix}\{c_0,c_1\}^T\\\{c_1,c_2\}^T\\ \vdots \\\{c_{M-1},c_M\}^T\end{bmatrix}=
\begin{bmatrix}\{D_0,0\}^T\\\{D_1,0\}^T\\ \vdots \\\{D_M,0\}^T\end{bmatrix}
\Rightarrow Mc=b
\\(省略初末状态处理，即M_0和M_M)
\\(省略单边特殊解，省略无约束初始化)
$$

至此，整个优化问题被转化成了求解线性方程组$A(t_0,t_1,...t_M)x=b(hPVA,tPVA)$，可以便捷地利用程序实现。特别的，由于A阵是由一个个下三角矩阵按列组成，可以利用带状矩阵的特性存储和求解。
$$
T_N=\begin{bmatrix}t_1^N,t_2^N,...,t_{pointsNum-1}^N\end{bmatrix},N=1,2,3,4\\
A_i=[L(T_N)],b=[hPVA,tPVA]\\
以上三者是优化器类的成员变量，利用传入的信息生成后即可开始求解。生成过程如下\\
A = \left[\begin{array}{cccccc|cccccc}
1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 2 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\ \hline
\vdots & \vdots & \vdots & \ddots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots \\ \hline
1 & T_1(i) & T_2(i) & T_3(i) & T_4(i) & T_5(i) & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 1 & 2T_1(i) & 3T_2(i) & 4T_3(i) & 5T_4(i) & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 2 & 6T_1(i) & 12T_2(i) & 20T_3(i) & 0 & 0 & 0 & 0 & 0 & 0 \\ \hline
\vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots \\ \hline
1 & T_1(N-1) & T_2(N-1) & T_3(N-1) & T_4(N-1) & T_5(N-1) & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 1 & 2T_1(N-1) & 3T_2(N-1) & 4T_3(N-1) & 5T_4(N-1) & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 2 & 6T_1(N-1) & 12T_2(N-1) & 20T_3(N-1) & 0 & 0 & 0 & 0 & 0 & 0 \\
\end{array}\right]
\\
b = \left[\begin{array}{c}
\text{headPVA.col(0)} \\
\text{headPVA.col(1)} \\
\text{headPVA.col(2)} \\
\vdots \\
\text{inPs.col(i)} \\
\vdots \\
\text{tailPVA.col(0)} \\
\text{tailPVA.col(1)} \\
\text{tailPVA.col(2)}
\end{array}\right]
$$

解x就是一个<6,3>的数组，分别描述三轴轨迹的5次多项式系数。在程序中调用`A.solve(b)`解算并将答案存储在b内，通过`b.block<6,3>`获取，基于此实现各种get函数，例如通过时间采样来得到约束点，求导即可得到速度、加速度、急动度的描述。

作为分段多项式描述，MINCO无疑和B样条同质；但其能经过控制点、参数无耦合的特性使其比B样条更加易控。

#### 轨迹形变

在最开头就已经介绍过ego轨迹优化的核心步骤，即minco轨迹$\mathscr{T}$（即其直接的$c$）向Astar生成的无碰撞轨迹靠拢。靠拢的方向，是通过二次可微的代价函数$\mathcal{K}$最小的优化问题提供的，即要实现效果如下：
$$
J=\min_{\mathscr{T}}\mathcal{K}(c(q,T),T)\\
约束：M(T)c(q,T)=b(q)\\
其中，q指路点坐标信息，T指路点时间信息
$$
因此，本质上是调整q和T，将代价函数做等效变换，可以改写为$J=\min_{q,t}\mathcal{W}(q,T)$至此唯一的问题在于如何计算$\frac{\partial \mathcal{W}}{\partial q},\frac{\partial \mathcal{W}}{\partial T}$。易得：
$$
\frac{\partial \mathcal{W}}{\partial q}=
\Tr(\frac{\partial c}{\partial q}^T\frac{\partial \mathcal{K}}{\partial c})=
\Tr(M^{-T}\frac{\partial b}{\partial q}^T\frac{\partial \mathcal{K}}{\partial c})\\
特别的，M是带状矩阵，可以对其做PLU分解求解梯度，而对T微分同理。
$$

### LBFGS（lbfgs）

LBFGS指受限的（limted）BFGS算法，缩写是四个提出者名字的首字母，没有实际含义。具体来说，ego使用的数值计算方法是以快启动的Barzilai-Borwein算法（BB算法）生成初始解开始，通过LBFGS迭代而成，二者都是拟牛顿法。

牛顿法迭代是利用线性拟合来逼近数值解，然而对于二次优化问题，线性拟合往往会造成反复横跳，甚至难以收敛，因此往往使用二次拟合：

| | 一次      | 二次      |
| --------------- | ----------------------------------------- | --------------------------------------------------------- |
| $Tylor$         | $f(x)=f(x_i)+\nabla f(x_i)(x_i-x_{i+1})]$ | $f(x)=f(x_i)+\nabla f(x_i)(x_i-x_{i+1})+H(x_i-x_{i+1})^2$ |
| $\nabla f(x)=0$ | $x(i+1)=x(i)-\nabla f\cdot f$             | $x(i+1)=x(i)-H^{-1}\cdot\nabla f$         |

Hessian矩阵难以求解，且无法保证可逆，因此考虑构造一个接近H阵但可逆的矩阵代替计算，即为拟牛顿法。具体的来说，BB和BFGS使用的拟合矩阵如下：

| BB           | BFGS         | L-BFGS       |
| ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
| $s_k=\{\alpha_k^1 = \frac{\mathbf{s}_{k-1}^T\mathbf{s}_{k-1}}{\mathbf{s}_{k-1}^T\mathbf{y}_{k-1}} , \alpha_k^2 = \frac{\mathbf{s}_{k-1}^T\mathbf{y}_{k-1}}{\mathbf{y}_{k-1}^T\mathbf{y}_{k-1}} \}$ | $H_{k+1} = \left(I - \rho_k y_k s_k^T\right) H_k \left(I - \rho_k s_k y_k^T\right) + \rho_k s_k s_k^T$<br />$\rho_k=\frac{1}{y_k^Ts_k}$ | $p_0=-\nabla f(x_k)$<br />$p_{i+1} = p_i - \frac{s_i^T p_i}{y_i^T s_i} y_i $<br />${p}' = \gamma {p}_m$<br />$p' = p' + (\frac{s_i^T p_i}{y_i^T s_i}-\frac{y_i^T p'}{y_i^T s_i}) s_i $ |

其中，$s_k=x_k-x_{k-1},y_k=\nabla f(x_k)-\nabla f(x_{k-1})$，而$\gamma$一般设为$ \frac{y_0^T s_0}{y_0^T y_0}$。BB算法没有使用近似H的算法，而是直接估计出步长快速收敛；L-BFGS相较于BFGS实现了无显性H计算步长，内存利用率更高。

L-BFGS是由头文件lbhgs.h实现的，并没有使用类封装，而是基于指针参数传递实现数据传输。文件主要分为三个部分：实际程序计算部分，向量运算、类型、函数定义，检错机制。

- 程序的检错机制非常庞大，几乎所有的分支都来自检错，同时程序提供了数十个检错标识符，比实际参与计算的参数都多，因此在此不表。
- 程序中定义了向量的加减乘除等各项函数作为运算符；同时，各个计算步骤的返回值也用结构体封装；此外，其他运算中使用的函数也在此定义。
- LBFGS在`inline int lbfgs_optimize`实现，需要传入:
```c++
int result = lbfgs::lbfgs_optimize(
          variable_num_,//变量数，即维数
          x_init,//初始解(double*)x
          &final_cost,//代价存储处的指针(double*)，计算出的实时代价存储其中
          PolyTrajOptimizer::costFunctionCallback,//代价函数回调，传入代价计算方法
          NULL,//最大步长策略，无则选用默认
          PolyTrajOptimizer::earlyExitCallback,//早停回调
          this,//泛型指针，和外部链接，此处自然是optimizeTrajectory()函数
          &lbfgs_params/*基础参数，含内存大小等*/);
```

`lbfgs_optimize`函数也是由初始化，运算和检错三部分组成，不过由于不处于ROS系统，因此没有涉及时间控制。特别的，lbfgs以控制内存为长，所有使用的内存空间都利用向量内存初始化函数`vecalloc()`的`malloc`初始化，并最后用`vecfree()`的`free`释放。忽略了初始化、检错和内存操作后，函数的架构得以明晰：

```c++
vecncpy(d, g, n);//d就是搜索方向γ（\gamma）,g对应梯度\nabla f
vec2norm(&xnorm, x, n), vec2norm(&gnorm, g, n);//每次标准化都有检查步骤，全部省略（即<1.0则置1.0）
else{
    vec2norminv(&step, d, n);
    k = 1,end = 0,loop = 1;
    while (loop == 1){
        veccpy(xp, x, n), veccpy(gp, g, n);
        step_min = param.min_step, step_max = param.max_step;//由于没有用最大步长策略，忽略有关代码
        ls = line_search_morethuente(n, x, &fx, g, d, &step, xp, gp, &step_min, &step_max, &cd, &param);
        if (ls < 0){/* 退出条件：Revert to the previous point. */
            veccpy(x, xp, n), veccpy(g, gp, n);
            ret = ls;
            loop = 0,continue;
        }
        vec2norm(&xnorm, x, n),vec2norm(&gnorm, g, n);
        it = &lm[end];
        vecdiff(it->s, x, xp, n);//s_{k+1} = x_{k+1} - x_{k} = step * d_{k}.
        vecdiff(it->y, g, gp, n);//y_{k+1} = g_{k+1} - g_{k}
        vecdot(&ys, it->y, it->s, n);//y^Ts 
        vecdot(&yy, it->y, it->y, n);//y^Ty 
        it->ys = ys;
        vecncpy(d, g, n);
        if (ys > DBL_EPSILON){/* Skip L-BFGS update when ys is too small as proposed in Ceres Solver */
            bound = (m <= k) ? m : k, ++k;
            end = (end + 1) % m, j = end;
            for (i = 0; i < bound; ++i){
                j = (j + m - 1) % m; /* if (--j == -1) j = m-1; */
                it = &lm[j];
                vecdot(&it->alpha, it->s, d, n);//\alpha_{j} = \rho_{j} s^{t}_{j} \cdot q_{k+1}
                it->alpha /= it->ys;
                vecadd(d, it->y, -it->alpha, n);//q_{i} = q_{i+1} - \alpha_{i} y_{i}
            }
            vecscale(d, ys / yy, n);
            for (i = 0; i < bound; ++i){
                it = &lm[j];
                vecdot(&beta, it->y, d, n);//\beta_{j} = \rho_{j} y^t_{j} \cdot \gamm_{i}
                beta /= it->ys;
                vecadd(d, it->s, it->alpha - beta, n);//\gamm_{i+1}=\gamm_{i}+(\alpha_{j}-\beta_{j})s_{j}
                j = (j + 1) % m; //if (++j == m) j = 0;
            }
        }
        step = 1.0;/*Now the search direction d is ready. We try step = 1 first.*/
    }
return ret;
```

LBFGS结构十分简单，关键围绕在两个判断上，即循环退出与更新跳过。

其中一个重要函数：line_search_morethuente也需要明确。



### GCOPTER（poly_traj_utils）

GCOPTER，即几何约束下的轨迹优化，是minco提出论文的主角，却是egoV2中的次要部分。egoV2将gcopter.hpp移植到poly_traj_utils.hpp内，成为了反复提到的命名空间`poly_traj`，因此可以将GCOPTER当做废代码。

### EGO（optimizeTrajectory）

`optimizeTrajectory()`的结构在注释上已有提示：三步准备和一个do-while循环：

- 准备指的是函数需要的标识符和时间初始化、轨迹相关参数导入，以及lbfgs参数确定；
- do内部就是运行lbfgs并检查其返回值是否是错误标识。

因此，核心就是LBFGS算法的执行：其优化的对象是什么，输出是什么，在何时调用？
```c++
int result = lbfgs::lbfgs_optimize(
          variable_num_,
          x_init,
          &final_cost,
          PolyTrajOptimizer::costFunctionCallback,
          NULL,
          PolyTrajOptimizer::earlyExitCallback,
          this,
          &lbfgs_params)
```

`costFunctionCallback`回调函数就是代价函数，对应了EGO论文中提到的光滑代价、碰撞代价和可行代价，具体是通过`PolyTrajOptimizer *opt`优化器类指针调用内部的函数实现：
```c++
opt->VirtualT2RealT(t, T); // Unbounded virtual time to real time
opt->jerkOpt_.generate(P, T); // Generate trajectory from {P,T}
opt->initAndGetSmoothnessGradCost2PT(gradT, smoo_cost); // Smoothness cost
opt->addPVAJGradCost2CT(gradT, obs_swarm_feas_qvar_costs, opt->cps_num_prePiece_); // Time int cost

if (opt->allowRebound())
	opt->roughlyCheckConstraintPoints(); // Trajectory rebound

opt->jerkOpt_.getGrad2TP(gradT, gradP); // Gradient prepagation
opt->VirtualTGradCost(T, t, gradT, gradt, time_cost); // Real time back to virtual time
opt->iter_num_ += 1;

return smoo_cost + obs_swarm_feas_qvar_costs.sum() + time_cost;
```

在LBFGS中，可以看到该函数的输入为`fx = cd.proc_evaluate(cd.instance, x, g, cd.n);`，即通过变量和minco得到的梯度结果计算代价。

`addPVAJGradCost2CT`，包揽了碰撞代价，集群代价（不表）、可行代价，`initAndGetSmoothnessGradCost2PT`则通过调用minco获取梯度基础的平滑代价，`VirtualTGradCost`提供时间代价，将真实时间非线性映射到虚拟时间中以作惩罚；这三类都是类内函数。

其中`allowRebound()`和`roughlyCheckConstraintPoints()`对应着EGO标志性的“反弹”优化。`allowRebound()`使用三项判据作为反弹轨迹的条件：优化迭代至少三次，轨迹夹角大于30°，多拓扑轨迹初始化完成避障；
执行反弹避障的函数就是`bool PolyTrajOptimizer::roughlyCheckConstraintPoints(void)`，同时整个Astar输出

```c++
int in_id, out_id;
vector<std::pair<int, int>> segment_ids;
bool flag_new_obs_valid = false;
int i_end = ConstraintPoints::two_thirds_id(cps_.points, touch_goal_);
for (int i = 1; i <= i_end; ++i){
	bool occ = grid_map_->getInflateOccupancy(cps_.points.col(i));//遍历2/3的约束点是否在障碍内
	if (occ){
		...//利用向量再检查是否在障碍内
		flag_new_obs_valid = true;
		int j;
		...//数据检错，同时得到障碍内点的开始和结束索引in、out，注意此二点都不在障碍内，便于Astar
		i = j + 1;
		segment_ids.push_back(std::pair<int, int>(in_id, out_id));//队列加入一组障碍
	}
}
```
此部分即生成初始minco，并查找所有的障碍段，障碍完全有可能不止一个，因此需要队列；同时，障碍段的开始和结束索引in、out实际在障碍物表面，而Astar的“笨拙”搜索方式能最大化得到沿障碍物表面的路劲：
```c++
if (flag_new_obs_valid){//确定有约束点在障碍内后启动
	vector<vector<Eigen::Vector3d>> a_star_pathes;
	for (size_t i = 0; i < segment_ids.size(); ++i){//对每组障碍
        Eigen::Vector3d in(cps_.points.col(segment_ids[i].second)), 
                        out(cps_.points.col(segment_ids[i].first));
        ASTAR_RET ret = a_star_->AstarSearch(grid_map_->getResolution(), in, out);//对障碍内点做避障Astar
        if (ret == ASTAR_RET::SUCCESS)
            a_star_pathes.push_back(a_star_->getPath());
        else...//Asatr失败情况处理
	}
    for (size_t i = 1; i < segment_ids.size(); i++){//避免障碍段互有重叠，注意i从1开始，遍历总数-1
        if (segment_ids[i - 1].second >= segment_ids[i].first){//后一段开头比前一段末尾还先，则重排二者值
            double middle = (double)(segment_ids[i - 1].second + segment_ids[i].first) / 2.0;
            segment_ids[i - 1].second = static_cast<int>(middle - 0.1);
            segment_ids[i].first = static_cast<int>(middle + 1.1);//平均后一加一减
        }
    }
```
检查障碍段是否重合是非常重要的步骤，因为Astar只断续地对障碍段搜索，无法保证搜索得到的段间可行，因此需要外部控制障碍段不重叠。如果没有测验，是不容易想到这一点的，可以作为ego中诸多被此文忽略的检错机制的缩影。
接下来就是寻找控制点法平面与障碍物表面的交点：
```c++
    for (size_t i = 0; i < segment_ids.size(); ++i){//遍历每一段障碍
    // step 1
        for (int j = segment_ids[i].first; j <= segment_ids[i].second; ++j)
        	cps_.flag_temp[j] = false;//全置F
    // step 2
        int got_intersection_id = -1;
        for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j){//遍历每段障内点
            Eigen::Vector3d ctrl_pts_law(cps_.points.col(j + 1) - cps_.points.col(j - 1)),
            //用以制造法平面的向量，前后点连接向量作为此点切线拟合
                            intersection_point;
            int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; //最优是最远点，但计算量大
            double val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law), 
            //无碰撞轨迹的中点点和约束点对应得到初始反弹向量，与切线向量点积得初始val
                   init_val = val;
            while (true){//遍历无碰撞轨迹，根据与切线点积调整方向，直到向量抵达到法平面
                last_Astar_id = Astar_id;
                if (val >= 0){
                    ++Astar_id;
                    if (Astar_id >= (int)a_star_pathes[i].size())//越界，停
                        break;
                }
                else{
                    --Astar_id;
                    if (Astar_id < 0)
                        break;
                }
                val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law);
                if (val * init_val <= 0 && (abs(val) > 0 || abs(init_val) > 0)) {
                    //搜索跨越正负且至少一个非零，则可认为到法平面，即可确定交点
                    intersection_point = a_star_pathes[i][Astar_id] + 
                        (
                            (a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                            (
                                ctrl_pts_law.dot(cps_.points.col(j) - a_star_pathes[i][Astar_id]) / 
                                ctrl_pts_law.dot
                                (
                                    a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]
                                )
                            ) // = t，即线性插值，
                        );
                    got_intersection_id = j;//最终结果
                    break;
                }
            }
            if (got_intersection_id >= 0){
                double length = (intersection_point - cps_.points.col(j)).norm();
                if (length > 1e-5){
                    cps_.flag_temp[j] = true;
                    for (double a = length; a >= 0.0; a -= grid_map_->getResolution()){
                        bool occ = grid_map_->getInflateOccupancy(
                            (a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));
                        if (occ || a < grid_map_->getResolution()){
                            if (occ)
                                a += grid_map_->getResolution();
                            cps_.base_point[j].push_back(
                                (a / length) * intersection_point + 
                                (1 - a / length) * cps_.points.col(j));
                            cps_.direction[j].push_back(
                                (intersection_point - cps_.points.col(j)).normalized());
                            break;
                        }
                    }
                }
                else
                    got_intersection_id = -1;
            }
        }
    //step 3
        if (got_intersection_id >= 0){
            for (int j = got_intersection_id + 1; j <= segment_ids[i].second; ++j)
                if (!cps_.flag_temp[j]){
                    cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
                    cps_.direction[j].push_back(cps_.direction[j - 1].back());
                }
            for (int j = got_intersection_id - 1; j >= segment_ids[i].first; --j)
                if (!cps_.flag_temp[j]){
                    cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
                    cps_.direction[j].push_back(cps_.direction[j + 1].back());
                }
        }
        else
            ROS_WARN_COND(VERBOSE_OUTPUT, "Failed to generate direction. It doesn't matter.");
    }
    force_stop_type_ = STOP_FOR_REBOUND;
    return true;
}
return false;
```





## 可视化（vis）

