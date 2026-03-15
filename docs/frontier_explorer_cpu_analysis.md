# Frontier Explorer CPU 占用分析

分析对象：`scripts/frontier_explorer.py`

本文总结三件事：

- 当前自主探索算法在做什么
- 为什么这个实现的 CPU 占用高
- 应该按什么顺序降低 CPU 占用

本文基于当前代码做静态分析，不是基于 `cProfile` 或运行时火焰图。

---

## 1. 算法在整个流程里做了什么

当前这个节点不是一个“找到 frontier 然后发一个导航点”这么简单的探索器。
它在一个 Python 节点里做了四层工作：

1. 把 `/map` 的 `OccupancyGrid` 转成 NumPy 栅格图。
2. 以机器人当前位置为起点，在自由空间里做 BFS，找到并聚类 frontier。
3. 计算一套带清障偏好的 GVD 表示：
   - 障碍物连通域标记
   - 距离变换
   - GVD 骨架提取
4. 再做一次偏向 GVD 的 A*，生成 waypoint，最后把 waypoint 发给 Nav2。

所以整个系统实际上在同时做：

- 自定义 frontier 搜索
- 自定义 GVD 路径搜索
- Nav2 全局规划
- Nav2 局部控制

这套设计的优点是路径更倾向于走走廊中间，缺点是比标准 frontier exploration 重很多。

---

## 2. 当前代码的主执行流程

`scripts/frontier_explorer.py` 的主链路如下：

1. 定时器触发 `_explore_tick()`
2. `_explore_tick()` 调用 `_make_plan()`
3. `_make_plan()` 内部依次执行：
   - 读取 TF，得到机器人在 `map` 下的位置
   - 检查是否有进展
   - 把地图转成 `np.array`
   - 调用 `_search_from()` 搜索 frontier
   - 调用 `_get_distance_transform()` 计算 GVD 相关缓存
   - 发布 frontier / GVD / path 可视化
   - 调用 `_find_gvd_path()` 做自定义 A*
   - 发送 `NavigateThroughPoses` 或 `NavigateToPose`

有一个关键行为需要单独说明：

- 当 `self.navigating == True` 时，节点在进度检查后会提前返回，不继续做重规划。

这意味着：

- 正常稳定导航时，这个节点本身不应该持续满负载
- 如果你看到 CPU 很高，通常说明它在频繁重规划、频繁失败重试，或者可视化本身就很重

---

## 3. CPU 占用高的核心原因

### 3.1 一次重规划里有多次“整张地图级”的 Python 遍历

当前实现中，几个最重的函数都是在 Python 里逐栅格处理：

- `_search_from()`：遍历可达自由空间并检查邻居
- `_build_frontier()`：对每个 frontier cluster 继续做 BFS
- `_compute_distance_and_labels()`：整图扫描障碍物并做连通域标记
- `_compute_distance_and_labels()`：再次对整图做 BFS 距离传播
- `_find_gvd_path()`：在自由空间图上做 A*

这才是 CPU 高的根本原因。

如果地图大小为 `N = width * height`，那么一次重规划可能会触发多次 `O(N)` 或接近 `O(N)` 的全图运算，而且大部分是在 Python 循环里完成，不是 C/C++ 后端。

### 3.2 GVD 缓存失效条件太敏感

`_get_distance_transform()` 当前是按 `/map` 的 `header.stamp` 判断缓存是否失效。

这会带来一个直接问题：

- 只要 SLAM 发布了一个新地图消息
- 即使地图只改了一小块
- 也会触发整套距离变换和 GVD 重算

当前参数又把这个问题放大了：

- `config/frontier_explorer_params.yaml`
  - `planner_frequency: 0.5`
- `config/slam_params.yaml`
  - `map_update_interval: 2.0`
  - `resolution: 0.05`

`0.5 Hz` 表示探索器每 `2 秒` 规划一次，而 SLAM 也是每 `2 秒` 更新一次地图。
这两个周期几乎完全重合，结果就是：基本每轮规划都会重算一次 GVD。

### 3.3 自定义 A* 仍然是在“大自由空间图”上跑

虽然名字叫 GVD path，但它不是在一张很小的“骨架图”上搜索。
当前实现是：

- 在整张 free-space grid 上做 A*
- 只是对不在 GVD 上的栅格增加罚分

这意味着搜索空间依然很大。
即使有 GVD 偏好，A* 仍然可能扩展大量自由栅格。

### 3.4 可视化本身也很耗 CPU

当前探索参数里默认开启：

```yaml
visualize: true
visualize_gvd: true
```

节点会持续发布：

- frontier marker
- GVD 骨架 marker
- path marker

其中 `_publish_gvd_markers()` 最重，因为它要：

- 找出所有 GVD 栅格
- 把坐标放进 Python `set`
- 创建大量 `geometry_msgs/Point`
- 组装大尺寸 `MarkerArray`

这不仅增加本节点 CPU，也会增加 RViz 端的渲染开销。

### 3.5 `Frontier` 保存了不少当前决策并不需要的数据

`Frontier` 结构目前保存：

- `points`
- `middle`
- `initial`

但真正参与决策的主要是：

- `size`
- `min_distance`
- `centroid`

也就是说，每个 frontier cluster 都在额外维护一大堆点坐标，但这些数据对最终 frontier 选择帮助有限，反而带来了 Python list 扩容和内存抖动。

### 3.6 这个节点和 Nav2 存在“重复规划”

当前流程是：

1. 这个节点先自己算出一条偏向 GVD 的路径
2. 再把 waypoint 发给 Nav2
3. Nav2 再做一次自己的路径规划和控制

所以系统实际上为同一个导航任务支付了两次规划成本。
如果 CPU 是刚性约束，这就是架构层面上最不经济的一点。

---

## 4. 当前最重的几个热点

从代码结构上看，最值得优先关注的热点是：

1. `_compute_distance_and_labels()`
2. `_search_from()`
3. `_find_gvd_path()`
4. `_publish_gvd_markers()`

这里最关键的判断不是“某一个函数写得不够优雅”，而是：

- 这个节点重算整图结构的频率太高
- 而且这些整图结构是在 Python 中计算的

---

## 5. 降低 CPU 占用的方案

### 5.1 先做参数级优化

这类改动风险最小，收益通常也最快。

#### A. 先关掉可视化

在 `config/frontier_explorer_params.yaml` 里改成：

```yaml
visualize: false
visualize_gvd: false
```

原因：

- 直接去掉大量 marker 构造开销
- 直接减轻 RViz 负载
- 这是最低风险的一步

#### B. 降低规划频率

建议先改成：

```yaml
planner_frequency: 0.2
```

如果还想再省一点，可以改成：

```yaml
planner_frequency: 0.1
```

原因：

- frontier exploration 一般不需要像局部控制那样高频
- 当前算法足够重，2 秒一次通常仍然偏高

代价：

- 新 frontier 出现后的响应会更慢

#### C. 缩小 GVD snap 搜索半径

建议改成：

```yaml
gvd_snap_radius: 1.0
```

原因：

- `_snap_to_gvd()` 的局部 BFS 搜索范围会变小
- nearest-GVD 搜索成本也会下降

代价：

- 某些场景下可能更容易找不到合适的 GVD 点

#### D. 如果允许，降低地图更新频率

在 `config/slam_params.yaml` 里考虑改成：

```yaml
map_update_interval: 4.0
```

或：

```yaml
map_update_interval: 5.0
```

原因：

- 更少的地图更新时间戳变化
- 更少的 GVD 缓存失效
- 更少的整图重算

代价：

- 地图刷新没有现在这么及时

#### E. 如果允许，调粗地图分辨率

在 `config/slam_params.yaml` 里考虑改成：

```yaml
resolution: 0.1
```

原因：

- 栅格边长翻倍后，整图单元数大约下降到四分之一
- 所有整图算法都会明显变快

代价：

- 地图更粗
- frontier 边界更不精细

---

### 5.2 再做低风险代码优化

这类改动不改变总体算法结构，但能去掉明显的无效开销。

#### A. 不再保存每个 frontier 的全部点

建议把 `Frontier` 精简到只保留：

- `size`
- `min_distance`
- `centroid`

并在 BFS 时做增量式质心统计。

原因：

- 减少 Python list 分配和扩容
- 降低内存抖动

#### B. 在 map callback 里缓存 NumPy 地图

当前是每次 `_make_plan()` 都把 `OccupancyGrid` 转成 `np.array`。
更合理的做法是：

- 在 `_map_callback()` 中转换一次
- 后续规划直接复用缓存数组

原因：

- 少一次整图转换
- 缓存策略更清晰

#### C. 不要只按 `header.stamp` 失效 GVD 缓存

更合理的方案包括：

- 最多每 `N` 秒重算一次 GVD
- 只有地图内容变化明显时才重算
- 在 `_map_callback()` 里维护一个更粗粒度的 dirty flag

原因：

- 当前缓存失效策略过于敏感

#### D. 只在 GVD 真变化时发布 GVD markers

不要每轮规划都重发 GVD 可视化。
应该改成：

- 只有在重建 GVD 后才发布

原因：

- 减少重复构造大量 `Point` 和 `Marker`
- 降低 RViz 和本节点的 CPU 开销

---

### 5.3 真正降得最多的结构性优化

如果目标是明显压低 CPU，而不是优先保留“走廊中线效果”，下面这些改动收益最大。

#### A. 直接移除自定义 GVD A*

保留：

- frontier 搜索
- frontier 排序

去掉：

- 自定义 free-space A*
- 多 waypoint 的 GVD 路径构造

然后直接发：

- `NavigateToPose(best_frontier)`

或者：

- `NavigateToPose(snapped_frontier)`

这是收益最大的简化方式，因为它直接删掉了一层重型 planner。

代价：

- 路径不一定像现在这样尽量贴近走廊中线
- 更依赖 Nav2 自己的 costmap 和 planner 调参

#### B. 如果必须保留 GVD，就不要在整张自由空间图上跑 A*

更合理的做法是：

- 先提取真正的 GVD 图结构
- 再只在 GVD 图上搜索

原因：

- 骨架图比整张 free-space grid 小得多

代价：

- 实现复杂度更高

#### C. 把整图计算移出纯 Python

适合迁移的部分包括：

- obstacle connected components
- distance transform
- label propagation
- skeleton extraction

可选实现方式：

- C++ 节点
- 带 C 后端的库，例如 `scipy.ndimage`

原因：

- 当前瓶颈本质上是大量 Python 层逐栅格循环

---

## 6. 推荐优化顺序

如果目标是“先快速降 CPU，且尽量不改架构”，建议按这个顺序来：

1. 关闭 `visualize` 和 `visualize_gvd`
2. 把 `planner_frequency` 降到 `0.2`
3. 缩小 `gvd_snap_radius`
4. 如果允许，增大 `map_update_interval`
5. 去掉 `Frontier.points` 这类当前决策不依赖的数据
6. 把 GVD 缓存失效逻辑从 `header.stamp` 改成更粗粒度策略
7. 如果 CPU 还是高，直接删掉自定义 GVD A*，只保留 frontier 选点

如果目标是“CPU 降得最多”，最有效的一步其实是：

- 移除自定义 GVD A* 这一层

---

## 7. 结论

当前这个自主探索节点 CPU 占用高，不是因为某一个小函数写得不好，而是因为它的总体结构决定了它很重：

- frontier BFS
- obstacle labeling
- distance transform
- GVD extraction
- GVD-biased A*
- marker generation

这些工作里有很多都是整张地图级别的运算，而且是在 Python 中完成的。

因此，真正有效的优化方向只有三个：

- 降低整图重算频率
- 关闭或减少高成本可视化
- 减少与 Nav2 的重复规划

如果你更看重探索质量和走廊中线效果，就保留 GVD 思路，重点优化缓存和可视化。
如果你更看重 CPU 占用，就应该简化结构，让 Nav2 承担更多路径规划工作。
