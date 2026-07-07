# LiDAR 模拟器 — 开发文档

> **版本**: v1.0
> **创建日期**: 2026-07-07
> **状态**: 开发中

---

## 目录

1. [项目概述](#1-项目概述)
2. [项目结构](#2-项目结构)
3. [坐标系定义](#3-坐标系定义)
4. [模块详细说明](#4-模块详细说明)
   - [4.1 angle_judge — 视场角判断](#41-angle_judge--视场角判断)
   - [4.2 distance_judge — 距离判断](#42-distance_judge--距离判断)
   - [4.3 occlusion_judge — 遮挡判断](#43-occlusion_judge--遮挡判断)
   - [4.4 judge — 综合判断](#44-judge--综合判断)
   - [4.5 point_make_and_map — 点云生成与地图构建](#45-point_make_and_map--点云生成与地图构建)
   - [4.6 main — 主函数](#46-main--主函数)
5. [接口规范（API）](#5-接口规范api)
6. [数据结构](#6-数据结构)
7. [参数配置表](#7-参数配置表)
8. [依赖环境](#8-依赖环境)
9. [开发任务拆分](#9-开发任务拆分)
10. [测试方案](#10-测试方案)
11. [注意事项与常见坑](#11-注意事项与常见坑)
12. [参考资料](#12-参考资料)

---

## 1. 项目概述

### 1.1 目标

开发一个 LiDAR 点云模拟器，模拟车载 LiDAR 对场景中锥桶（交通锥）的扫描过程，生成带有噪声的点云数据。

### 1.2 核心流程

```
┌──────────────┐
│  输入场景数据  │  ← 车辆位置、朝向、锥桶位置列表
└──────┬───────┘
       ▼
┌──────────────┐
│  遍历每个锥桶  │
└──────┬───────┘
       ▼
┌──────────────────────────────────┐
│          judge() 综合判断         │
│  ┌────────────┐ ┌─────────────┐  │
│  │ 角度判断    │ │ 距离判断     │  │
│  │ (120° FOV) │ │ (1.5m~50m)  │  │
│  └────────────┘ └─────────────┘  │
│  ┌────────────┐ ┌─────────────┐  │
│  │ 遮挡判断    │ │ 前方判断     │  │
│  └────────────┘ └─────────────┘  │
└──────────────┬───────────────────┘
               ▼ (通过全部判断)
┌──────────────────────────────────┐
│   point_make_and_map()           │
│   → 生成 12-16 个模拟反射点      │
│   → 添加高斯噪声(σ=0.02m)        │
│   → 加入全局点云地图             │
└──────────────┬───────────────────┘
               ▼
┌──────────────────────────────────┐
│   生成 200-500 个随机地面点       │
│   (z ≈ -1.0m, 供 RANSAC 使用)    │
└──────────────┬───────────────────┘
               ▼
┌──────────────┐
│ 输出点云地图  │
└──────────────┘
```

---

## 2. 项目结构

```
lidar_sim/
├── DEVELOPMENT.md          ← 本文档（开发文档）
├── Lidar_simulation.md     ← 原始需求描述
├── lidar_simulator.py      ← 主程序（所有模块集中在此）
├── config.py               ← 参数配置文件（待创建）
├── tests/                  ← 单元测试目录（待创建）
│   ├── test_angle.py
│   ├── test_distance.py
│   ├── test_point_gen.py
│   └── test_judge.py
└── output/                 ← 输出点云文件目录（待创建）
```

---

## 3. 坐标系定义

### 3.1 世界坐标系（右手系）

```
              z (上)
              ↑
              │
              │
              │
              └──────────→ x (车辆前进方向参考轴)
             ╱
            ╱
           ╱
          y (车辆左侧)
```

| 轴 | 方向 | 说明 |
|----|------|------|
| **x** | 正东 / 车辆初始前进方向 | 水平基准轴 |
| **y** | 正北 / 车辆左侧 | 水平方向 |
| **z** | 向上（天顶） | 垂直方向 |

### 3.2 车辆坐标系

```
              车辆前方
                 ↑ x_v
                 │
                 │
                 │
       ──────────┼──────────
                 │
                 │
                 ↓
              y_v (车辆左侧)
```

- 原点：LiDAR 传感器中心
- x_v：车辆前进方向（与 `car_heading` 一致）
- y_v：车辆左侧

### 3.3 航向角定义

- **0 rad**：朝向 x 轴正方向
- **正值**：从 x 轴逆时针旋转（偏左）
- **单位**：弧度（rad）

```
                     90° (π/2)
                       ↑
                       │
                       │
        180°(π) ───────┼───────→ 0° (0) / 360° (2π)
                       │
                       │
                       ↓
                    -90° (-π/2)
```

---

## 4. 模块详细说明

### 4.1 `angle_judge` — 视场角判断

#### 功能
判断锥桶是否在以车辆前进方向为中心、±60° 的扇形视场范围内。

#### 算法原理

```
                    ╱ 120° FOV
                   ╱
        ─────────●─────────  ← 车辆前进方向 (car_heading)
                 │╲
                 │ ╲ 60°
                 │  ╲
                 │   ╲
```

#### 数学推导

```
Step 1: 向量计算
    dx = cone_position[0] - car_position[0]
    dy = cone_position[1] - car_position[1]

Step 2: 方位角（使用 arctan2 区分象限）
    angle_to_cone = arctan2(dy, dx)     ∈ (-π, π]

Step 3: 转到车辆坐标系
    relative_angle = angle_to_cone - car_heading

Step 4: 归一化到 [-π, π]
    relative_angle = (relative_angle + π) mod 2π - π

Step 5: 判断
    |relative_angle| ≤ π/3 (60°)  →  在视场内
```

#### 当前代码状态

```python
def angle_judge(car_position, car_heading, cone_position):
    # 角度判断
    dx = cone_position[0] - car_position[0]
    dy = cone_position[1] - car_position[1]

    angle_to_cone = np.arctan2(dy, dx)

    relative_angle = angle_to_cone - car_heading

    relative_angle = (relative_angle + np.pi) % (2 * np.pi) - np.pi

    half_fov =   # ⚠️ 待补充：应为 np.radians(60)
```

#### 待完成部分

| 项目 | 说明 |
|------|------|
| `half_fov` | 需赋值为 `np.radians(60)` 或 `np.pi / 3` |
| 返回值 | 需返回 `bool`：`abs(relative_angle) <= half_fov` |
| 参数校验 | 建议添加输入维度检查 |

---

### 4.2 `distance_judge` — 距离判断

#### 功能
判断锥桶是否在 LiDAR 的有效测距范围内（1.5m ~ 50m）。

#### 数学原理

```
    distance = √(dx² + dy² + dz²)

    有效范围:  1.5m ≤ distance ≤ 50m
```

```
    ←─── 无效 ───→│←────── 有效范围 ──────→│←─── 无效 ───→
                   │                        │
    ───────────────┼────────────────────────┼──────────────
                 1.5m                     50m
                最小量程                  最大量程
```

#### 当前代码状态

```python
def distance(car_position, point):
    if()    # ⚠️ 待实现
```

#### 待完成部分

| 项目 | 说明 |
|------|------|
| 距离计算 | 使用 `np.linalg.norm(cone_position - car_position)` |
| 范围判断 | `return 1.5 <= dist <= 50.0` |

#### 参考实现

```python
def distance_judge(car_position, cone_position, 
                   min_dist=1.5, max_dist=50.0):
    """判断锥桶是否在有效距离内"""
    dist = np.linalg.norm(cone_position - car_position)
    return min_dist <= dist <= max_dist
```

---

### 4.3 `occlusion_judge` — 遮挡判断

#### 功能
判断从 LiDAR 到锥桶的视线（LOS, Line of Sight）是否被其他物体遮挡。

#### 算法选择

| 方案 | 复杂度 | 适用场景 | 推荐 |
|------|--------|----------|------|
| 线段-AABB 相交 | O(n) | 锥桶作为离散遮挡物 | ⭐ 推荐 |
| 深度缓冲 (Z-Buffer) | O(像素) | 有深度图 | 适用于后期 |
| 无遮挡（跳过） | O(1) | 简单场景/V1 | 仅用于临时调试 |

#### 锥桶遮挡模型

遮挡物不再简化为球体，而是根据锥桶规格表建立竖直轴对齐包围盒（AABB）。锥桶位置表示底面中心，尺寸单位为米。

| 锥桶名称 | 颜色/特征 | 反光条/条纹 | 尺寸 |
|---------|----------|-------------|------|
| 大号橘色桩桶 | 橘色 | 双反光条 | 0.35 × 0.35 × 0.70 |
| 小号红色桩桶 | 红色 | 单反光条 | 0.20 × 0.20 × 0.30 |
| 小号黄色桩桶 | 黄色 | 黑色条纹 | 0.20 × 0.20 × 0.30 |
| 小号蓝色桩桶 | 蓝色 | 单反光条 | 0.20 × 0.20 × 0.30 |

#### 射线投射原理

```
    LiDAR ●━━━━━━━━━━━━━━━━━━━━━━● 锥桶
              ↑
              │ 检查射线是否与任何障碍物相交
              │
         ┌────┴────┐
         │ 锥桶AABB │  ← 若线段与包围盒相交，则目标锥桶被遮挡
         └─────────┘
```

#### 接口定义

```python
def occlusion_judge(car_position, cone_position, obstacles):
    """
    判断锥桶是否被遮挡

    Parameters:
        car_position  : np.array([x, y, z])  LiDAR位置
        cone_position : np.array([x, y, z])  锥桶位置
        obstacles     : list[dict]            障碍物列表（锥桶位置 + 颜色/类型/尺寸）

    Returns:
        bool: True = 被遮挡, False = 未被遮挡
    """
```

#### 障碍物输入格式

```python
obstacles = [
    {'position': np.array([2.5, 0.0, 0.0]), 'color': 'yellow'},
    {'position': np.array([4.0, 0.2, 0.0]), 'type': 'large_orange'},
    {'position': np.array([6.0, -0.1, 0.0]), 'size': [0.20, 0.20, 0.30]},
]
```

---

### 4.4 `judge` — 综合判断

#### 功能
对单个锥桶执行全部四个前置条件判断，决定其是否应被 LiDAR 扫描到。

#### 判断条件一览

| 序号 | 条件 | 判断函数 | 阈值 |
|------|------|----------|------|
| ① | 是否在车前方 | `front()` | x_v > 0 |
| ② | 是否在 120° 视场内 | `angle_judge()` | ±60° |
| ③ | 是否在有效距离内 | `distance_judge()` | 1.5m ~ 50m |
| ④ | 是否无遮挡 | `occlusion_judge()` | 无交点 |

#### 流程图

```
              锥桶
               │
               ▼
        ┌──────────────┐
        │ front()?     │──── No ──→ return False
        │ 在车前方？    │
        └──────┬───────┘
               │ Yes
               ▼
        ┌──────────────┐
        │ angle_judge()?│──── No ──→ return False
        │ 在120°内？    │
        └──────┬───────┘
               │ Yes
               ▼
        ┌──────────────┐
        │distance_judge?│─── No ──→ return False
        │ 在距离内？    │
        └──────┬───────┘
               │ Yes
               ▼
        ┌──────────────┐
        │occlusion_    │
        │judge()?      │─── Yes ─→ return False（被遮挡）
        │有无遮挡？    │
        └──────┬───────┘
               │ No
               ▼
          return True
```

#### 接口定义

```python
def judge(car_position, car_heading, cone_position, obstacles=None):
    """
    综合判断锥桶是否在LiDAR可检测范围内

    Parameters:
        car_position  : np.array([x, y, z])  LiDAR/车辆位置
        car_heading   : float (rad)          车辆航向角
        cone_position : np.array([x, y, z])  锥桶位置
        obstacles     : list[dict], optional  障碍物列表

    Returns:
        bool: True = 可被扫描到, False = 不可检测
    """
```

---

### 4.5 `point_make_and_map` — 点云生成与地图构建

#### 功能
1. 为可见锥桶生成 12-16 个模拟表面反射点
2. 添加高斯噪声（σ = 0.02m）
3. 将点加入全局地图
4. 生成 200-500 个随机地面点（z ≈ -1.0m）

#### 锥桶模型

```
         /\\
        /  \\        ← 锥桶外形
        \\  /
         \\/
         
    点云采样使用锥台近似，尺寸来自锥桶规格表:
    - 大号橘色桩桶: 0.35 × 0.35 × 0.70m
    - 小号红/黄/蓝桩桶: 0.20 × 0.20 × 0.30m
    - 底面半径: max(width, depth) / 2
    - 顶面半径: bottom_radius * 0.3

    z (高度)
    ↑
    H ━━━  ← 顶部
    │  ╱╲
    │ ╱  ╲
    │╱    ╲
    0 ━━━━━  ← 底部 (半径 0.1m)
```

#### 点云生成算法

```
    对于每个可见锥桶:
        1. 按锥桶类型读取尺寸 [width, depth, H]
        2. 随机选取 12-16 个高度层（在 0 ~ H 之间均匀采样）
        3. 对每个高度层:
            a. 计算该高度的锥桶半径（线性插值）
            b. 随机生成一个角度 θ ∈ [0, 2π)
            c. 计算点的位置:
               r = bottom_radius * (1 - h/H) + top_radius * (h/H)
               x = cone_x + r * cos(θ)
               y = cone_y + r * sin(θ)
               z = cone_z + h
            d. 添加高斯噪声:
               x += N(0, σ=0.02)
               y += N(0, σ=0.02)
               z += N(0, σ=0.02)
        3. 将生成的点加入全局点云列表
```

#### 高斯噪声

```
    概率密度:
    
    p(x) = 1/(σ√(2π)) * e^(-(x-μ)²/(2σ²))
    
    其中: μ = 0, σ = 0.02m
    
         │    ██
         │   ████
         │  ██████
         │ ████████
         │██████████
    ─────┼──────────────→ 偏移量
        -0.04  0  0.04
    
    68% 的点落在 ±0.02m 以内
    95% 的点落在 ±0.04m 以内
```

#### NumPy 噪声生成

```python
# 生成 1 个噪声值（标量）
noise = np.random.normal(loc=0.0, scale=0.02)

# 生成 N 个噪声值（向量）
noise_array = np.random.normal(loc=0.0, scale=0.02, size=N)

# 生成 shape=(N, 3) 的三维噪声（N 个点，每点 xyz 各一个噪声）
noise_xyz = np.random.normal(loc=0.0, scale=0.02, size=(N, 3))
```

#### 地面点生成

```python
# 在 LiDAR 下方 z ≈ -1.0m 处生成随机地面点
# 用于 RANSAC 地面分割算法的输入

n_ground_points = np.random.randint(200, 501)  # 200~500 个
x_range = np.random.uniform(-30, 30, n_ground_points)
y_range = np.random.uniform(-30, 30, n_ground_points)
z_ground = np.random.normal(loc=-1.0, scale=0.05, size=n_ground_points)

ground_points = np.column_stack([x_range, y_range, z_ground])
```

#### 接口定义

```python
def point_make_and_map(cone_position, lidar_height=1.0, global_map=None):
    """
    为可见锥桶生成模拟反射点并加入地图

    Parameters:
        cone_position : np.array([x, y, z])  锥桶位置
        lidar_height  : float                LiDAR离地高度（默认1.0m）
        global_map    : list / np.array      全局点云阵列（引用传递）

    Returns:
        newly_generated_points: np.array(shape=(N, 3))  本次生成的锥桶点
        ground_points: np.array(shape=(M, 3))            本次生成的地面点
    """
```

---

### 4.6 `front` — 前方判断

#### 功能
判断锥桶是否在车辆前进方向的前半部分（初步筛选）。

#### 原理

```
             前方 (x_v > 0)
           ╱          ╲
          ╱            ╲
         ╱   可检测区    ╲
        ╱                ╲
       ╱                  ╲
      ●────────────────────●
     LiDAR
       ╲                  ╱
        ╲                ╱
         ╲   后方       ╱
          ╲ (x_v < 0) ╱
           ╲          ╱
             不可检测
```

#### 实现

```python
def front(car_position, car_heading, cone_position):
    """判断锥桶是否在车辆前方"""
    # 计算向量
    dx = cone_position[0] - car_position[0]
    dy = cone_position[1] - car_position[1]
    
    # 投影到车辆前进方向
    x_forward = dx * np.cos(car_heading) + dy * np.sin(car_heading)
    
    return x_forward > 0
```

---

### 4.7 `main` — 主函数

#### 功能
程序入口，协调各模块的执行流程。

#### 流程

```python
def main():
    # 1. 获取车辆状态
    car_position, car_heading = get_car_state()    # 从外部读取或硬编码
    
    # 2. 获取场景数据
    cone_positions = load_cones()                    # 锥桶位置列表
    obstacles = load_obstacles() if has_obstacles else None
    
    # 3. 初始化全局点云地图
    global_map = np.empty((0, 3))
    
    # 4. 遍历所有锥桶
    for i, cone in enumerate(cone_positions):
        # 4.1 初步筛选：是否在车前方
        if not front(car_position, car_heading, cone):
            continue
        
        # 4.2 综合判断
        if judge(car_position, car_heading, cone, obstacles):
            # 4.3 生成点云并加入地图
            cone_points, ground_points = point_make_and_map(
                cone, lidar_height=1.0
            )
            global_map = np.vstack([global_map, cone_points, ground_points])
    
    # 5. 输出结果
    save_point_cloud(global_map, "output/point_cloud.npy")
    print(f"生成点云总数: {len(global_map)}")
```

---

## 5. 接口规范（API）

### 5.1 函数签名总览

```python
# 基础判断函数
def front(car_position          : np.ndarray,  # shape (3,)
          car_heading           : float,       # 弧度
          cone_position         : np.ndarray   # shape (3,)
          )                     -> bool:
    ...

def angle_judge(car_position    : np.ndarray,  # shape (3,)
                car_heading     : float,       # 弧度
                cone_position   : np.ndarray,  # shape (3,)
                fov_deg         : float        # 默认 120
                )               -> bool:
    ...

def distance_judge(car_position : np.ndarray,
                   cone_position: np.ndarray,
                   min_dist     : float,       # 默认 1.5
                   max_dist     : float        # 默认 50.0
                   )            -> bool:
    ...

def occlusion_judge(car_position : np.ndarray,
                    cone_position: np.ndarray,
                    obstacles    : list[dict]
                    )            -> bool:
    ...

# 综合判断
def judge(car_position          : np.ndarray,
          car_heading           : float,
          cone_position         : np.ndarray,
          obstacles             : list or None
          )                     -> bool:
    ...

# 点云生成
def point_make_and_map(cone_position : np.ndarray,
                       lidar_height  : float,       # 默认 1.0
                       global_map    : np.ndarray or None
                       )             -> tuple[np.ndarray, np.ndarray]:
    ...

# 主函数
def main() -> None:
    ...
```

### 5.2 返回值约定

| 函数 | 返回类型 | 说明 |
|------|----------|------|
| `front()` | `bool` | `True` = 在车前方 |
| `angle_judge()` | `bool` | `True` = 在视场内 |
| `distance_judge()` | `bool` | `True` = 在距离内 |
| `occlusion_judge()` | `bool` | `True` = 被遮挡 |
| `judge()` | `bool` | `True` = 可通过全部判断 |
| `point_make_and_map()` | `tuple(ndarray, ndarray)` | (锥桶点云, 地面点云) |

---

## 6. 数据结构

### 6.1 点云数据

```python
# 单帧点云: shape (N, 3)
# N = 所有锥桶生成的点 + 随机地面点
point_cloud = np.array([
    [x1, y1, z1],    # 点1
    [x2, y2, z2],    # 点2
    ...
    [xN, yN, zN],    # 点N
], dtype=np.float64)

# 字段说明:
#   x: 水平位置 (m)
#   y: 水平位置 (m)
#   z: 垂直高度 (m)
```

### 6.2 锥桶数据

```python
# 锥桶列表: list of np.array([x, y, z])
cones = [
    np.array([5.0, 2.0, 0.0]),     # 锥桶1: 底面中心坐标
    np.array([10.0, -1.0, 0.0]),   # 锥桶2
    np.array([3.0, 6.0, 0.0]),     # 锥桶3
]

# 锥桶规格表，尺寸为 [宽, 深, 高]，单位 m
cone_specs = {
    'large_orange': [0.35, 0.35, 0.70],  # 大号橘色桩桶，发车线/收车线
    'small_red'   : [0.20, 0.20, 0.30],  # 小号红色桩桶，起点/终点区域
    'small_yellow': [0.20, 0.20, 0.30],  # 小号黄色桩桶，赛道右侧边界
    'small_blue'  : [0.20, 0.20, 0.30],  # 小号蓝色桩桶，赛道左侧边界
}
```

### 6.3 车辆状态

```python
# 车辆/传感器状态
car_state = {
    'position'    : np.array([0.0, 0.0, 0.0]),  # LiDAR 位置 (m)
    'heading'     : 0.0,                          # 航向角 (rad)
    'lidar_height': 1.0,                          # LiDAR 离地高度 (m)
}
```

### 6.4 障碍物数据

```python
# 障碍物列表（锥桶按规格表转换为竖直 AABB）
obstacles = [
    {'position': np.array([2.0, 1.0, 0.0]), 'color': 'yellow'},
    {'position': np.array([5.0, 3.0, 0.0]), 'type': 'large_orange'},
    {'position': np.array([6.0, 2.0, 0.0]), 'size': [0.20, 0.20, 0.30]},
]
```

---

## 7. 参数配置表

> 建议将所有可调参数集中在 `config.py` 中管理。

| 参数名 | 符号 | 默认值 | 单位 | 说明 |
|--------|------|--------|------|------|
| `FOV_DEG` | θ_fov | 120 | ° | 水平视场角 |
| `FOV_HALF` | θ_half | 60 | ° | 视场半角 |
| `MIN_DIST` | d_min | 1.5 | m | 最小有效距离 |
| `MAX_DIST` | d_max | 50.0 | m | 最大有效距离 |
| `LARGE_ORANGE_SIZE` | — | 0.35×0.35×0.70 | m | 大号橘色桩桶尺寸 |
| `SMALL_CONE_SIZE` | — | 0.20×0.20×0.30 | m | 小号红/黄/蓝桩桶尺寸 |
| `POINTS_PER_CONE` | n_pt | 12~16 | 个 | 每锥桶采样点数 |
| `NOISE_STD` | σ | 0.02 | m | 高斯噪声标准差 |
| `GROUND_Z` | z_gnd | -1.0 | m | 地面点高度（相对 LiDAR） |
| `GROUND_Z_STD` | σ_gnd | 0.05 | m | 地面点高度噪声 |
| `N_GROUND_MIN` | — | 200 | 个 | 随机地面点最小数量 |
| `N_GROUND_MAX` | — | 500 | 个 | 随机地面点最大数量 |
| `GROUND_XY_RANGE` | — | ±30 | m | 地面点水平范围 |
| `LIDAR_HEIGHT` | h_lidar | 1.0 | m | LiDAR 离地高度 |

---

## 8. 依赖环境

### 8.1 Python 版本

- **Python**: ≥ 3.8

### 8.2 依赖包

```
numpy>=1.20.0
```

### 8.3 安装命令

```bash
pip install numpy
```

### 8.4 可选依赖（后续扩展）

```bash
pip install matplotlib       # 可视化点云
pip install open3d           # 点云处理与显示
```

---

## 9. 开发任务拆分

### 9.1 任务分配表

| 模块 | 负责人 | 优先级 | 状态 | 说明 |
|------|--------|--------|------|------|
| `config.py` | 待分配 | P0 | ⬜ 待开发 | 参数配置文件 |
| `front()` | 待分配 | P0 | ⬜ 待开发 | 前方判断（简单，用于入门） |
| `angle_judge()` | 待分配 | P0 | 🟡 部分完成 | 框架已有，需补充返回值 |
| `distance_judge()` | 待分配 | P0 | ⬜ 待开发 | 距离判断 |
| `occlusion_judge()` | 待分配 | P1 | ⬜ 待开发 | 遮挡判断（可延后） |
| `judge()` | 待分配 | P0 | ⬜ 待开发 | 综合判断（组合上述模块） |
| `point_make_and_map()` | 待分配 | P0 | ⬜ 待开发 | 点云生成核心 |
| `main()` | 待分配 | P0 | 🟡 部分完成 | 框架已有，需完善逻辑 |
| 单元测试 | 待分配 | P1 | ⬜ 待开发 | 测试脚本 |
| 可视化脚本 | 待分配 | P2 | ⬜ 待开发 | matplotlib 三维显示 |

### 9.2 开发顺序建议

```
Phase 1 (MVP — 尽快交付):
    config.py → front() → angle_judge() → distance_judge()
    → judge() → point_make_and_map() → main()

Phase 2 (功能完善):
    occlusion_judge() → 单元测试

Phase 3 (工具完善):
    可视化脚本 → 数据导入/导出
```

---

## 10. 测试方案

### 10.1 单元测试

#### test_angle.py — 角度判断测试

```python
import numpy as np
from lidar_simulator import angle_judge

def test_cone_directly_in_front():
    """锥桶在正前方 → 应在视场内"""
    car_pos = np.array([0.0, 0.0, 0.0])
    heading = 0.0
    cone = np.array([5.0, 0.0, 0.0])
    assert angle_judge(car_pos, heading, cone) == True

def test_cone_at_45_deg():
    """锥桶在左前方 45° → 应在视场内"""
    car_pos = np.array([0.0, 0.0, 0.0])
    heading = 0.0
    cone = np.array([3.0, 3.0, 0.0])  # 45°
    assert angle_judge(car_pos, heading, cone) == True

def test_cone_at_exactly_60_deg():
    """锥桶恰好在 60° 边界 → 应在视场内（≤）"""
    car_pos = np.array([0.0, 0.0, 0.0])
    heading = 0.0
    # tan(60°) = √3 ≈ 1.732
    cone = np.array([1.0, np.sqrt(3), 0.0])
    assert angle_judge(car_pos, heading, cone) == True

def test_cone_at_90_deg():
    """锥桶在正左方 90° → 应不在视场内"""
    car_pos = np.array([0.0, 0.0, 0.0])
    heading = 0.0
    cone = np.array([0.0, 5.0, 0.0])
    assert angle_judge(car_pos, heading, cone) == False

def test_cone_behind():
    """锥桶在正后方 → 应不在视场内"""
    car_pos = np.array([0.0, 0.0, 0.0])
    heading = 0.0
    cone = np.array([-5.0, 0.0, 0.0])
    assert angle_judge(car_pos, heading, cone) == False

def test_heading_rotation():
    """车辆旋转 90° 后，原正左方变为正前方"""
    car_pos = np.array([0.0, 0.0, 0.0])
    heading = np.pi / 2  # 朝向 y 轴正方向
    cone = np.array([0.0, 5.0, 0.0])  # 在 y 轴正方向
    assert angle_judge(car_pos, heading, cone) == True

def test_angle_wrap_around():
    """测试角度跨越 ±π 边界的情况"""
    car_pos = np.array([0.0, 0.0, 0.0])
    heading = np.deg2rad(170)  # 朝向接近正左偏后
    # 锥桶在 -170° 方向（即车辆右前方 20°）
    cone = np.array([np.cos(np.deg2rad(-170)), np.sin(np.deg2rad(-170)), 0.0]) * 5
    assert angle_judge(car_pos, heading, cone) == True
```

#### test_distance.py — 距离判断测试

```python
import numpy as np
from lidar_simulator import distance_judge

def test_within_range():
    """距离 10m → 应在范围内"""
    car_pos = np.array([0.0, 0.0, 0.0])
    cone = np.array([10.0, 0.0, 0.0])
    assert distance_judge(car_pos, cone) == True

def test_too_close():
    """距离 1.0m → 应不在范围内"""
    car_pos = np.array([0.0, 0.0, 0.0])
    cone = np.array([1.0, 0.0, 0.0])
    assert distance_judge(car_pos, cone) == False

def test_too_far():
    """距离 60m → 应不在范围内"""
    car_pos = np.array([0.0, 0.0, 0.0])
    cone = np.array([60.0, 0.0, 0.0])
    assert distance_judge(car_pos, cone) == False

def test_boundary_min():
    """恰好在 1.5m → 应在范围内（≤）"""
    car_pos = np.array([0.0, 0.0, 0.0])
    cone = np.array([1.5, 0.0, 0.0])
    assert distance_judge(car_pos, cone) == True

def test_boundary_max():
    """恰好在 50m → 应在范围内（≤）"""
    car_pos = np.array([0.0, 0.0, 0.0])
    cone = np.array([50.0, 0.0, 0.0])
    assert distance_judge(car_pos, cone) == True
```

#### test_point_gen.py — 点云生成测试

```python
import numpy as np
from lidar_simulator import point_make_and_map

def test_point_count():
    """生成的点数应在 12~16 之间"""
    cone = np.array([5.0, 0.0, 0.0])
    points, _ = point_make_and_map(cone)
    assert 12 <= len(points) <= 16

def test_point_shape():
    """每个点应有 3 个坐标 (x, y, z)"""
    cone = np.array([5.0, 0.0, 0.0])
    points, _ = point_make_and_map(cone)
    assert points.shape[1] == 3

def test_points_near_cone():
    """生成的点应在锥桶附近（距离 < 0.5m）"""
    cone = np.array([5.0, 0.0, 0.0])
    points, _ = point_make_and_map(cone)
    distances = np.linalg.norm(points - cone, axis=1)
    assert np.all(distances < 0.5)

def test_ground_points_count():
    """地面点数量应在 200~500 之间"""
    cone = np.array([5.0, 0.0, 0.0])
    _, ground = point_make_and_map(cone)
    assert 200 <= len(ground) <= 500

def test_ground_z_near_minus_one():
    """地面点 z 坐标应在 -1.0 附近"""
    cone = np.array([5.0, 0.0, 0.0])
    _, ground = point_make_and_map(cone)
    z_mean = np.mean(ground[:, 2])
    assert abs(z_mean - (-1.0)) < 0.1
```

### 10.2 集成测试

```python
# test_integration.py
import numpy as np
from lidar_simulator import judge, point_make_and_map

def test_full_pipeline():
    """端到端测试：完整流程"""
    car_pos = np.array([0.0, 0.0, 0.0])
    heading = 0.0
    
    # 场景：3 个锥桶
    cones = [
        np.array([5.0, 0.0, 0.0]),     # 正前方 → 可见
        np.array([0.0, 5.0, 0.0]),     # 正左方 → 不可见
        np.array([-5.0, 0.0, 0.0]),    # 正后方 → 不可见
    ]
    
    visible_count = 0
    for cone in cones:
        if judge(car_pos, heading, cone):
            visible_count += 1
    
    assert visible_count == 1  # 只有正前方的锥桶可见
```

---

## 11. 注意事项与常见坑

### 11.1 角度相关

| 坑 | 说明 | 解决方案 |
|----|------|----------|
| 角度/弧度混淆 | NumPy 三角函数使用弧度 | 始终用 `np.radians()` 转换 |
| `atan` vs `atan2` | `atan` 无法区分象限 | 必须用 `np.arctan2(dy, dx)` |
| 角度跨越 ±π | 170° 和 -170° 实际只差 20° | 归一化：`(x+π) % 2π - π` |
| 航向角方向 | 正方向定义不统一 | 本文档约定：逆时针为正 |

### 11.2 NumPy 相关

| 坑 | 说明 | 解决方案 |
|----|------|----------|
| 数组维度 | `np.array([1,2,3])` shape 是 `(3,)` 不是 `(3,1)` | 需要时用 `.reshape(-1, 1)` |
| 浅拷贝问题 | `a = b` 不复制数据 | 需要复制时用 `a = b.copy()` |
| `vstack` 空数组 | `np.vstack([np.empty((0,3)), points])` 要求维度匹配 | 确保空数组 shape 为 `(0, 3)` |
| 随机数种子 | 每次运行结果不同 | 调试时用 `np.random.seed(42)` 固定 |

### 11.3 逻辑相关

| 坑 | 说明 | 解决方案 |
|----|------|----------|
| 边界条件 | 恰好在 60° 或 50m 时是否算"在内" | 本文档约定：`≤` 算在内 |
| 浮点精度 | `0.1 + 0.2 != 0.3` | 比较时用 `np.isclose()` |
| 空列表处理 | 没有锥桶时程序是否崩溃 | 添加空列表检查 |

### 11.4 代码规范

```python
# ✅ 推荐
def angle_judge(car_position: np.ndarray, car_heading: float) -> bool:
    """判断锥桶是否在视场内"""
    dx = cone_position[0] - car_position[0]
    ...

# ❌ 不推荐
def angle_judge(a, b):  # 命名不清晰
    x = b[0] - a[0]     # 魔法数字
    ...
```

---

## 12. 参考资料

### 12.1 NumPy 文档

| 函数 | 文档链接 |
|------|----------|
| `np.arctan2` | https://numpy.org/doc/stable/reference/generated/numpy.arctan2.html |
| `np.linalg.norm` | https://numpy.org/doc/stable/reference/generated/numpy.linalg.norm.html |
| `np.random.normal` | https://numpy.org/doc/stable/reference/generated/numpy.random.normal.html |
| `np.vstack` | https://numpy.org/doc/stable/reference/generated/numpy.vstack.html |
| `np.column_stack` | https://numpy.org/doc/stable/reference/generated/numpy.column_stack.html |

### 12.2 相关概念

- **RANSAC 地面分割**: 随机采样一致性算法，用于从点云中提取平面
- **FOV (Field of View)**: 视场角，LiDAR 可探测的角度范围
- **高斯噪声**: 模拟传感器测量误差的标准模型

### 12.3 文件清单

| 文件 | 说明 |
|------|------|
| `DEVELOPMENT.md` | 本文档 |
| `Lidar_simulation.md` | 原始需求 |
| `lidar_simulator.py` | 主程序 |
| `config.py` | 参数配置（待创建） |
| `tests/` | 测试目录（待创建） |
| `output/` | 输出目录（待创建） |

---

## 附录 A：config.py 参考模板

```python
"""
config.py — LiDAR 模拟器参数配置
集中管理所有可调参数，便于团队协作与调参
"""

# ─── 视场参数 ───
FOV_DEG = 120           # 水平视场角 (°)
FOV_HALF = 60           # 视场半角 (°)

# ─── 距离参数 ───
MIN_DIST = 1.5          # 最小有效距离 (m)
MAX_DIST = 50.0         # 最大有效距离 (m)

# ─── 锥桶规格参数（宽, 深, 高，单位 m）───
LARGE_ORANGE_SIZE = (0.35, 0.35, 0.70)  # 大号橘色桩桶
SMALL_RED_SIZE = (0.20, 0.20, 0.30)     # 小号红色桩桶
SMALL_YELLOW_SIZE = (0.20, 0.20, 0.30)  # 小号黄色桩桶
SMALL_BLUE_SIZE = (0.20, 0.20, 0.30)    # 小号蓝色桩桶

# ─── 点云生成参数 ───
POINTS_PER_CONE_MIN = 12    # 每锥桶最少点数
POINTS_PER_CONE_MAX = 16    # 每锥桶最多点数
NOISE_STD = 0.02            # 高斯噪声标准差 (m)

# ─── 地面点参数 ───
GROUND_Z = -1.0         # 地面点高度，相对 LiDAR (m)
GROUND_Z_STD = 0.05     # 地面点高度噪声 (m)
N_GROUND_MIN = 200      # 地面点最小数量
N_GROUND_MAX = 500      # 地面点最大数量
GROUND_XY_RANGE = 30.0  # 地面点水平范围 (±m)

# ─── 传感器参数 ───
LIDAR_HEIGHT = 1.0      # LiDAR 离地高度 (m)

# ─── 随机种子（调试用，None 表示随机）───
RANDOM_SEED = None
```

---

## 附录 B：完整主程序参考实现

```python
"""
lidar_simulator.py — LiDAR 点云模拟器
模拟车载 LiDAR 对锥桶场景的扫描过程
"""

import numpy as np
from config import *


def front(car_position, car_heading, cone_position):
    """判断锥桶是否在车辆前方"""
    dx = cone_position[0] - car_position[0]
    dy = cone_position[1] - car_position[1]
    x_forward = dx * np.cos(car_heading) + dy * np.sin(car_heading)
    return x_forward > 0


def angle_judge(car_position, car_heading, cone_position, fov_deg=120):
    """判断锥桶是否在视场角内"""
    dx = cone_position[0] - car_position[0]
    dy = cone_position[1] - car_position[1]
    angle_to_cone = np.arctan2(dy, dx)
    relative_angle = angle_to_cone - car_heading
    relative_angle = (relative_angle + np.pi) % (2 * np.pi) - np.pi
    half_fov = np.radians(fov_deg / 2)
    return abs(relative_angle) <= half_fov


def distance_judge(car_position, cone_position, min_dist=1.5, max_dist=50.0):
    """判断锥桶是否在有效距离内"""
    dist = np.linalg.norm(cone_position - car_position)
    return min_dist <= dist <= max_dist


def occlusion_judge(car_position, cone_position, obstacles):
    """判断锥桶是否被遮挡：锥桶按规格表转换为竖直 AABB。"""
    if not obstacles:
        return False
    # 对每个障碍锥桶：
    # 1. 根据 color/type/size 得到 [宽, 深, 高]
    # 2. 以 position 为底面中心生成 AABB
    # 3. 用 slab test 判断 LiDAR→目标锥桶线段是否与 AABB 相交
    # 任一相交即返回 True
    ...


def judge(car_position, car_heading, cone_position, obstacles=None):
    """综合判断锥桶是否可被 LiDAR 扫描到"""
    if not front(car_position, car_heading, cone_position):
        return False
    if not angle_judge(car_position, car_heading, cone_position):
        return False
    if not distance_judge(car_position, cone_position):
        return False
    if obstacles and occlusion_judge(car_position, cone_position, obstacles):
        return False
    return True


def point_make_and_map(cone_position, lidar_height=1.0):
    """
    为可见锥桶生成模拟反射点
    返回: (锥桶点云, 地面点云)
    """
    # 锥桶参数
    bottom_r = 0.10
    top_r = 0.03
    height = 0.50

    # 随机确定采样点数
    n_points = np.random.randint(12, 17)

    # 在锥桶表面采样
    heights = np.random.uniform(0, height, n_points)
    angles = np.random.uniform(0, 2 * np.pi, n_points)

    # 线性插值计算每个高度的半径
    radii = bottom_r * (1 - heights / height) + top_r * (heights / height)

    # 计算点坐标
    x = cone_position[0] + radii * np.cos(angles)
    y = cone_position[1] + radii * np.sin(angles)
    z = cone_position[2] + heights

    cone_points = np.column_stack([x, y, z])

    # 添加高斯噪声
    noise = np.random.normal(0, 0.02, cone_points.shape)
    cone_points += noise

    # 生成地面点
    n_ground = np.random.randint(200, 501)
    gx = np.random.uniform(-30, 30, n_ground)
    gy = np.random.uniform(-30, 30, n_ground)
    gz = np.random.normal(-1.0, 0.05, n_ground)
    ground_points = np.column_stack([gx, gy, gz])

    return cone_points, ground_points


def main():
    """主函数"""
    # 设置随机种子（调试用）
    if RANDOM_SEED is not None:
        np.random.seed(RANDOM_SEED)

    # 车辆状态
    car_position = np.array([0.0, 0.0, 0.0])
    car_heading = 0.0

    # 场景数据
    cones = [
        np.array([5.0, 0.0, 0.0]),
        np.array([10.0, 3.0, 0.0]),
        np.array([3.0, -2.0, 0.0]),
        np.array([0.0, 8.0, 0.0]),     # 正左方，应不可见
        np.array([-5.0, 0.0, 0.0]),     # 正后方，应不可见
    ]

    # 初始化全局点云
    global_map = np.empty((0, 3))

    # 处理每个锥桶
    visible_count = 0
    for cone in cones:
        if judge(car_position, car_heading, cone):
            cone_pts, ground_pts = point_make_and_map(cone)
            global_map = np.vstack([global_map, cone_pts, ground_pts])
            visible_count += 1

    print(f"锥桶总数: {len(cones)}")
    print(f"可见锥桶: {visible_count}")
    print(f"点云总数: {len(global_map)}")

    # 保存结果
    np.save("output/point_cloud.npy", global_map)
    print("点云已保存至 output/point_cloud.npy")


if __name__ == "__main__":
    main()
```

---

> **文档维护**: 请在每次接口变更时同步更新本文档。
> **问题反馈**: 如有疑问请在项目群中讨论或提交 Issue。 
