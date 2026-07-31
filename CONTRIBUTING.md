# G1 Robot 展厅导览系统 — 贡献规范

本规范从现有代码提炼，并补充审查中发现的关键约束。所有新代码与改动须遵循以下约定。

---

## 1. Git / 提交规范

### 1.1 仓库卫生

- **禁止提交构建产物**：`build/`、`devel/` 已在 `.gitignore` 中忽略，切勿 `git add -f` 强制加入。
- **禁止提交运行时隐私数据**：人脸照片、支付码、会话文件、日志不得入库。
- **禁止提交 IDE 个人配置**：`.vscode/`、`.idea/` 下的路径均为开发者本机绝对路径，不入库。
- **依赖锁文件**：Python 依赖变更时同步更新 `requirements.txt`；C++ 依赖版本记录在 `package.xml` 的 `<depend>` 中。

### 1.2 提交信息

- 使用中文或英文均可，但**同一仓库内保持一致**。现有历史为中文，推荐继续使用中文。
- 格式：`<类型>: <简述>`，类型包括 `feat`（新功能）、`fix`（修复）、`refactor`（重构）、`docs`（文档）、`chore`（杂项）。
- 示例：`feat: 新增多点导航讲解脚本`、`fix: 修复 IMU 加速度预积分 bug`、`refactor: 7 个控制器合并为基类+子类`。
- 一个提交只做一件事；混合改动拆分为多个提交。

### 1.3 分支模型

- `main`：稳定可部署状态，建图/导航/运控均可在真机运行。
- `dev`：日常开发集成分支。
- 功能分支：`feat/<短描述>`、`fix/<短描述>`，合并后删除。
- PR 合并前须在工作空间全新 `catkin_make` 通过（无 ROS 环境时至少通过 Python `py_compile`）。

---

## 2. ROS 工程规范

### 2.1 包命名与结构

- 包名使用 `snake_case`，与 ROS REP 144 一致：`fastlio`、`xju_pnc`、`velocity_smoother_ema`。
- 每个包须包含：`package.xml`（license/maintainer **不得为 TODO**）、`CMakeLists.txt`、`src/`、`launch/`、`config/`。
- `package.xml` 的 `<license>` 须填实际许可证（本项目自研代码用 BSD-2-Clause）。
- 第三方 vendored 包（`livox_ros_driver2`、`cyclonedds`、`unitree_sdk2` 等）不修改其源码；如需补丁，在 README 记录补丁内容。

### 2.2 Launch 文件

- **所有路径参数化**：地图文件、PCD 路径用 `<arg>` 暴露，默认值可运行，但不硬编码特定日期/用户路径。
  ```xml
  <!-- 正确 -->
  <arg name="map_pcd" default="$(arg map3d_dir)/map.pcd" />
  <param name="pcd_path" value="$(arg map_pcd)" />

  <!-- 错误 -->
  <param name="pcd_path" value="/workspace/map3D/map_20260730_121040.pcd" />
  ```
- 用 `$(find pkg)` 定位包内资源，不用绝对路径。
- `rosparam` 配置统一放 `config/*.yaml`，launch 中用 `<rosparam command="load">` 加载。
- RViz 配置放 `rviz/*.rviz`，launch 中用 `args="-d $(find pkg)/rviz/xxx.rviz"` 引用。

### 2.3 Topic / Frame / Param 命名

- **Topic**：`snake_case`，带前导 `/`：`/cmd_vel`、`/cmd_vel_smooth`、`/move_base/GlobalPlanner/plan`。
- **Frame**：`/map`、`/odom`、`/base_link`、`/body`、`/slam_odom`——与现有 TF 树一致，不得自创新名。
- **ROS Param**：节点私有参数用 `~` 前缀（`~base_url`、`~robot_ip`、`~pcd_path`），全局参数用 `/move_base/...` 命名空间。
- **网络/IP/端口**：一律从 `rosparam` 读取，默认值保留现有配置，不在源码中硬编码。

### 2.4 Service / Msg 定义

- `.srv` 文件放 `srv/`，字段命名 `snake_case`，返回值须包含 `int32 status` + `string message`（现有 `SaveMap.srv`、`SlamReLoc.srv` 遵循此模式）。
- 自定义 `.msg` 放 `msg/`，`package.xml` 声明 `message_generation` / `message_runtime`。

---

## 3. Python 控制器规范

### 3.1 控制器基类模式

所有 G1 运动控制器**必须继承 `G1BaseController`**（`g1_controller_base.py`），只覆盖策略钩子：

```python
from g1_controller_base import G1BaseController

class MyController(G1BaseController):
    node_name = "unitree_my_controller"      # ROS 节点名
    lock_style = "latch"                      # "latch" | "counter" | "none"
    use_stopmove_deadband = True              # |cmd|<0.01 时 StopMove 还是 Move

    def configure_params(self):
        """设置 max_v / max_acc / 权重 / 增益。保留各自具体数值。"""
        self.max_vx = 1.5
        self.max_acc_v = 5.0
        ...

    def setup_feedback(self):
        """订阅 DDS 里程计或 ROS /odom。无反馈则 pass。"""
        ...

    def solve_step(self, v_current, v_target, v_last_cmd, max_v, max_acc, **kw):
        """单轴求解，返回命令速度。"""
        ...

    def process_cmd_vel(self, msg):
        """cmd_vel 预处理（禁倒车 / 角速度限幅）。默认直通。"""
        ...
```

- **禁止复制粘贴整个控制器**——新变体只写差异部分，共享逻辑在基类。
- 入口统一用 `G1BaseController.run(MyController)`。

### 3.2 参数化与配置

- 所有速度/加速度/权重/PID 增益在 `configure_params()` 中设为实例属性，**不散落在 `__init__` 各处**。
- 具体数值视为数据，保留各自调参结果，不强行统一。
- 期望可外部调参时，用 `rospy.get_param("~name", default)`，默认值即现有值。

### 3.3 异常与日志

- SDK 初始化（`ChannelFactoryInitialize`、`Init`）须 `try/except`，失败 `sys.exit(-1)`。
- 控制循环内的异常**不得静默吞掉**：`except Exception as e: rospy.logerr(...)`，不得裸 `except:`。
- 调试打印用 `rospy.loginfo_throttle` / `rospy.logwarn_throttle` 限流，不用 `print` 做高频输出（`int(t*5)%1==0` 永真式禁止）。
- 长期运行的节点须注册 `rospy.on_shutdown`，退出前发 `StopMove()` / `Damp()`。

### 3.4 安全底线（推荐补齐，当前代码缺失）

- **里程计新鲜度看门狗**：回调记录 `rospy.Time.now()`，控制循环中若 >0.5s 无新反馈，零目标并发 `StopMove`。
- **速度饱和**：所有 `solve_step` 返回值须 clip 到 `±max_v`；开环控制器在送 `Move()` 前也须 clip。
- **陈旧指令看门狗**：`cmd_vel_callback` 记时间戳，>0.5s 无新指令则发零速。

---

## 4. C++ SLAM 规范

### 4.1 头文件

- 用 `#pragma once`（现有 `commons.h` 遵循此模式），不用 `#ifndef` 守卫。
- Eigen 类型所在的 `struct`/`class` 须考虑对齐：`std::vector<EigenType>` 用 `Eigen::aligned_allocator`，或编译为 C++17。

### 4.2 内存与资源

- **RAII 优先**：用 `std::lock_guard` / `std::unique_lock`，禁止手写 `mutex.lock()/unlock()`。
- **禁止信号处理器内做 I/O / 加锁**：`signalHandler` 只设 `std::atomic<bool>` 标志 + `ros::shutdown()`，保存逻辑移到 `main()` 线程 join 后。
- **PCL / 文件 I/O 返回值必须检查**：`PCDReader::read()`、`PCDWriter::writeBinaryCompressed()` 返回非 0 时报错，不得静默继续。
- **定长缓冲须 clamp**：用扫描尺寸索引预分配数组前，`std::min(size, NUM_MAX_POINTS)`。

### 4.3 线程安全

- 跨线程共享状态（`SharedData` 的 `key_poses`、`cloud_history`、`loop_history`、各 flag）**读写均须持同一把锁**。
- 布尔 flag 跨线程传递用 `std::atomic<bool>`，不用 `bool`。
- `std::thread` 析构前必须 `join()`，否则 `std::terminate`。

### 4.4 数值健壮性

- 除以范数前检查 `> epsilon`（`mean_acc_.norm()`、`normvec.norm()`）。
- 空容器访问前检查 `empty()`（`points.back()`、`points.end()-1`）。
- `asin` 参数 clamp 到 `[-1, 1]` 防止 NaN（`rotate2rpy`）。

### 4.5 构建配置

- `CMakeLists.txt` 中 C++ 标准只声明一次：`set(CMAKE_CXX_STANDARD 14)` + `set(CMAKE_CXX_STANDARD_REQUIRED ON)`，不重复设 `-std=` 标志。
- OpenMP 显式 `target_link_libraries(... OpenMP::OpenMP_CXX)`，不靠 `CMAKE_CXX_FLAGS` 隐式链接。
- 测试用临时节点不加入 `add_executable`；已删除的源文件须同步移除构建目标。

---

## 5. 新增包 / 模块检查清单

提交新包或重大改动前，确认：

- [ ] `package.xml` 的 license/maintainer/description 已填实际值
- [ ] `CMakeLists.txt` 无重复标准标志、无死构建目标
- [ ] launch 文件路径全部 `<arg>` 参数化，无硬编码日期/用户路径
- [ ] 网络 IP 从 `rosparam` 读取
- [ ] Python 控制器继承 `G1BaseController`，无复制粘贴
- [ ] C++ 跨线程共享状态持锁，信号处理器仅设标志
- [ ] 无构建产物 / 隐私数据 / IDE 配置入库
- [ ] `py_compile` 通过（无 ROS 环境时）；有 ROS 环境时 `catkin_make` 通过
- [ ] 提交信息为 `<类型>: <简述>` 格式
