# Dummy ROS2 Jazzy · D0

开发记录：2026-10-20；收尾复核：2026-10-21。

当前仅完成 **D0 开发与 Windows 静态检查**，Ubuntu 24.04 / Jazzy 编译、MoveIt 运行与 RViz2 显示均待验证。用户的 Ubuntu 是双系统另一侧，本轮无法运行。未开展 D1～D6，也没有规划性能或执行精度结果。

## 来源与工作空间

- 上游：https://gitee.com/switchpi/dummy.git
- 分支：`auk`，提交：`3a9d17464308e1532b3d648a80da6db899cc1f01`
- 来源目录：`ros2/dummy_ws/src`，`auk` 是分支名，不是磁盘路径前缀。
- 只引入 `dummy-ros2_description` 与 `dummy_moveit_config`；没有复制 hardware、firmware 或机械 CAD。
- 保留描述包原名（含连字符），未随意重命名资源引用。若目标环境报告包名规范警告，先区分警告和实际构建失败。
- 原始 URDF 几何、关节参数、7 个 STL 均未修改；仅改动派生配置与构建，并新增 D0 入口。详见 [来源和版权](LICENSES_OR_ATTRIBUTION.md)、[D0 审查](docs/D0_report_2026-10-20.md)。

```text
src/dummy-ros2_description/   上游模型
src/dummy_moveit_config/     上游配置 + D0 启动/诊断
tools/audit_model.py         非 ROS 的静态检查
results/D0_2026-10-20/       本轮真实检查与溯源记录
```

## Ubuntu 侧 D0 验收（尚未执行）

在 Ubuntu 24.04 上，先确认已安装 ROS2 Jazzy 并在新终端执行下面命令。只在软件环境操作，不连接真实机械臂。

```bash
git clone https://github.com/pipi2023911-spec/dummy-ros2.git
cd dummy-ros2
source /opt/ros/jazzy/setup.bash
mkdir -p results/D0_ubuntu
cat /etc/os-release > results/D0_ubuntu/os-release.txt
printenv ROS_DISTRO > results/D0_ubuntu/ros-distro.txt
git rev-parse HEAD > results/D0_ubuntu/commit.txt
sudo apt update
sudo apt install ros-jazzy-moveit ros-jazzy-joint-state-publisher-gui \
  python3-colcon-common-extensions python3-rosdep liburdfdom-tools
# 仅当此系统尚未初始化 rosdep 时运行 sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
set -o pipefail
colcon build --symlink-install --event-handlers console_direct+ \
  2>&1 | tee results/D0_ubuntu/build.log
source install/setup.bash
dpkg-query -W 'ros-jazzy-*' > results/D0_ubuntu/ros-packages.txt
xacro "$(ros2 pkg prefix --share dummy_moveit_config)/config/dummy-ros2.urdf.xacro" \
  > results/D0_ubuntu/dummy.urdf
check_urdf results/D0_ubuntu/dummy.urdf \
  2>&1 | tee results/D0_ubuntu/check_urdf.log
ros2 launch dummy_moveit_config d0_check.launch.py \
  2>&1 | tee results/D0_ubuntu/model_check.log
```

逐条运行，构建/解析出现错误先停止，保留日志。`d0_model_check` 应输出六个规划变量、两个姿态的限位与自碰撞结果、六次 FK 矩阵变化，最后 `result=PASS`。同时检查进程退出码和日志：**launch 命令正常结束不能代替诊断 PASS**。PASS 只覆盖已检查状态，不证明完整运动空间没有自碰撞。

显示模型：

```bash
ros2 launch dummy_moveit_config d0_model.launch.py \
  2>&1 | tee results/D0_ubuntu/rviz.log
```

在关节 GUI 中小幅改变 Joint1～Joint6，观察对应连杆变化；在第二个已 source 的终端检查：

```bash
ros2 run tf2_ros tf2_echo world link6_1_1
```

验收时保存 RViz 截图和 TF 输出，确认 RobotModel 无资源加载错误、Fixed Frame 为 `world`、六轴均更新。Joint6 的变化可能主要表现为姿态，而非末端位置变化。RViz 可切换 RobotModel 的 Collision Enabled 查看碰撞网格；这仍不是碰撞计算。

本轮启动入口不启动 `move_group`、控制器或任何实物接口。上游其他启动文件仅保留参考，尤其 Servo 示例未构建；不要用它们作为本轮验收命令。

## Windows 静态检查复现

在隔离 Python 环境安装 `xacro==2.1.1`、`PyYAML==6.0.3` 后：

```text
python tools/audit_model.py --output results/D0_local/static_audit.json
```

脚本检查关节树、规划链、限位引用、STL 二进制结构及有限数值、Python/YAML 语法，并真正调用 Xacro 展开控制模型。在临时副本中替换 `$(find ...)` 为路径，所以它**没有验证 ament 包发现、ROS URDF 加载、碰撞或 RViz**。不要用 `python -O` 运行（会关闭断言）。

## 当前停止点

等待 Ubuntu 的构建、模型诊断、RViz 与自碰撞配置验收。没有通过 D0，也没有开始 D1。详细待决项及回退方式见审查报告。
