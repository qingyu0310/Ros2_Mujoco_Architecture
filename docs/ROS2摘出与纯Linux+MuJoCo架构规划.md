# ROS2 摘出：纯 Linux 骨架 + MuJoCo 桥接

> 日期：2026-11-10。
> 骨架来源：`temp/Dust_Linux_Architecture-main`。
> 范围：**骨架落地 + MuJoCo 桥接**。

---

## 1. 做法

两件事：

1. `temp/Dust_Linux_Architecture-main` 整份搬进当前工作区，当底座
2. 加一层 MuJoCo bridge —— `framework/drivers/communication/mujoco/` + `project/thread/sim/trd_sim.cpp`

bridge 的位置与 EtherCAT 主站完全同构：

```text
EtherCAT：framework/drivers/communication/ethercat/ecat.hpp  +  project/thread/ethercat/trd_ethercat.cpp
MuJoCo  ：framework/drivers/communication/mujoco/sim.hpp     +  project/thread/sim/trd_sim.cpp
```

---

## 2. 骨架内容（照搬）

```text
Kconfig                    构建期功能选择入口
scripts/kconfig.py         Kconfig 前端，生成 .config / kconfig.cmake / autoconf.h
CMakeLists.txt             根构建，跑 Kconfig 前端 + 装配
framework/
├── init/                  注册宏、启动阶段、线程外壳、故障处置、信号处理
├── shell/                 调试台：命令分派、DUST_LOG、var、boot、thread
├── cmd/                   build/run 脚本、YAML 读取、链接脚本
├── topic/                 进程内单槽通道 channel.hpp
├── algorithm/             纯计算
├── modules/               设备能力（骨架阶段空装配位）
└── drivers/               总线与硬件驱动（新增 mujoco）
project/
├── Kconfig                门禁 TRD_* / USE_*
├── thread/                线程
└── CMakeLists.txt         装配入口
```

---

## 3. MuJoCo 桥接

### 3.1 驱动 `framework/drivers/communication/mujoco/sim.hpp`

```cpp
/**
 * @brief MuJoCo 仿真驱动：加载模型、推进物理、按引用交换过程数据
 *
 * 不认识业务语义，只按模型下标收发。
 */
class Sim
{
public:
    bool Init(const char* model_path);          // mj_loadXML + mj_makeData
    void Step();                                // mj_step，一拍
    mjtNum* Ctrl();                             // 长度 model_->nu
    const mjtNum* Qpos(int joint_id) const;
    const mjtNum* Qvel(int joint_id) const;
    int  ActuatorId(const char* name) const;
    int  JointId(const char* name) const;
    double Time() const;
    void Close();                               // mj_deleteData + mj_deleteModel
};
```

### 3.2 界面 `framework/drivers/communication/mujoco/viewer.hpp`

用 `Simulate(..., is_passive = true)`。被动模式下 `Simulate` 不拥有 `mjModel/mjData`（`simulate.h:141-144`），界面只做 `Sync()` + `Render()`，**循环由 `init::Thread` 自己写**，不调 `RenderLoop()`。

好处：不用编 `simulate/main.cc` 那套双线程，没有 `Simulate::mtx` 竞争，停止语义与 `RequestStop()` 天然对齐。

### 3.3 线程 `project/thread/sim/trd_sim.cpp`

```cpp
static Sim sim_ {};
static init::Thread thread_ {};

static init::Thread::TaskResult Task(void* context)
{
    sim_.Step();
    return init::Thread::TaskResult::Continue;
}

bool thread_init()  { return sim_.Init(config_.model_path.c_str()); }
bool thread_start() { thread_.Start(Task, std::chrono::milliseconds(0), nullptr, "sim",
                                    ThreadFaultConfig{nullptr, nullptr, ThreadFaultPolicy::Recover}); return true; }
void thread_stop()  { thread_.RequestStop(); }
void thread_join()  { thread_.Join(); }

REGISTER_INIT(thread_init, LateInit, High, HaltOnFail, nullptr, "sim_init");
REGISTER_THREAD(thread_start, LateThread, thread_stop, thread_join, "sim_start");
```

`period` 传 0：物理步长由 `model_->opt.timestep` 与 `mj_step` 决定，不跟外壳周期绑定。

### 3.4 资产与配置

- 模型资产：`mujoco/models|robot|worlds` → `project/mujoco/`
- 配置：`project/thread/sim/sim.yaml`，键 `model`（相对配置根的路径）、`headless`、相机参数
- 读取：`cmd::config::Config::Load("project/thread/sim/sim.yaml", {"model"})`

---

## 4. 构建改动

### 根 `CMakeLists.txt` 追加

```cmake
if(CONFIG_DUST_COM_MUJOCO)
    list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/framework/drivers/communication/mujoco/cmake")
    find_package(Mujoco REQUIRED)
endif()

if(CONFIG_DUST_COM_MUJOCO_VIEWER)
    find_package(glfw3 REQUIRED)
    set(OpenGL_GL_PREFERENCE GLVND)
    find_package(OpenGL REQUIRED)
endif()
```

`FindMujoco.cmake` 从 `mujoco/cmake/` 原样搬到 `framework/drivers/communication/mujoco/cmake/`。

### `framework/drivers/Kconfig` 追加

```kconfig
config DUST_COM_MUJOCO
    bool "MuJoCo simulation driver"
    default n
    help
      加载 MJCF、推进物理、按 ctrl/qpos 交换过程数据。

config DUST_COM_MUJOCO_VIEWER
    bool "MuJoCo interactive viewer (passive simulate)"
    default y
    depends on DUST_COM_MUJOCO
    help
      起 Simulate 界面（被动模式）。关掉即 headless。
```

### `project/Kconfig` 追加

```kconfig
config TRD_SIM
    bool "MuJoCo sim thread"
    default y
    select DUST_COM_MUJOCO
```

---

## 5. 实施步骤

> 用户说"开始第 N 步"= 只做表格里那一步，改完停下报告，等"继续"。

| 步 | 文件 | 动作 | 依赖前置 | 连带 |
| --- | --- | --- | --- | --- |
| 1 | 根 `CMakeLists.txt`、`Kconfig` | 从骨架整份复制 | — | 编不过（framework/project 未就位） |
| 2 | `scripts/kconfig.py` | 整份复制 | 1 | 编不过 |
| 3 | `framework/init/`、`framework/cmd/`、`framework/shell/`、`framework/topic/` | 四个目录整份复制 | 1 | 编不过 |
| 4 | `framework/CMakeLists.txt`、`framework/algorithm/` | 复制骨架装配；算法保留现有代码，补 `math/scalar.hpp`（只在 `install/` 里有） | 3 | 编不过 |
| 5 | `project/CMakeLists.txt`、`project/Kconfig`、`project/thread/CMakeLists.txt`、`project/thread/test/trd_test.cpp` | 整份复制 | 3 | **第 5 步后骨架可编译** |
| 6 | `.clangd`、`.vscode/settings.json`、`.gitignore` | 按骨架改（CompilationDatabase 指向本机 `project/build`） | 5 | — |
| 7 | `framework/drivers/communication/mujoco/cmake/FindMujoco.cmake` | 从 `mujoco/cmake/` 搬 | 5 | — |
| 8 | `framework/drivers/communication/mujoco/sim.hpp`、`sim.cpp` | 新建驱动 | 7 | — |
| 9 | `framework/drivers/communication/mujoco/viewer.hpp`、`viewer.cpp` | 新建被动模式界面 | 8 | 编不过（缺 glfw 链接） |
| 10 | `framework/drivers/CMakeLists.txt`、`framework/drivers/Kconfig` | 加 `DUST_COM_MUJOCO` / `_VIEWER` 与源文件条件 | 8、9 | — |
| 11 | `mujoco/models`、`robot`、`worlds` | 移到 `project/mujoco/` | 8 | — |
| 12 | `project/thread/sim/trd_sim.cpp`、`sim.yaml` | 新建仿真线程 | 8、11 | — |
| 13 | `project/Kconfig`、`project/thread/CMakeLists.txt` | 加 `TRD_SIM` | 12 | **第 13 步后 MuJoCo 可跑** |

---

## 6. 验收

- [ ] `dust build` 通过，无 ament / colcon 依赖
- [ ] `dust run` 后 `thread list` 有 `sim`
- [ ] `thread show sim` 的 `run_count` 在涨
- [ ] 界面能看到模型，窗口关闭后进程正常退出
- [ ] `grep -rn "rclcpp\|rosidl\|ament\|colcon" framework/ project/ scripts/ Kconfig CMakeLists.txt` 为空

---

## 7. 待拍板

| # | 问题 | 推荐 |
| --- | --- | --- |
| A | 界面用被动模式自写循环，还是保留现有 `RenderLoop()` 双线程 | 被动模式 |
| B | 资产放 `project/mujoco/` 还是 `framework/drivers/communication/mujoco/assets/` | `project/mujoco/` |
