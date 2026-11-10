/**
 * @file trd_sim.cpp
 * @author qingyu
 * @brief MuJoCo 仿真线程 —— 推进物理，界面在独立线程
 * @version 0.1
 * @date 2026-11-10
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "Init_entry.hpp"
#include "thread.hpp"
#include "framework/drivers/communication/mujoco/sim.hpp"
#include "framework/drivers/communication/mujoco/viewer.hpp"

namespace thread::sim
{

static constexpr const char* kModelPath = "mujoco/robot/square_chassis.xml";   // 模型文件

static Sim    sim_ {};           // 仿真驱动
static Viewer viewer_ {};        // 界面
static init::Thread thread_ {};  // 物理线程

/**
 * @brief 任务单拍：推进一个物理步
 *
 * @param context 未使用
 * @return Continue=正常
 */
static init::Thread::TaskResult Task(void* context)
{
    (void)context;

    sim_.Step();
    return init::Thread::TaskResult::Continue;
}

/**
 * @brief 初始化：加载模型并起界面
 * @return true=成功
 */
bool thread_init()
{
    if (!sim_.Init(kModelPath))
    {
        return false;
    }
    return viewer_.Init(sim_.Model(), sim_.Data(), kModelPath);
}

/**
 * @brief 启动物理线程
 *
 * 仿真挂了整条链路都没意义，按 Recover 处置。
 *
 * @return true=成功
 */
bool thread_start()
{
    thread_.Start(Task, std::chrono::milliseconds(0), nullptr, "sim", ThreadFaultConfig{nullptr, nullptr, ThreadFaultPolicy::Recover});
    return true;
}

/**
 * @brief 通知物理线程停止
 */
void thread_stop()
{
    thread_.RequestStop();
}

/**
 * @brief 等待物理线程退出并关界面
 */
void thread_join()
{
    thread_.Join();
    viewer_.Close();
}

/**
 * @brief 释放仿真驱动
 */
void thread_cleanup()
{
    sim_.Close();
}

REGISTER_INIT(thread_init, LateInit, High, HaltOnFail, thread_cleanup, "sim_init");
REGISTER_THREAD(thread_start, LateThread, thread_stop, thread_join, "sim_start");

} // namespace thread::sim
