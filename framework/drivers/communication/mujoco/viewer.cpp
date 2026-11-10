/**
 * @file viewer.cpp
 * @author qingyu
 * @brief MuJoCo 界面线程实现
 * @version 0.3
 * @date 2026-11-10
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "viewer.hpp"

#include <simulate/glfw_adapter.h>

Viewer::~Viewer()
{
    Close();
}

/**
 * @brief 启动界面线程，并把模型交给它
 */
bool Viewer::Init(mjModel* model, mjData* data, const char* filename)
{
    if (model == nullptr || data == nullptr || thread_.joinable())
    {
        return false;
    }

    thread_ = std::thread([this]
    {
        mjvCamera  camera;
        mjvOption  option;
        mjvPerturb perturb;

        mjv_defaultCamera(&camera);
        mjv_defaultOption(&option);
        mjv_defaultPerturb(&perturb);

        // 窗口、渲染上下文、Simulate 都在这条线程里建：GLFW 要求
        simulate_ = std::make_unique<mujoco::Simulate>(
            std::make_unique<mujoco::GlfwAdapter>(), &camera, &option, &perturb, /* is_passive = */ false);

        simulate_->RenderLoop();
    });

    // 等界面线程把 Simulate 建出来，之后才能 Load（Load 要等 RenderLoop 清 loadrequest）
    while (simulate_ == nullptr)
    {
        std::this_thread::yield();
    }

    simulate_->Load(model, data, filename == nullptr ? "" : filename);
    return true;
}

/**
 * @brief 物理侧与界面同步用的互斥锁
 */
mujoco::SimulateMutex& Viewer::Mutex()
{
    return simulate_->mtx;
}

/**
 * @brief 请求界面退出并等待线程结束
 */
void Viewer::Close()
{
    if (simulate_ != nullptr)
    {
        simulate_->exitrequest.store(1);
    }
    if (thread_.joinable())
    {
        thread_.join();
    }
    simulate_.reset();
}
