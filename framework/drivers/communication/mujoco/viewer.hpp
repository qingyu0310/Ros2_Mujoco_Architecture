/**
 * @file viewer.hpp
 * @author qingyu
 * @brief MuJoCo 界面：RenderLoop 独占一条线程，模型由调用方在这条线程之外交进来
 * @version 0.3
 * @date 2026-11-10
 *
 * @copyright Copyright (c) 2026
 *
 * @note 线程布局照抄 MuJoCo 自己的 simulate/main.cc：界面一条线程跑 RenderLoop()，
 *       物理一条线程，两边靠 Simulate::mtx 串起来
 * @note Simulate 与 GlfwAdapter 必须在 RenderLoop 那条线程里构造（GLFW 绑定创建线程），
 *       而 Load 必须在这条线程之外调用——Load 内部要等 RenderLoop 清 loadrequest，
 *       放在同一条线程就是自己等自己
 */

#pragma once

#include <memory>
#include <thread>

#include <mujoco/mujoco.h>
#include <simulate/simulate.h>

/**
 * @brief MuJoCo 界面线程
 *
 * @note 析构会请求界面退出并等待线程结束
 */
class Viewer
{
public:
    ~Viewer();

    Viewer() = default;
    Viewer(const Viewer&) = delete;
    Viewer& operator=(const Viewer&) = delete;

    /**
     * @brief 启动界面线程，并把模型交给它
     *
     * @param model 模型，生命周期由调用方持有且长于本对象
     * @param data  数据，生命周期由调用方持有且长于本对象
     * @param filename 界面标题里显示的文件名
     * @return true=成功；false=参数非法或界面线程启动失败
     *
     * @note 必须在物理线程调用，不能放在界面线程
     */
    bool Init(mjModel* model, mjData* data, const char* filename);

    /**
     * @brief 物理侧与界面同步用的互斥锁
     */
    mujoco::SimulateMutex& Mutex();

    /**
     * @brief 请求界面退出并等待线程结束
     */
    void Close();

private:
    std::unique_ptr<mujoco::Simulate> simulate_;   // 界面，由界面线程构造
    std::thread                       thread_;     // 界面线程
};
