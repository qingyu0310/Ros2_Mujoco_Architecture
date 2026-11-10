/**
 * @file sim.hpp
 * @author qingyu
 * @brief MuJoCo 仿真驱动：加载模型、推进物理、按引用交换过程数据
 * @version 0.1
 * @date 2026-11-10
 *
 * @copyright Copyright (c) 2026
 *
 * @note 不认识业务语义：不定义执行器/关节的名字含义，只按模型下标收发。
 *       过程数据的类型由使用方按名字查到下标后自行解释。
 */

#pragma once

#include <mujoco/mujoco.h>

/**
 * @brief MuJoCo 仿真驱动：一个模型、一份数据，控制量进、状态量出
 *
 * @note Init 与 Close 成对；析构不再重复释放
 * @note 线程约束：Step 只允许在拥有本对象的线程调用
 */
class Sim
{
public:
    /**
     * @brief 加载模型并分配数据
     *
     * @param model_path MJCF 文件路径
     * @return true=成功；false=加载失败，错误已通过 mujoco 的 error 缓冲带出
     */
    bool Init(const char* model_path);

    /**
     * @brief 推进一个物理步
     */
    void Step();

    /**
     * @brief 控制量缓冲，长度 = 模型执行器数
     */
    mjtNum* Ctrl();

    /**
     * @brief 读关节位置
     *
     * @param joint_id 关节下标，由 JointId 查得
     * @return 位置指针；关节非法时返回 nullptr
     */
    const mjtNum* Qpos(int joint_id) const;

    /**
     * @brief 读关节速度
     *
     * @param joint_id 关节下标，由 JointId 查得
     * @return 速度指针；关节非法时返回 nullptr
     */
    const mjtNum* Qvel(int joint_id) const;

    /**
     * @brief 按名字查执行器下标
     *
     * @param name 模型里的执行器名
     * @return 下标；找不到返回 -1
     */
    int ActuatorId(const char* name) const;

    /**
     * @brief 按名字查关节下标
     *
     * @param name 模型里的关节名
     * @return 下标；找不到返回 -1
     */
    int JointId(const char* name) const;

    /**
     * @brief 模型指针，交给界面用
     */
    mjModel* Model();

    /**
     * @brief 数据指针，交给界面用
     */
    mjData* Data();

    /**
     * @brief 仿真时间
     */
    double Time() const;

    /**
     * @brief 释放模型与数据
     */
    void Close();

private:
    mjModel* model_ = nullptr;   // 模型
    mjData*  data_  = nullptr;   // 数据
};
