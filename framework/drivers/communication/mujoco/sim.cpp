/**
 * @file sim.cpp
 * @author qingyu
 * @brief MuJoCo 仿真驱动实现
 * @version 0.1
 * @date 2026-11-10
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "sim.hpp"

#include "log.hpp"

/**
 * @brief 加载模型并分配数据
 */
bool Sim::Init(const char* model_path)
{
    if (model_path == nullptr)
    {
        return false;
    }

    char error[1024] = {};
    model_ = mj_loadXML(model_path, nullptr, error, sizeof(error));
    if (model_ == nullptr)
    {
        DUST_LOG_ERR("mujoco: load %s failed: %s", model_path, error);
        return false;
    }

    data_ = mj_makeData(model_);
    if (data_ == nullptr)
    {
        DUST_LOG_ERR("mujoco: makeData failed");
        mj_deleteModel(model_);
        model_ = nullptr;
        return false;
    }

    return true;
}

/**
 * @brief 推进一个物理步
 */
void Sim::Step()
{
    if (model_ != nullptr && data_ != nullptr)
    {
        mj_step(model_, data_);
    }
}

/**
 * @brief 控制量缓冲
 */
mjtNum* Sim::Ctrl()
{
    return data_ == nullptr ? nullptr : data_->ctrl;
}

/**
 * @brief 读关节位置
 */
const mjtNum* Sim::Qpos(int joint_id) const
{
    if (model_ == nullptr || data_ == nullptr || joint_id < 0 || joint_id >= model_->njnt)
    {
        return nullptr;
    }
    return data_->qpos + model_->jnt_qposadr[joint_id];
}

/**
 * @brief 读关节速度
 */
const mjtNum* Sim::Qvel(int joint_id) const
{
    if (model_ == nullptr || data_ == nullptr || joint_id < 0 || joint_id >= model_->njnt)
    {
        return nullptr;
    }
    return data_->qvel + model_->jnt_dofadr[joint_id];
}

/**
 * @brief 按名字查执行器下标
 */
int Sim::ActuatorId(const char* name) const
{
    if (model_ == nullptr || name == nullptr)
    {
        return -1;
    }
    return mj_name2id(model_, mjOBJ_ACTUATOR, name);
}

/**
 * @brief 按名字查关节下标
 */
int Sim::JointId(const char* name) const
{
    if (model_ == nullptr || name == nullptr)
    {
        return -1;
    }
    return mj_name2id(model_, mjOBJ_JOINT, name);
}

/**
 * @brief 模型指针，交给界面用
 */
mjModel* Sim::Model()
{
    return model_;
}

/**
 * @brief 数据指针，交给界面用
 */
mjData* Sim::Data()
{
    return data_;
}

/**
 * @brief 仿真时间
 */
double Sim::Time() const
{
    return data_ == nullptr ? 0.0 : data_->time;
}

/**
 * @brief 释放模型与数据
 */
void Sim::Close()
{
    if (data_ != nullptr)
    {
        mj_deleteData(data_);
        data_ = nullptr;
    }
    if (model_ != nullptr)
    {
        mj_deleteModel(model_);
        model_ = nullptr;
    }
}
