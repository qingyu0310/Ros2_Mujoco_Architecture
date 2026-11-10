# ============================================================================
#
# framework/drivers/CMakeLists.txt —— 底层通讯/设备驱动装配
#
# 骨架阶段没有驱动实现，只导出 include 路径。
# 新增驱动时照 CONFIG_DUST_COM_* / CONFIG_DUST_DEV_* 加块，
# 源文件挂到 framework_drivers 上。
#
# ============================================================================

add_library(framework_drivers INTERFACE)
target_include_directories(framework_drivers INTERFACE ${PROJECT_SOURCE_DIR})

# ============================================================================
#
# CONFIG_DUST_COM_MUJOCO：MuJoCo 仿真驱动
#
# ============================================================================

if(CONFIG_DUST_COM_MUJOCO)
    add_subdirectory(communication/mujoco)
    target_link_libraries(framework_drivers INTERFACE framework_mujoco)
endif()
