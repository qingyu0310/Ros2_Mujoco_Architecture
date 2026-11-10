# ============================================================================
#
# framework/drivers/communication/mujoco —— MuJoCo 仿真驱动
#
# CONFIG_DUST_COM_MUJOCO        加载模型、推进物理
# CONFIG_DUST_COM_MUJOCO_VIEWER 追加界面（simulate 源码 + glfw + OpenGL）
#
# ============================================================================

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(Mujoco REQUIRED)

add_library(framework_mujoco STATIC sim.cpp)
target_include_directories(framework_mujoco PUBLIC
    ${PROJECT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/framework/shell/log
    ${PROJECT_SOURCE_DIR}/framework/shell/log/process
    ${PROJECT_SOURCE_DIR}/framework/shell/log/queue
    ${PROJECT_SOURCE_DIR}/framework/shell/log/transport
    ${PROJECT_SOURCE_DIR}/framework/shell/log/editor
    ${Mujoco_INCLUDE_DIR})
target_link_libraries(framework_mujoco PUBLIC Mujoco::Mujoco)
target_compile_options(framework_mujoco PRIVATE -Wall -Wextra -Wpedantic)

if(CONFIG_DUST_COM_MUJOCO_VIEWER)
    find_package(glfw3 REQUIRED)
    set(OpenGL_GL_PREFERENCE GLVND)
    find_package(OpenGL REQUIRED)

    # MuJoCo 不把 simulate 发成库，官方集成包的做法也是直接编它的源码。
    # 头文件写成 <simulate/simulate.h>，所以 include 根是 SDK 目录（simulate 的上一级）。
    get_filename_component(MUJOCO_SDK_DIR "${Mujoco_INCLUDE_DIR}" DIRECTORY)
    if(NOT EXISTS "${MUJOCO_SDK_DIR}/simulate/simulate.cc")
        message(FATAL_ERROR
            "找不到 MuJoCo 的 simulate 源码：${MUJOCO_SDK_DIR}/simulate。"
            "设 MUJOCO_DIR，或把 SDK 装在 ~/.local/mujoco-<版本>")
    endif()

    # 第三方源码单独成一个目标：不套本工程的告警选项
    add_library(framework_mujoco_simulate STATIC
        ${MUJOCO_SDK_DIR}/simulate/simulate.cc
        ${MUJOCO_SDK_DIR}/simulate/glfw_adapter.cc
        ${MUJOCO_SDK_DIR}/simulate/platform_ui_adapter.cc
        ${MUJOCO_SDK_DIR}/simulate/glfw_dispatch.cc)
    target_include_directories(framework_mujoco_simulate SYSTEM PUBLIC
        ${MUJOCO_SDK_DIR}
        ${Mujoco_INCLUDE_DIR})
    target_link_libraries(framework_mujoco_simulate PUBLIC Mujoco::Mujoco glfw OpenGL::GL lodepng)

    target_sources(framework_mujoco PRIVATE viewer.cpp)
    target_include_directories(framework_mujoco SYSTEM PUBLIC ${MUJOCO_SDK_DIR})
    target_link_libraries(framework_mujoco PUBLIC framework_mujoco_simulate)
endif()
