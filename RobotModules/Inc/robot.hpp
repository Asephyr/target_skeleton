/*
 * @Author: Asephyr 1558825085@qq.com
 * @Date: 2026-10-18 18:03:24
 * @LastEditors: Asephyr 1558825085@qq.com
 * @LastEditTime: 2026-11-07 21:00:07
 * @FilePath: \target_skeleton\RobotModules\Inc\robot.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 *******************************************************************************
 * @file      :robot.hpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ROBOT_MODULES_ROBOT_HPP_
#define ROBOT_MODULES_ROBOT_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

#include "chassis.hpp"

namespace hello_world {
namespace remote_control {
class DT7;
}  // namespace remote_control
}  // namespace hello_world
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* ============================================================================
 * 【新人任务】声明顶层状态机类 Robot（namespace robot）。
 *
 * 你要做：
 *   1. 设计 robot::Robot，作为本板每个控制周期（1ms）的入口：update() 汇总
 *      数据与电源状态，run() 按状态分发执行。
 *   2. 实现「生命状态」三态：
 *      - Dead：上电初始态，输出全零；
 *      - Resurrection：恢复自检，等待 IMU 零漂收敛 + 全部电机反馈在线 + 约 1s；
 *      - Working：正常控制。
 *      三态迁移逻辑自己组织（写在 update() / run() 或单独的迁移函数里均可）。
 *   3. Working 态下根据遥控器左右拨杆组合映射工作模式（见考核说明第 3 章），
 *      并把摇杆量（过死区后）与目标模式下发给底盘机构。
 *   4. 提供注册接口（registerChassis / registerImu / registerRc），
 *      由 Instance 层工厂统一接线。
 *   5. 实现离线保护：遥控器 isOffline() 时强制进入待机（急停）。
 *
 * 约束 / 提示：
 *   - 状态机的基类与迁移机制**由你自己设计**（原工程用组件库的
 *     hello_world::module::ModuleFsm，本骨架不提供）。可以参考
 *     code_control/ 下其他项目的 Robot 实现，但不要直接拷贝。
 *   - 控制周期入口由 Task/Src/main_task.cpp 的定时中断调用；本类不要开中断、
 *     不要死等延时。
 *   - 急停：遥控器拨到待机档、拨杆组合未定义、遥控器离线时，都必须保证所有
 *     电机指令归零（硬性验收项）。
 *   - 状态切换时记得复位上一状态残留的指令，避免电机突变。
 * ==========================================================================*/

namespace robot {

class Robot {
 public:
  enum class LifeState : uint8_t {
    kDead,
    kResurrection,
    kWorking,
  };

  Robot() = default;
  ~Robot() = default;

  void update();
  void run();
  void standby();
  void reset();

  void registerChassis(Chassis* chassis);
  void registerImu(hello_world::imu::Imu* imu);
  void registerRc(hello_world::remote_control::DT7* rc);

 private:
  Chassis* chassis_ = nullptr;
  hello_world::imu::Imu* imu_ = nullptr;
  hello_world::remote_control::DT7* rc_ = nullptr;
  LifeState life_state_ = LifeState::kDead;
  Chassis::WorkingMode working_mode_ = Chassis::WorkingMode::kStandby;
};

}  // namespace robot

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
#endif /* ROBOT_MODULES_ROBOT_HPP_ */
