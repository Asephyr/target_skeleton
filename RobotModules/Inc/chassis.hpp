/**
 *******************************************************************************
 * @file      :chassis.hpp
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
#ifndef ROBOT_MODULES_CHASSIS_HPP_
#define ROBOT_MODULES_CHASSIS_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

#include "ins_imu.hpp"
#include "ins_motor.hpp"
#include "ins_pid.hpp"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* ============================================================================
 * 【新人任务】声明平衡底盘机构类 Chassis（namespace robot）。
 *
 * 你要做：
 *   1. 设计 robot::Chassis，承载本车的两条控制链路：
 *      - 轮组：两轮平衡的 LQR 状态反馈；
 *      - 旋转电机：GM6020 的速度闭环。
 *   2. 提供工作模式枚举（待机 / 轨道巡航 / 轨道手动 / 手动平衡类）与
 *      update()（按模式更新参考量）/ run()（生成并下发指令）两个周期入口，
 *      另提供 standby()（急停归零）与 reset()。
 *   3. 设计 Config 结构体，把 LQR 增益与指令限幅集中为可整定参数，
 *      构造时传入。
 *   4. 提供注册接口（左右轮电机、旋转电机、旋转 PID、IMU）与
 *      setWorkingMode() / setRcInput()，供 Robot 每周期下发遥控输入。
 *
 * 约束 / 提示：
 *   - LQR 状态向量、控制律与符号约定见考核说明第 3 章（硬性约定，验收按此检查）。
 *   - 转向量不进入 LQR 状态向量，单独叠加到左右轮输出上。
 *   - 指令下发时必须做电机反馈掉线兜底：isOffline() 为真则零输出并复位 PID，
 *     防止掉线期间累积的指令在重新上线瞬间输出（硬性验收项）。
 *   - 电机指令只写到电机对象（setInput）；由 comm_task 每周期 encode 并
 *     通过 CAN 发出（CAN 层由自己实现，骨架不提供 CAN 通信组件）。
 * ==========================================================================*/

namespace robot {

class Chassis {
 public:
  enum class WorkingMode : uint8_t {
    kStandby,
    kRailCruise,
    kRailManual,
    kManualLocked,
    kManualBalance,
    kManualSpin,
  };

  Chassis() = default;
  ~Chassis() = default;

  void update();
  void run();
  void standby();
  void reset();

  void registerMotorLeft(hw_motor::Motor* motor);
  void registerMotorRight(hw_motor::Motor* motor);
  void registerMotorRotate(hw_motor::Motor* motor);
  void registerImu(hello_world::imu::Imu* imu);

  void setWorkingMode(WorkingMode mode);
  WorkingMode workingMode() const;
  void setRcInput(float lv, float lh, float rh);  // 下发过死区后的遥控输入

  bool isAllMotorsOnline();
  void applyCmd();

 private:
  hw_motor::Motor* motor_left_ = nullptr;
  hw_motor::Motor* motor_right_ = nullptr;
  hw_motor::Motor* motor_rotate_ = nullptr;
  hello_world::imu::Imu* imu_ = nullptr;
  WorkingMode working_mode_ = WorkingMode::kStandby;

  float left_cmd_ = 0.0f;          // 左轮输出指令
  float right_cmd_ = 0.0f;         // 右轮输出指令
  float rotate_cmd_ = 0.0f;        // 旋转电机输出指令
  bool zero_once_left_ = true;     // 左轮恢复/切换后首帧先发零
  bool zero_once_right_ = true;    // 右轮恢复/切换后首帧先发零
  bool zero_once_rotate_ = true;   // 旋转电机恢复/切换后首帧先发零
  float rc_lv_ = 0.0f;             // 左摇杆纵向输入
  float rc_lh_ = 0.0f;             // 左摇杆横向输入
  float rc_rh_ = 0.0f;             // 右摇杆横向输入

  void applyMotorCmd(hw_motor::Motor* motor, float cmd, bool& zero_once);  // 单电机安全下发
};

}  // namespace robot



/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
#endif /* ROBOT_MODULES_CHASSIS_HPP_ */
