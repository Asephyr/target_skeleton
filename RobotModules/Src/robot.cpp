/**
 *******************************************************************************
 * @file      :robot.cpp
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
/* Includes ------------------------------------------------------------------*/
#include "robot.hpp"

#include "ins_rc.hpp"
#include "tick.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
namespace {

constexpr uint32_t kResurrectionDelayMs = 1000;  // Resurrection 最短停留 1s

float ApplyDeadzone(float value, float threshold) {
  return (value > -threshold && value < threshold) ? 0.0f : value;  // 死区内归零
}

}  // namespace
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
/* ============================================================================
 * 【新人任务】实现 robot.hpp 中声明的顶层状态机。
 *
 * 你要做：
 *   1. update()：按「更新数据 → 迁移电源状态」组织：
 *      - 调用 IMU 的 update()（必须在 1ms 周期内调用，Mahony 依赖固定采样周期）；
 *      - 锁存零漂标定完成标志；
 *      - updatePwrState() 实现 Dead → Resurrection → Working 的迁移条件。
 *   2. runOnDead() / runOnResurrection()：复位数据并调用 standby() 级联到底盘。
 *   3. runOnWorking()：按「模式映射 → 下发遥控输入 → 底盘更新 → 底盘执行」
 *      的顺序组织，即每周期调用底盘机构的 update() 与 run()。
 *   4. standby()：让底盘进入待机（急停）状态，所有电机零输出。
 *   5. reset()：完整复位。
 *   6. runAlways()：每个周期都执行的动作（无需求可留空）。
 *
 * 约束 / 提示：
 *   - 遥控器离线保护放在模式映射入口：isOffline() 为真时直接回到待机模式，
 *     不要继续读取已冻结的拨杆 / 摇杆值。
 *   - 不要在这里直接操作电机：电机指令统一由底盘机构生成与下发。
 *   - 平衡车特有：上电必须先进待机档，任何异常都回落到待机。
 * ==========================================================================*/

namespace robot {

void Robot::update() {
  if (imu_ != nullptr) {
    imu_->update();  // 1ms 更新姿态与零漂
  }
  updatePwrState();  // 更新生命状态
}

void Robot::run() {
  switch (life_state_) {
    case LifeState::kDead:
      runOnDead();  // Dead 态：零输出
      break;
    case LifeState::kResurrection:
      runOnResurrection();  // 自检态：零输出
      break;
    case LifeState::kWorking:
      runOnWorking();  // 工作态：底盘周期
      break;
    default:
      standby();  // 异常状态回待机
      break;
  }
}

void Robot::standby() {
  working_mode_ = Chassis::WorkingMode::kStandby;  // 记录待机模式
  if (chassis_ != nullptr) {
    chassis_->standby();  // 底盘零输出并复位
  }
}

void Robot::reset() {
  life_state_ = LifeState::kDead;                 // 回到上电初始态
  working_mode_ = Chassis::WorkingMode::kStandby;  // 默认待机
  state_enter_tick_ = 0;                          // 清状态计时
  mode_armed_ = false;                            // 重新要求先回待机档
  if (chassis_ != nullptr) {
    chassis_->setWorkingMode(Chassis::WorkingMode::kStandby);  // 底盘回待机
    chassis_->reset();  // 清底盘控制状态
  }
}

void Robot::updatePwrState() {
  switch (life_state_) {
    case LifeState::kDead: {
      if (chassis_ != nullptr && imu_ != nullptr) {
        life_state_ = LifeState::kResurrection;  // 依赖齐全，开始自检
        state_enter_tick_ = hello_world::tick::GetTickMs();  // 记录进入时刻
        chassis_->setWorkingMode(Chassis::WorkingMode::kStandby);  // 底盘待机
        chassis_->reset();  // 清残留状态
      }
      break;
    }
    case LifeState::kResurrection: {
      const bool imu_ready =
          imu_ != nullptr && imu_->isOffsetCalcFinished();  // IMU 零漂完成
      const bool motors_ready =
          chassis_ != nullptr && chassis_->isAllMotorsOnline();  // 电机在线
      const bool delay_done =
          hello_world::tick::GetTickMs() - state_enter_tick_ >=
          kResurrectionDelayMs;  // 自检满 1s
      if (imu_ready && motors_ready && delay_done) {
        life_state_ = LifeState::kWorking;  // 自检通过，进入工作态
        working_mode_ = Chassis::WorkingMode::kStandby;  // 上电默认待机
        mode_armed_ = false;  // 上电后先等一次待机档
        chassis_->setWorkingMode(working_mode_);  // 同步到底盘
        chassis_->reset();  // 清自检残留
        state_enter_tick_ = hello_world::tick::GetTickMs();  // 记录进入时刻
      }
      break;
    }
    case LifeState::kWorking:
      break;  // 工作态由底盘处理离线保护
    default:
      life_state_ = LifeState::kDead;  // 异常回 Dead
      break;
  }
}

void Robot::runOnDead() {
  standby();  // Dead 态强制零输出
}

void Robot::runOnResurrection() {
  standby();  // 自检态强制零输出
}

void Robot::runOnWorking() {
  if (chassis_ == nullptr) {
    return;  // 底盘未注册，保持不动
  }

  updateWorkingMode();  // 遥控入口保护 + 拨杆映射

  if (working_mode_ == Chassis::WorkingMode::kStandby) {
    standby();  // 急停/离线/未定义组合
    return;
  }

  chassis_->update();  // 底盘按模式更新参考/状态
  chassis_->run();      // 底盘生成并下发指令
}

void Robot::updateWorkingMode() {
  if (rc_ == nullptr || rc_->isOffline()) {
    working_mode_ = Chassis::WorkingMode::kStandby;  // 失联强制待机
    return;
  }

  using hello_world::remote_control::SwitchState;
  const SwitchState l = rc_->rc_l_switch();
  const SwitchState r = rc_->rc_r_switch();

  Chassis::WorkingMode mode = Chassis::WorkingMode::kStandby;
  if (l == SwitchState::kUp && r == SwitchState::kUp) {
    mode = Chassis::WorkingMode::kRailCruise;  // 左上 + 右上：轨道巡航
  } else if (l == SwitchState::kUp && r == SwitchState::kMid) {
    mode = Chassis::WorkingMode::kRailManual;  // 左上 + 右中：轨道手动
  } else if (l == SwitchState::kMid && r == SwitchState::kUp) {
    mode = Chassis::WorkingMode::kManualLocked;  // 左中 + 右上：手动锁定
  } else if (l == SwitchState::kMid && r == SwitchState::kMid) {
    mode = Chassis::WorkingMode::kManualBalance;  // 左中 + 右中：手动平衡
  } else if (l == SwitchState::kMid && r == SwitchState::kDown) {
    mode = Chassis::WorkingMode::kManualSpin;  // 左中 + 右下：手动旋转
  } else {
    mode = Chassis::WorkingMode::kStandby;  // 其余组合：待机/急停
  }

  if (mode == Chassis::WorkingMode::kStandby) {
    mode_armed_ = true;  // 已确认待机档
  } else if (!mode_armed_) {
    mode = Chassis::WorkingMode::kStandby;  // 上电未经过待机档，先锁在待机
  }

  working_mode_ = mode;           // 记录当前模式
  chassis_->setWorkingMode(mode);  // 模式变化时底盘复位

  if (mode == Chassis::WorkingMode::kStandby) {
    return;  // 待机/急停不再下发摇杆
  }

  chassis_->setRcInput(ApplyDeadzone(rc_->rc_lv(), 0.05f),  // 纵向过死区
                       ApplyDeadzone(rc_->rc_lh(), 0.05f),  // 横向过死区
                       rc_->rc_rh());  // 旋转步进原始值
}

void Robot::registerChassis(Chassis* chassis) { chassis_ = chassis; }

void Robot::registerImu(hello_world::imu::Imu* imu) { imu_ = imu; }

void Robot::registerRc(hello_world::remote_control::DT7* rc) { rc_ = rc; }

}  // namespace robot
/* Private function definitions ----------------------------------------------*/
