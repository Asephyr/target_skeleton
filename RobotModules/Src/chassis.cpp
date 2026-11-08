/**
 *******************************************************************************
 * @file      :chassis.cpp
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
#include "chassis.hpp"

#include "pid.hpp"
#include "td.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
/* ============================================================================
 * 【新人任务】实现 chassis.hpp 中声明的平衡底盘机构。
 *
 * 你要做：
 *   1. 轮组 LQR（核心难点）：
 *      - 构建状态向量 [x, v, θ, w]：θ 取 IMU pitch、w 取 gyro_pitch、
 *        v 取左右轮速均值（注意左右符号）、x 由 v 积分；
 *      - 控制律 u = -(k_x·err_x + k_v·err_v + k_θ·err_θ + k_w·err_w)；
 *      - 速度参考必须斜坡限幅（防指令突变翻车），参考位移由参考速度积分；
 *      - 左轮输出 = +u + turn，右轮输出 = -u + turn。
 *   2. 旋转电机速度环：
 *      - 右摇杆横向边沿触发步进目标速度，经 PID 计算电流指令；
 *      - 待机 / 纯平衡模式下目标清零并复位 PID。
 *   3. 指令下发：
 *      - 待机模式强制三电机零输出；
 *      - 电机反馈离线时零输出并复位 PID（掉线兜底）；
 *      - 只调用 setInput() 写指令，CAN 编码与发送在 comm_task 中完成。
 *   4. 进入待机 / 切换模式时清空参考量与 LQR 状态，避免残留指令突变。
 *
 * 约束 / 提示：
 *   - IMU 零漂未收敛前不得输出平衡指令。
 *   - 控制周期 1ms，位移 / 速度积分使用 0.001s。
 *   - 增益只是起点，需要在架起的车上小步长整定；先调 θ / w 让车能"站住"，
 *     再加入 v / x 项做位置保持。
 *   - 所有数学写在纸上推导清楚再上代码，符号错误会直接导致翻车。
 * ==========================================================================*/

namespace {

constexpr float kRotateStickTrigger = 0.8f;   // 右摇杆触发阈值
constexpr float kRotateStickRelease = 0.6f;   // 右摇杆回差释放阈值
constexpr float kRotateSpeedStep = 4.0f;      // 每次步进速度，单位 rad/s
constexpr float kRotateSpeedLimit = 35.0f;    // 旋转目标速度限幅，单位 rad/s

float ClampSym(float value, float limit) {
  if (value > limit) {
    return limit;
  }
  if (value < -limit) {
    return -limit;
  }
  return value;
}

}  // namespace

namespace robot {

void Chassis::update() {
  if (working_mode_ == WorkingMode::kStandby) {
    return;  // 待机不做积分/控制
  }

  const bool any_offline =
      motor_left_ == nullptr || motor_left_->isOffline() ||
      motor_right_ == nullptr || motor_right_->isOffline() ||
      motor_rotate_ == nullptr || motor_rotate_->isOffline();
  if (any_offline) {
    reset();  // 任一电机掉线：清空控制状态
    return;
  }

  updateRotateRef();  // 更新旋转目标速度
}

void Chassis::run() {
  if (working_mode_ == WorkingMode::kStandby) {
    left_cmd_ = 0.0f;    // 待机左轮零输出
    right_cmd_ = 0.0f;   // 待机右轮零输出
    rotate_cmd_ = 0.0f;  // 待机旋转电机零输出
    applyCmd();          // 下发零帧
    return;
  }

  left_cmd_ = 0.0f;   // 轮组 LQR 未接入，先零输出
  right_cmd_ = 0.0f;  // 右轮零输出
  runRotate();        // 旋转电机速度闭环
  applyCmd();         // 安全下发
}

void Chassis::standby() {
  setWorkingMode(WorkingMode::kStandby);  // 切到待机模式
  reset();                                // 清空残留控制状态
  left_cmd_ = 0.0f;                       // 左轮零输出
  right_cmd_ = 0.0f;                      // 右轮零输出
  rotate_cmd_ = 0.0f;                     // 旋转电机零输出
  applyCmd();                             // 立即下发零帧
}

void Chassis::reset() {
  left_cmd_ = 0.0f;        // 清空左轮指令
  right_cmd_ = 0.0f;       // 清空右轮指令
  rotate_cmd_ = 0.0f;      // 清空旋转电机指令
  zero_once_left_ = true;  // 复位后首帧先发零
  zero_once_right_ = true;
  zero_once_rotate_ = true;
  rc_lv_ = 0.0f;              // 清零遥控输入
  rc_lh_ = 0.0f;
  rc_rh_ = 0.0f;
  rotate_speed_ref_ = 0.0f;   // 清空旋转目标速度
  rc_rh_high_ = false;        // 清空右摇杆正向触发锁存
  rc_rh_low_ = false;         // 清空右摇杆负向触发锁存
  if (pid_rotate_ != nullptr) {
    pid_rotate_->reset();     // 复位旋转速度环 PID
  }
  if (filter_rotate_ != nullptr) {
    filter_rotate_->reset();  // 复位速度反馈滤波器
  }
}

void Chassis::registerMotorLeft(hw_motor::Motor* motor) {
  motor_left_ = motor;
}

void Chassis::registerMotorRight(hw_motor::Motor* motor) {
  motor_right_ = motor;
}

void Chassis::registerMotorRotate(hw_motor::Motor* motor) {
  motor_rotate_ = motor;
}

void Chassis::registerImu(hello_world::imu::Imu* imu) { imu_ = imu; }

void Chassis::registerPid(hw_pid::Pid* pid) {
  pid_rotate_ = pid;  // 保存旋转速度环 PID
  if (pid_rotate_ != nullptr) {
    pid_rotate_->reset();  // 注册时先清空 PID 状态
  }
}

void Chassis::registerFilter(hw_filter::Td* filter) {
  filter_rotate_ = filter;  // 保存旋转速度反馈滤波器
  if (filter_rotate_ != nullptr) {
    filter_rotate_->reset();  // 注册时先清空滤波器状态
  }
}

void Chassis::setRcInput(float lv, float lh, float rh) {
  rc_lv_ = lv;  // 左摇杆纵向
  rc_lh_ = lh;  // 左摇杆横向
  rc_rh_ = rh;  // 右摇杆横向
}

void Chassis::setWorkingMode(WorkingMode mode) {
  if (working_mode_ == mode) {
    return;
  }
  working_mode_ = mode;  // 更新工作模式
  reset();               // 模式切换清残留状态
}

Chassis::WorkingMode Chassis::workingMode() const { return working_mode_; }

bool Chassis::isAllMotorsOnline() {
  return motor_left_ != nullptr && motor_right_ != nullptr &&
         motor_rotate_ != nullptr && !motor_left_->isOffline() &&
         !motor_right_->isOffline() && !motor_rotate_->isOffline();
}

bool Chassis::isRotateEnabled() const {
  return working_mode_ != WorkingMode::kStandby;  // 当前按 3.4：非待机均允许旋转
}

void Chassis::updateRotateRef() {
  if (!isRotateEnabled()) {
    rotate_speed_ref_ = 0.0f;  // 待机或禁用旋转时目标清零
    rc_rh_high_ = false;
    rc_rh_low_ = false;
    return;
  }

  if (!rc_rh_high_ && rc_rh_ > kRotateStickTrigger) {
    rotate_speed_ref_ += kRotateSpeedStep;  // 正向一次步进
    rc_rh_high_ = true;
  } else if (rc_rh_ < kRotateStickRelease) {
    rc_rh_high_ = false;  // 回差释放，允许下次触发
  }

  if (!rc_rh_low_ && rc_rh_ < -kRotateStickTrigger) {
    rotate_speed_ref_ -= kRotateSpeedStep;  // 负向一次步进
    rc_rh_low_ = true;
  } else if (rc_rh_ > -kRotateStickRelease) {
    rc_rh_low_ = false;  // 回差释放，允许下次触发
  }

  rotate_speed_ref_ = ClampSym(rotate_speed_ref_, kRotateSpeedLimit);  // 目标限幅
}

void Chassis::runRotate() {
  const bool rotate_ready =
      isRotateEnabled() && motor_rotate_ != nullptr && pid_rotate_ != nullptr &&
      !motor_rotate_->isOffline();

  if (!rotate_ready) {
    rotate_cmd_ = 0.0f;  // 不可用时零输出
    if (pid_rotate_ != nullptr) {
      pid_rotate_->reset();  // 复位 PID
    }
    if (filter_rotate_ != nullptr) {
      filter_rotate_->reset();  // 复位滤波器
    }
    return;
  }

  float speed_fdb = motor_rotate_->vel();  // 旋转电机反馈速度
  if (filter_rotate_ != nullptr) {
    speed_fdb = filter_rotate_->update(speed_fdb);  // 速度反馈滤波
  }

  rotate_cmd_ = pid_rotate_->update(rotate_speed_ref_, speed_fdb);  // 速度环输出
}

void Chassis::applyCmd() {
  const bool any_offline =
      motor_left_ == nullptr || motor_left_->isOffline() ||
      motor_right_ == nullptr || motor_right_->isOffline() ||
      motor_rotate_ == nullptr || motor_rotate_->isOffline();
  if (any_offline) {
    left_cmd_ = 0.0f;    // 任一电机掉线：全部零输出
    right_cmd_ = 0.0f;
    rotate_cmd_ = 0.0f;
    zero_once_left_ = true;    // 恢复后首帧仍为零
    zero_once_right_ = true;
    zero_once_rotate_ = true;
    if (pid_rotate_ != nullptr) {
      pid_rotate_->reset();    // 掉线时复位旋转 PID
    }
    if (filter_rotate_ != nullptr) {
      filter_rotate_->reset(); // 掉线时复位速度滤波
    }
  }

  applyMotorCmd(motor_left_, left_cmd_, zero_once_left_);         // 左轮安全下发
  applyMotorCmd(motor_right_, right_cmd_, zero_once_right_);      // 右轮安全下发
  applyMotorCmd(motor_rotate_, rotate_cmd_, zero_once_rotate_);   // 旋转电机安全下发
}

void Chassis::applyMotorCmd(hw_motor::Motor* motor, float cmd,
                            bool& zero_once) {
  if (motor == nullptr) {
    return;
  }

  if (motor->isOffline()) {
    motor->setInput(0.0f);  // 掉线电机零输出
    zero_once = true;       // 恢复后首帧仍为零
  } else if (zero_once) {
    motor->setInput(0.0f);  // 恢复/切换后的首帧零输出
    zero_once = false;
  } else {
    motor->setInput(cmd);   // 正常下发指令
  }
  motor->setNeedToTransmit();  // 标记发送，零帧也要发
}

}  // namespace robot



/* Private function definitions ----------------------------------------------*/


