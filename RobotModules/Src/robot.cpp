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
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
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

void Robot::update() {}

void Robot::run() {}

void Robot::standby() {}

void Robot::reset() {}

void Robot::registerChassis(Chassis* chassis) { chassis_ = chassis; }

void Robot::registerImu(hello_world::imu::Imu* imu) { imu_ = imu; }

void Robot::registerRc(hello_world::remote_control::DT7* rc) { rc_ = rc; }

}  // namespace robot
/* Private function definitions ----------------------------------------------*/
