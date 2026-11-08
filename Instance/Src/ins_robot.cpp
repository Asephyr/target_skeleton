/*
 * @Author: Asephyr 1558825085@qq.com
 * @Date: 2026-10-18 14:36:30
 * @LastEditors: Asephyr 1558825085@qq.com
 * @LastEditTime: 2026-11-01 21:48:27
 * @FilePath: \target_skeleton\Instance\Src\ins_robot.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 *******************************************************************************
 * @file      :ins_robot.cpp
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
#include "ins_robot.hpp"
#include "chassis.hpp"
#include "robot.hpp"
#include "ins_imu.hpp"
#include "ins_motor.hpp"
#include "ins_rc.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
robot::Robot* CreateRobot(void) {
  /* TODO【新人任务】:
   * 返回顶层状态机 robot::Robot 单例，并在此完成全部依赖注册。
   *
   * 你要做：
   *   1. 静态定义 robot::Chassis 与 robot::Robot 对象（类由你在
   *      RobotModules/Inc 下补全）。Chassis 的构造参数 Config 里放 LQR 增益与
   *      指令限幅，整定值以实车为准（见考核说明文档）。
   *   2. 首次调用时完成注册（重复调用直接返回同一对象）：
   *      - Robot 注册 Chassis、Imu、遥控器；
   *      - Chassis 注册左右轮电机、旋转电机、旋转 PID、Imu。
   *   3. 返回 Robot 的地址。
   *
   * 约束 / 提示：
   *   - 这里是唯一做依赖接线的地方：main_task 只调用 CreateRobot()，
   *     不再自己 Create*() 再逐个注册。
   *   - 静态初始化阶段不得触碰 HAL；IMU 的 initHardware() 由 main_task 显式调用。
   *   - 需要补 include："chassis.hpp"、"ins_imu.hpp"、"ins_motor.hpp"、
   *     "ins_pid.hpp"、"ins_rc.hpp"、"robot.hpp"。
   */
  
  static robot::Chassis chassis;
  static robot::Robot robot;
  static bool initialized = false;

  if (!initialized) {
    auto* imu = CreateImu();

    chassis.registerMotorLeft(CreateMotorLeft());
    chassis.registerMotorRight(CreateMotorRight());
    chassis.registerMotorRotate(CreateMotorRotate());
    chassis.registerImu(imu);

    robot.registerChassis(&chassis);
    robot.registerImu(imu);
    robot.registerRc(CreateRemoteControl());

    initialized = true;
  }

  return &robot;
}
/* Private function definitions ----------------------------------------------*/
