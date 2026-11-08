/**
 *******************************************************************************
 * @file      :main_task.cpp
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
#include "main_task.hpp"

#include "iwdg.h"
#include "tim.h"

#include "comm_task.hpp"
#include "ins_all.hpp"
#include "robot.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

namespace {

robot::Robot* robot_ptr = nullptr;

}  // namespace

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
void MainTaskInit(void) {
  /* TODO【新人任务】:
   * 一次性初始化（由 main() 在 while(1) 之前调用一次）：
   *   1. 通过 Instance 层工厂拿到顶层对象：robot_ptr = CreateRobot()。
   *      （依赖的电机 / IMU / 遥控器 / 底盘都已在工厂内注册好，这里不要再逐个注册。）
   *   2. 硬件初始化：CreateImu()->initHardware()。
   *      注意该调用内部阻塞（BMI088 检测 + 启动 + 零漂准备），不要在中断里做。
   *   3. 通讯初始化：CommTaskInit()。
   *   4. 启动 1ms 控制中断：HAL_TIM_Base_Start_IT(&htim6)。
   *
   * 注意：不要在这里写任何周期性逻辑。
   */


 
    robot_ptr = CreateRobot();
    if (robot_ptr == nullptr) {
      return;
   }

    auto* imu = CreateImu();
    if (imu != nullptr) {
      imu->initHardware();
    }

    CommTaskInit();
    HAL_TIM_Base_Start_IT(&htim6);
  

}

void MainTask(void) {
  /* TODO【新人任务】:
   * 1ms 控制周期入口：按「感知 → 执行」调用顶层状态机：
   *   robot_ptr->update();  // 更新数据 + 迁移电源状态
   *   robot_ptr->run();     // 按状态分发执行
   * 保持薄薄一层，不要在这里堆业务逻辑。
   */

  
     if (robot_ptr == nullptr) {
    return;
  }

  robot_ptr->update();
  robot_ptr->run();

}

/* Private function definitions ----------------------------------------------*/
/* TODO【新人任务】:
 * 实现 htim6 周期中断回调 HAL_TIM_PeriodElapsedCallback：
 *   - 先判中断来自 htim6（避免把别的定时器中断误当成 1ms tick）；
 *   - 喂独立看门狗 HAL_IWDG_Refresh(&hiwdg)；
 *   - 完成上电启动延时后，再调用 MainTask() 与 CommTask()。
 *
 * 思考题（考核点）：
 *   - 喂狗应该放在启动延时门控之前还是之后？为什么？
 * 
 *    HAL_IWDG_Refresh() 放在 htim6 判断之后、启动延时门控之前，证明 1ms 控制循环还活着
 *   
 * - 如果只在"遥控器串口收到数据"时喂狗，遥控器失联会发生什么？
 *     复位之后、程序重新跑起来之前，电机还能收到零指令帧吗？   不能
 *   - 喂狗条件应当证明"控制循环还活着"还是"遥控器还连着"？遥控器失联应由
 *     软件离线保护兜底，看门狗只负责控制循环卡死。
 *
 * 注意：
 *   - HAL 回调是 C 链接，需用 extern "C" 包裹，否则中断里找不到符号。
 */

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
  if (htim != &htim6) {
    return;
  }

  HAL_IWDG_Refresh(&hiwdg);

  static uint32_t startup_tick = 0;
  if (startup_tick < 1000u) {
    ++startup_tick;
    return;
  }

  MainTask();
  CommTask();
}