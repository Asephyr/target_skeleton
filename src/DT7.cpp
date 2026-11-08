/**
 *******************************************************************************
 * @file      : DT7.cpp
 * @brief     : 遥控器 DT7 接收类
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2023-12-05      Caikunzhen      1. 完成测试
 *  V1.1.0      2024-07-11      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *  该类依赖串口接收管理器 UartRxMgr，使用前请确保 UartRxMgr 按要求配置于初始化，其中
 *  串口波特率设置为 100kbps，字长 9 Bits(include Parity)，偶校验，停止位 1，只接收。
 *  串口接收管理器中 buf_len 设置为 19，max_process_data_len 设置为 18，eof_type 设
 *  置为 EofType::kIdle
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "DT7.hpp"

#include "assert.hpp"

namespace hello_world
{
namespace remote_control
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/

static const uint16_t kRcOffset = 1024u;
static const float kRcRatio = 1.0f / 660;
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

HW_OPTIMIZE_O2_START

DT7 &DT7::operator=(const DT7 &other)
{
  if (this != &other) {
    rc_lv_ = other.rc_lv_;
    rc_lh_ = other.rc_lh_;
    rc_rv_ = other.rc_rv_;
    rc_rh_ = other.rc_rh_;
    rc_wheel_ = other.rc_wheel_;

    rc_l_switch_ = other.rc_l_switch_;
    rc_r_switch_ = other.rc_r_switch_;
    last_rc_l_switch_ = other.last_rc_l_switch_;
    last_rc_r_switch_ = other.last_rc_r_switch_;

    mouse_l_btn_ = other.mouse_l_btn_;
    mouse_r_btn_ = other.mouse_r_btn_;
    mouse_x_ = other.mouse_x_;
    mouse_y_ = other.mouse_y_;
    mouse_z_ = other.mouse_z_;

    key_ = other.key_;

    oc_ = other.oc_;

    is_updated_ = other.is_updated_;
    update_cb_ = other.update_cb_;

    decode_success_cnt_ = other.decode_success_cnt_;
    decode_fail_cnt_ = other.decode_fail_cnt_;
  }

  return *this;
}

DT7::DT7(DT7 &&other)
{
  rc_lv_ = other.rc_lv_;
  rc_lh_ = other.rc_lh_;
  rc_rv_ = other.rc_rv_;
  rc_rh_ = other.rc_rh_;
  rc_wheel_ = other.rc_wheel_;

  rc_l_switch_ = other.rc_l_switch_;
  rc_r_switch_ = other.rc_r_switch_;
  last_rc_l_switch_ = other.last_rc_l_switch_;
  last_rc_r_switch_ = other.last_rc_r_switch_;

  mouse_l_btn_ = other.mouse_l_btn_;
  mouse_r_btn_ = other.mouse_r_btn_;
  mouse_x_ = other.mouse_x_;
  mouse_y_ = other.mouse_y_;
  mouse_z_ = other.mouse_z_;

  key_ = other.key_;

  oc_ = std::move(other.oc_);

  is_updated_ = other.is_updated_;
  update_cb_ = other.update_cb_;

  decode_success_cnt_ = other.decode_success_cnt_;
  decode_fail_cnt_ = other.decode_fail_cnt_;
}

DT7 &DT7::operator=(DT7 &&other)
{
  if (this != &other) {
    rc_lv_ = other.rc_lv_;
    rc_lh_ = other.rc_lh_;
    rc_rv_ = other.rc_rv_;
    rc_rh_ = other.rc_rh_;
    rc_wheel_ = other.rc_wheel_;

    rc_l_switch_ = other.rc_l_switch_;
    rc_r_switch_ = other.rc_r_switch_;
    last_rc_l_switch_ = other.last_rc_l_switch_;
    last_rc_r_switch_ = other.last_rc_r_switch_;

    mouse_l_btn_ = other.mouse_l_btn_;
    mouse_r_btn_ = other.mouse_r_btn_;
    mouse_x_ = other.mouse_x_;
    mouse_y_ = other.mouse_y_;
    mouse_z_ = other.mouse_z_;

    key_ = other.key_;

    oc_ = std::move(other.oc_);

    is_updated_ = other.is_updated_;
    update_cb_ = other.update_cb_;

    decode_success_cnt_ = other.decode_success_cnt_;
    decode_fail_cnt_ = other.decode_fail_cnt_;
  }

  return *this;
}

bool DT7::decode(size_t len, const uint8_t *data, uint32_t rx_id)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(data != nullptr, "Error data pointer");
#pragma endregion

  if (len != kRcRxDataLen_) {
    decode_fail_cnt_++;
    return false;
  }

  uint16_t tmp;

  tmp = (data[0] | (data[1] << 8)) & 0x07FF;
  rc_rh_ = (tmp - kRcOffset) * kRcRatio;
  tmp = ((data[1] >> 3) | (data[2] << 5)) & 0x07FF;
  rc_rv_ = (tmp - kRcOffset) * kRcRatio;
  tmp = ((data[2] >> 6) | (data[3] << 2) | (data[4] << 10)) & 0x07FF;
  rc_lh_ = (tmp - kRcOffset) * kRcRatio;
  tmp = ((data[4] >> 1) | (data[5] << 7)) & 0x07FF;
  rc_lv_ = (tmp - kRcOffset) * kRcRatio;
  tmp = data[16] | (data[17] << 8);
  rc_wheel_ = (tmp - kRcOffset) * kRcRatio;

  tmp = ((data[5] >> 4) & 0x000C) >> 2;
  if (tmp < 1 || tmp > 3) {
    tmp = 0;
  }
  last_rc_l_switch_ = rc_l_switch_;
  rc_l_switch_ = SwitchState(tmp);

  tmp = (data[5] >> 4) & 0x0003;
  if (tmp < 1 || tmp > 3) {
    tmp = 0;
  }
  last_rc_r_switch_ = rc_r_switch_;
  rc_r_switch_ = SwitchState(tmp);

  mouse_x_ = data[6] | (data[7] << 8);
  mouse_y_ = data[8] | (data[9] << 8);
  mouse_z_ = data[10] | (data[11] << 8);
  mouse_l_btn_ = data[12];
  mouse_r_btn_ = data[13];

  key_.data = data[14] | (data[15] << 8);

  oc_.update();

  decode_success_cnt_++;
  is_updated_ = true;
  if (update_cb_) {
    update_cb_();
  }
  return true;
}
/* Private function definitions ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace remote_control
}  // namespace hello_world
