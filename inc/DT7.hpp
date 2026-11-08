/**
 *******************************************************************************
 * @file      : DT7.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_REMOTE_CONTROL_DT7_HPP_
#define HW_COMPONENTS_DEVICES_REMOTE_CONTROL_DT7_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

#include "offline_checker.hpp"
#include "receiver.hpp"
#include "remote_control_base.hpp"
#include "system.hpp"

namespace hello_world
{
namespace remote_control
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

HW_OPTIMIZE_O2_START
class DT7 : public comm::Receiver
{
 public:
  /**
   * @brief       默认构造函数
   * @retval       None
   * @note        使用该方式实例化对象时，需要在外部调用 init() 函数进行初始化
   */
  DT7(void) = default;
  /**
   * @brief       构造函数
   * @param        offline_tick_thres: 离线阈值，单位：ms
   * @retval       None
   * @note        None
   */
  DT7(uint32_t offline_tick_thres) : oc_(offline_tick_thres) {}
  DT7(const DT7 &) = default;
  DT7 &operator=(const DT7 &other);
  DT7(DT7 &&other);
  DT7 &operator=(DT7 &&other);

  virtual ~DT7(void) = default;

  /* 重载方法 */

  virtual uint32_t rxId(void) const override { return 0u; }

  virtual const RxIds &rxIds(void) const override { return rx_ids_; }
  /**
   * @brief       将接收到的数据解包
   * @param        len: 数据长度
   * @param        data: 接收到的数据
   * @param        rx_id: 接收 ID
   * @retval       解包成功返回 true，否则返回 false
   * @note        None
   */
  virtual bool decode(size_t len, const uint8_t *data, uint32_t rx_id) override;

  virtual bool isUpdate(void) const override { return is_updated_; }

  virtual void clearUpdateFlag(void) override { is_updated_ = false; }

  /**
   * @brief       注册更新回调函数
   * @param        cb: 回调函数指针，在 decode 函数解码成功后被调用，不使用时传入
   *               nullptr
   * @retval       None
   * @note        如注测封装好的看门狗刷新函数
   *
   */
  virtual void registerUpdateCallback(pUpdateCallback cb) override
  {
    update_cb_ = cb;
  }

  /* 配置方法 */

  /**
   * @brief       初始化，使用默认构造函数后请务必调用此函数
   * @param        offline_tick_thres: 离线阈值，单位：ms
   * @retval       None
   * @note        None
   */
  void init(uint32_t offline_tick_thres) { oc_.init(offline_tick_thres); }

  /* 数据修改与获取 */

  float rc_lv(void) const { return rc_lv_; }
  float rc_lh(void) const { return rc_lh_; }
  float rc_rv(void) const { return rc_rv_; }
  float rc_rh(void) const { return rc_rh_; }
  float rc_wheel(void) const { return rc_wheel_; }

  SwitchState rc_l_switch(void) const { return rc_l_switch_; }
  SwitchState rc_r_switch(void) const { return rc_r_switch_; }
  SwitchState last_rc_l_switch(void) const { return last_rc_l_switch_; }
  SwitchState last_rc_r_switch(void) const { return last_rc_r_switch_; }

  int16_t mouse_x(void) const { return mouse_x_; }
  int16_t mouse_y(void) const { return mouse_y_; }
  int16_t mouse_z(void) const { return mouse_z_; }

  bool mouse_l_btn(bool reset = false)
  {
    bool tmp = mouse_l_btn_;
    if (reset) {
      mouse_l_btn_ = false;
    }
    return tmp;
  }
  bool mouse_r_btn(bool reset = false)
  {
    bool tmp = mouse_r_btn_;
    if (reset) {
      mouse_r_btn_ = false;
    }
    return tmp;
  }
  bool key_W(bool reset = false)
  {
    bool tmp = key_.W;
    if (reset) {
      key_.W = false;
    }
    return tmp;
  }
  bool key_S(bool reset = false)
  {
    bool tmp = key_.S;
    if (reset) {
      key_.S = false;
    }
    return tmp;
  }
  bool key_A(bool reset = false)
  {
    bool tmp = key_.A;
    if (reset) {
      key_.A = false;
    }
    return tmp;
  }
  bool key_D(bool reset = false)
  {
    bool tmp = key_.D;
    if (reset) {
      key_.D = false;
    }
    return tmp;
  }
  bool key_SHIFT(bool reset = false)
  {
    bool tmp = key_.SHIFT;
    if (reset) {
      key_.SHIFT = false;
    }
    return tmp;
  }
  bool key_CTRL(bool reset = false)
  {
    bool tmp = key_.CTRL;
    if (reset) {
      key_.CTRL = false;
    }
    return tmp;
  }
  bool key_Q(bool reset = false)
  {
    bool tmp = key_.Q;
    if (reset) {
      key_.Q = false;
    }
    return tmp;
  }
  bool key_E(bool reset = false)
  {
    bool tmp = key_.E;
    if (reset) {
      key_.E = false;
    }
    return tmp;
  }
  bool key_R(bool reset = false)
  {
    bool tmp = key_.R;
    if (reset) {
      key_.R = false;
    }
    return tmp;
  }
  bool key_F(bool reset = false)
  {
    bool tmp = key_.F;
    if (reset) {
      key_.F = false;
    }
    return tmp;
  }
  bool key_G(bool reset = false)
  {
    bool tmp = key_.G;
    if (reset) {
      key_.G = false;
    }
    return tmp;
  }
  bool key_Z(bool reset = false)
  {
    bool tmp = key_.Z;
    if (reset) {
      key_.Z = false;
    }
    return tmp;
  }
  bool key_X(bool reset = false)
  {
    bool tmp = key_.X;
    if (reset) {
      key_.X = false;
    }
    return tmp;
  }
  bool key_C(bool reset = false)
  {
    bool tmp = key_.C;
    if (reset) {
      key_.C = false;
    }
    return tmp;
  }
  bool key_V(bool reset = false)
  {
    bool tmp = key_.V;
    if (reset) {
      key_.V = false;
    }
    return tmp;
  }
  bool key_B(bool reset = false)
  {
    bool tmp = key_.B;
    if (reset) {
      key_.B = false;
    }
    return tmp;
  }

  bool isKeyboardPressed(void) const { return key_.data != 0u; }
  bool isMousePressed(void) const { return mouse_l_btn_ || mouse_r_btn_; }
  bool isMouseMoved(void) const
  {
    return mouse_x_ != 0 || mouse_y_ != 0 || mouse_z_ != 0;
  }
  bool isUsingKeyboardMouse(void) const
  {
    return isKeyboardPressed() || isMousePressed() || isMouseMoved();
  }
  bool isRcSwitchChanged(void) const
  {
    return (rc_l_switch_ != last_rc_l_switch_ &&
            last_rc_l_switch_ != SwitchState::kErr) ||
           (rc_r_switch_ != last_rc_r_switch_ &&
            last_rc_r_switch_ != SwitchState::kErr);
  }
  bool isRcMoved(void) const
  {
    return (rc_lv_ != 0) || (rc_lh_ != 0) || (rc_rv_ != 0) ||
           (rc_rh_ != 0) || (rc_wheel_ != 0);
  }
  bool isUsingRc(void) const { return isRcSwitchChanged() || isRcMoved(); }

  bool isOffline(void) { return oc_.isOffline(); }

  static const uint8_t kRcRxDataLen_ = 18u;  ///* 遥控器接收数据长度

 private:
  /* 遥控器数据 */

  float rc_lv_ = 0.0f;     ///* 遥控器左摇杆竖直值，[-1, 1]
  float rc_lh_ = 0.0f;     ///* 遥控器左摇杆水平值，[-1, 1]
  float rc_rv_ = 0.0f;     ///* 遥控器右摇杆竖直值，[-1, 1]
  float rc_rh_ = 0.0f;     ///* 遥控器右摇杆水平值，[-1, 1]
  float rc_wheel_ = 0.0f;  ///* 遥控器拨轮值，[-1, 1]

  SwitchState rc_l_switch_ = SwitchState::kErr;  ///* 遥控器左拨杆值
  SwitchState rc_r_switch_ = SwitchState::kErr;  ///* 遥控器右拨杆值
  /** 上一次接收到的遥控器左拨杆值 */
  SwitchState last_rc_l_switch_ = SwitchState::kErr;
  /** 上一次接收到的遥控器右拨杆值 */
  SwitchState last_rc_r_switch_ = SwitchState::kErr;

  /* 键鼠数据 */

  bool mouse_l_btn_ = false;  ///* 鼠标左键是否按下
  bool mouse_r_btn_ = false;  ///* 鼠标右键是否按下
  int16_t mouse_x_ = 0u;      ///* 鼠标x轴数值
  int16_t mouse_y_ = 0u;      ///* 鼠标y轴数值
  int16_t mouse_z_ = 0u;      ///* 鼠标z轴数值

  union {
    uint16_t data = 0u;
    struct {
      uint16_t W : 1;
      uint16_t S : 1;
      uint16_t A : 1;
      uint16_t D : 1;
      uint16_t SHIFT : 1;
      uint16_t CTRL : 1;
      uint16_t Q : 1;
      uint16_t E : 1;
      uint16_t R : 1;
      uint16_t F : 1;
      uint16_t G : 1;
      uint16_t Z : 1;
      uint16_t X : 1;
      uint16_t C : 1;
      uint16_t V : 1;
      uint16_t B : 1;
    };
  } key_;  ///* 键盘按键

  OfflineChecker oc_ = OfflineChecker(100u);

  bool is_updated_ = false;  ///* 是否有新数据更新
  pUpdateCallback update_cb_ = nullptr;
  RxIds rx_ids_ = {0u};      ///* 接收端 ID 列表

  /** 接收状态统计 */

  uint32_t decode_success_cnt_ = 0u;  ///* 解码成功次数
  uint32_t decode_fail_cnt_ = 0u;     ///* 解码失败次数
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace remote_control
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_REMOTE_CONTROL_DT7_HPP_ */
