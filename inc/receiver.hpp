/**
 *******************************************************************************
 * @file      : receiver.hpp
 * @brief     : 用于统一接收端接口的虚基类
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2024-07-10      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *  需要通过通用通信驱动接收的类需要继承该接口类
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_BSP_COMM_RECEIVER_HPP_
#define HW_COMPONENTS_BSP_COMM_RECEIVER_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstddef>
#include <cstdint>

#include "allocator.hpp"
#include "list.hpp"

namespace hello_world
{
namespace comm
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/**
 * @brief       接收端接口
 * @note        需要通过通用通信驱动接收的类需要继承该接口类
 */
class Receiver : virtual public MemMgr
{
 public:
  typedef void (*pUpdateCallback)(void);
  typedef tools::list<uint32_t> RxIds;

  Receiver(void) = default;
  Receiver(const Receiver &) = default;
  Receiver &operator=(const Receiver &) = default;
  Receiver(Receiver &&) = default;
  Receiver &operator=(Receiver &&) = default;

  virtual ~Receiver(void) = default;

  /**
   * @brief       获取接收端当前 ID
   * @param        None
   * @retval       接收端当前 ID
   * @note        接收管理区会根据 ID 进行管理，相同 ID 的接收端会共同解析一条报文
   */
  virtual uint32_t rxId(void) const = 0;

  /**
   * @brief       获取接收端所有 ID
   * @param        None
   * @retval       接收端所有 ID
   * @note        接收管理区会根据 ID 进行管理，相同 ID 的接收端会共同解析一条报文
   */
  virtual const RxIds &rxIds(void) const = 0;

  /**
   * @brief       解码，由接收管理器调用
   * @param        len: 数据长度
   * @param        data: 数据指针
   * @param        rx_id: 接收 ID
   * @retval       解码成功返回 true，否则返回 false
   * @note        建议在内部统计解码成功与失败的次数，通过二者之和可以判断是否有收到对
   *              应 ID 的数据，便于调试时进行问题定位
   */
  virtual bool decode(size_t len, const uint8_t *data, uint32_t rx_id) = 0;

  /**
   * @brief       是否有更新数据
   * @retval       有更新数据返回 true，否则返回 false
   * @note        建议定义一个标志位，并在 decode 中解码成功后置位
   */
  virtual bool isUpdate(void) const = 0;

  /**
   * @brief       清除更新标志
   * @retval       None
   * @note        None
   */
  virtual void clearUpdateFlag(void) = 0;

  /**
   * @brief       注册更新回调函数
   * @param        cb: 回调函数指针，在 decode 函数解码成功后被调用，不使用时传入
   *               nullptr
   * @retval       None
   * @note        建议定义一个 pUpdateCallback 的函数指针，并在 decode 中解码成功后
   *              调用（函数指针非空时调用），以此实现更新回调
   */
  virtual void registerUpdateCallback(pUpdateCallback cb) = 0;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace comm
}  // namespace hello_world

#endif /* HW_COMPONENTS_BSP_COMM_RECEIVER_HPP_ */
