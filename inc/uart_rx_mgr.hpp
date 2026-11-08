/**
 *******************************************************************************
 * @file      : uart_rx_mgr.hpp
 * @brief     : UART 接收管理器
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2024-07-10      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *  1. 请在 STM32CubeMX 中开启 UART 的 DMA 发送（Circular），开启串口全局中断，同时
 *  尽可能保证 DMA 接收中断的 Preemption Priority 和 Sub Priority 值低于串口全局中断
 *  的 Preemption Priority 和 Sub Priority 值（即优先级更高），具体硬件配置详见 Wiki
 *  2. 由于内部使用了硬件句柄，因此如果计划将实例作为全局变量时（全局变量初始化时对应的
 *  硬件句柄可能会还未初始化完毕），建议采取一下方法：
 *    1）声明指针，后续通过 `new` 的方式进行初始化
 *    2）声明指针，后续通过返回函数（CreateXXXIns）中的静态变量（因为该变量只有在第一
 *    次调用该函数时才会运行初始化程序）进行初始化
 *    3）使用无参构造函数，后续调用 `init` 方法进行初始化
 *    4）使用无参构造函数，后续使用拷贝赋值函数或是移动赋值函数进行初始化
 *  3. 若不注册数据处理函数，则只有 ID 为 0 的接收器会收到数据
 *  4. H7 系列的 DMA 有工作区域要求，详见 Wiki 上的配置方法
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_BSP_COMM_UART_RX_MGR_HPP_
#define HW_COMPONENTS_BSP_COMM_UART_RX_MGR_HPP_

/* Includes ------------------------------------------------------------------*/
#include "stm32_hal.hpp"

/* 开启 UART 与 DMA 才允许编译 */
#if defined(HAL_UART_MODULE_ENABLED) && defined(HAL_DMA_MODULE_ENABLED)

#include "rx_mgr.hpp"
#include "system.hpp"

namespace hello_world
{
namespace comm
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

HW_OPTIMIZE_O2_START

enum UartRxStatus {
  kUartRxStatusOk = 0,                   ///< 正常
  kUartRxStatusRecvStartErr = 1 << 0,    ///< 接收开启错误
  kUartRxStatusProcessingData = 1 << 1,  ///< 正在处理数据
  kUartRxStatusDataOverflow = 1 << 2,    ///< 数据溢出
};

enum class UartRxEofType {
  kIdle = 0,    ///< 以空闲帧作为一帧的结束
  kManual = 1,  ///< 自行判断一帧的结束
};

class UartRxMgr : public RxMgr
{
 public:
  typedef UartRxStatus Status;
  typedef UartRxEofType EofType;

  /**
   * @brief       处理数据的函数指针，由接收管理器调用
   * @param        rx_data: 接收到的数据
   * @param        rx_data_len: 接收到的数据长度
   * @param        processed_data_len: 传入 processed_data 的长度，返回处理后数据
   *               的长度
   * @param        processed_data: 返回处理后的数据，可暂存尚未处理完的数据
   * @param        rx_data_processed_len: 返回已处理的数据长度
   * @param        id: 返回处理后数据的对应 ID
   * @retval       处理得到一帧完整数据则返回 true，否则返回 false
   * @note        接收管理器会传入接收到的数据，该函数将返回处理后的一帧数据，同时告知
   *              已处理的数据长度，若为将传入的数据完全处理完（rx_data_len !=
   *              rx_data_processed_len），接收管理器会进一步调用该函数，传入剩余数
   *              据，直至处理完所有数据
   */
  typedef bool (*pProcessData)(
      const uint8_t *rx_data, size_t rx_data_len, size_t &processed_data_len,
      uint8_t *processed_data, size_t &rx_data_processed_len, uint32_t &id);

  /**
   * @brief       默认构造函数
   * @retval       None
   * @note        使用该方式实例化对象时，需要在外部调用 init() 函数进行初始化
   */
  UartRxMgr(void) = default;
  /**
   * @brief       构造函数
   * @param        huart: UART 句柄
   * @param        eof_type: 结束类型，可选值为：
   *   @arg        EofType::kIdle: 空闲帧作为一帧结束
   *   @arg        EofType::kManual: 自行判断一帧的结束
   * @param        buf_len: 接收缓存长度，当 eof_type 为 EofType::kIdle 时，
   *               buf_len 需大于实际接受数据的最大长度（建议大一），当 eof_type 为
   *               EofType::kManual 时，buf_len 不宜过大，其长度保证使两次中断触发的
   *               间隔内数据能被正常处理完毕即可（如 32）
   * @param        max_process_data_len: 处理后一帧数据的最大长度
   * @retval       None
   * @note        None
   */
  UartRxMgr(UART_HandleTypeDef *huart, EofType eof_type, size_t buf_len,
            size_t max_process_data_len);
  /**
   * @brief       构造函数
   * @param        huart: UART 句柄
   * @param        eof_type: 结束类型，可选值为：
   *   @arg        EofType::kIdle: 空闲帧作为一帧结束
   *   @arg        EofType::kManual: 自行判断一帧的结束
   * @param        rx_buf: 接收缓存，使用期间不可释放
   * @param        buf_len: 接收缓存长度，当 eof_type 为 EofType::kIdle 时，
   *               buf_len 需大于实际接受数据的最大长度（建议大一），当 eof_type 为
   *               EofType::kManual 时，buf_len 不宜过大，其长度保证使两次中断触发的
   *               间隔内数据能被正常处理完毕即可（如 32）
   * @param        max_process_data_len: 处理后一帧数据的最大长度
   * @retval       None
   * @note        对于 DMA 由工作区域要求的 H7 系列，需要使用此构造函数
   */
  UartRxMgr(UART_HandleTypeDef *huart, EofType eof_type, uint8_t *rx_buf,
            size_t buf_len, size_t max_process_data_len);
  UartRxMgr(const UartRxMgr &) = default;
  UartRxMgr &operator=(const UartRxMgr &other);
  UartRxMgr(UartRxMgr &&other);
  UartRxMgr &operator=(UartRxMgr &&other);

  virtual ~UartRxMgr(void);

  /* 重载方法 */

  /**
   * @brief       开始串口接收
   * @retval       None
   * @note        只需调用一次
   */
  virtual void startReceive(void) override;

  /**
   * @brief       停止串口接收
   * @retval       None
   * @note        None
   */
  virtual void stopReceive(void) override;

  /* 配置方法 */

  /**
   * @brief       初始化函数，使用默认构造函数后请务必调用此函数
   * @param        huart: UART 句柄
   * @param        eof_type: 结束类型，可选值为：
   *   @arg        EofType::kIdle: 空闲帧作为一帧结束
   *   @arg        EofType::kManual: 自行判断一帧的结束
   * @param        buf_len: 接收缓存长度，当 eof_type 为 EofType::kIdle 时，
   *               buf_len 需大于实际接受数据的最大长度（建议大一），当 eof_type 为
   *               EofType::kManual 时，buf_len 不宜过大，其长度保证使两次中断触发的
   *               间隔内数据能被正常处理完毕即可（如 32）
   * @param        max_process_data_len: 处理后一帧数据的最大长度
   * @retval       None
   * @note        None
   */
  void init(UART_HandleTypeDef *huart, EofType eof_type, size_t buf_len,
            size_t max_process_data_len);

  /**
   * @brief       初始化函数，使用默认构造函数后请务必调用此函数
   * @param        huart: UART 句柄
   * @param        eof_type: 结束类型，可选值为：
   *   @arg        EofType::kIdle: 空闲帧作为一帧结束
   *   @arg        EofType::kManual: 自行判断一帧的结束
   * @param        rx_buf: 接收缓存，使用期间不可释放
   * @param        buf_len: 接收缓存长度，当 eof_type 为 EofType::kIdle 时，
   *               buf_len 需大于实际接受数据的最大长度（建议大一），当 eof_type 为
   *               EofType::kManual 时，buf_len 不宜过大，其长度保证使两次中断触发的
   *               间隔内数据能被正常处理完毕即可（如 32）
   * @param        max_process_data_len: 处理后一帧数据的最大长度
   * @retval       None
   * @note        对于 DMA 由工作区域要求的 H7 系列，需要使用此初始化函数
   */
  void init(UART_HandleTypeDef *huart, EofType eof_type, uint8_t *rx_buf,
            size_t buf_len, size_t max_process_data_len);

  /* 功能性方法 */

  /**
   * @brief       数据处理函数函数
   * @param        func: 数据处理函数
   * @retval       None
   * @note        不注册则所有数据 ID 为 0，直接返回原始数据
   */
  void registerProcessDataFunc(pProcessData func);

  void resetProcessDataFunc(void) { process_data_func_ = ProcessData; }

  /* 回调函数 */

  /**
   * @brief       串口接收事件回调
   * @param        huart: UART 句柄
   * @param        size: 当前数据位置
   * @retval       None
   * @note        请将次函数放置到对应的 HAL_UARTEx_RxEventCallback 函数中，用于在
   *              接收到数据时进行处理
   */
  void rxEventCallback(UART_HandleTypeDef *huart, uint16_t size);

  /**
   * @brief       串口接收错误回调
   * @param        huart: UART 句柄
   * @retval       None
   * @note        请将次函数放置到对应的 HAL_UART_ErrorCallback 函数中，用于在因开启
   *              接收时因某些错误导致接收开启失败时重新开启接收，以确保接收能被开启
   */
  void errorCallback(UART_HandleTypeDef *huart);

  /* 数据修改与获取 */

  Status status(void) const { return status_; }

 private:
  /* 功能性方法 */

  /**
   * @brief       获取实际接收到的数据
   * @param        size: 当前数据位置
   * @param        data_len: 返回数据长度
   * @retval       是否处理完成
   * @note        None
   */
  bool getData(uint16_t size, size_t &data_len);

  /**
   * @brief       默认数据处理函数
   * @param        rx_data: 接收到的数据
   * @param        rx_data_len: 接收到的数据长度
   * @param        processed_data_len: 传入 processed_data 的长度，返回处理后数据
   *               的长度
   * @param        processed_data: 返回处理后的数据，可暂存尚未处理完的数据
   * @param        rx_data_processed_len: 返回已处理的数据长度
   * @param        id: 返回处理后数据的对应 ID
   * @retval       处理完成返回 true，否则返回 false
   * @note        默认处理函数，直接返回原始数据
   */
  static bool ProcessData(
      const uint8_t *rx_data, size_t rx_data_len, size_t &processed_data_len,
      uint8_t *processed_data, size_t &rx_data_processed_len, uint32_t &id);

  /* 硬件相关 */

  UART_HandleTypeDef *huart_ = nullptr;

  /* 接收状态相关 */

  Status status_ = kUartRxStatusOk;
  EofType eof_type_ = EofType::kIdle;
  bool start_receive_ = false;
  uint32_t decode_success_cnt_ = 0;
  uint32_t receive_cnt_ = 0;
  HAL_UART_RxEventTypeTypeDef last_rx_event_type_ = HAL_UART_RXEVENT_TC;

  /* 接收数据相关 */

  size_t buf_len_ = 0;
  uint8_t *rx_buf_ = nullptr;   ///< 接收缓冲区
  uint8_t *rx_data_ = nullptr;  ///< 实际接受到的数据
  size_t data_idx_ = 0;         ///< 接收到的数据索引
  bool is_alloc_buf_ = false;   ///< 缓冲区是否为内部分配

  size_t max_process_data_len_ = 0;    ///< 最大处理数据长度
  uint8_t *processed_data_ = nullptr;  ///< 处理后的数据
  size_t buf_handled_idx_ = 0;         ///< 已处理的缓冲区数据索引

  pProcessData process_data_func_ = ProcessData;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace comm
}  // namespace hello_world

#endif /* HAL_UART_MODULE_ENABLED && HAL_DMA_MODULE_ENABLED */

#endif /* HW_COMPONENTS_BSP_COMM_UART_RX_MGR_HPP_ */
