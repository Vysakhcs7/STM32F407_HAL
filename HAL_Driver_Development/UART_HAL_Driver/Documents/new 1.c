#include<uint32_t.h>

typedef struct
{
  __IO uint32_t SR;         !< USART Status register,                   Address offset: 0x00 
  __IO uint32_t DR;         !< USART Data register,                     Address offset: 0x04 
  __IO uint32_t BRR;        !< USART Baud rate register,                Address offset: 0x08 
  __IO uint32_t CR1;        !< USART Control register 1,                Address offset: 0x0C 
  __IO uint32_t CR2;        !< USART Control register 2,                Address offset: 0x10 
  __IO uint32_t CR3;        !< USART Control register 3,                Address offset: 0x14 
  __IO uint32_t GTPR;       !< USART Guard time and prescaler register, Address offset: 0x18 
} USART_TypeDef;



typedef struct
{
  uint32_t BaudRate;                  /*!< This member configures the UART communication baud rate.
                                           The baud rate is computed using the following formula:
                                           - IntegerDivider = ((PCLKx) / (8 * (OVR8+1) * (huart->Init.BaudRate)))
                                           - FractionalDivider = ((IntegerDivider - ((uint32_t) IntegerDivider)) * 8 * (OVR8+1)) + 0.5
                                           Where OVR8 is the "oversampling by 8 mode" configuration bit in the CR1 register. */

  uint32_t WordLength;                /*!< Specifies the number of data bits transmitted or received in a frame.
                                           This parameter can be a value of @ref UART_Word_Length */

  uint32_t StopBits;                  /*!< Specifies the number of stop bits transmitted.
                                           This parameter can be a value of @ref UART_Stop_Bits */

  uint32_t Parity;                    /*!< Specifies the parity mode.
                                           This parameter can be a value of @ref UART_Parity
                                           @note When parity is enabled, the computed parity is inserted
                                                 at the MSB position of the transmitted data (9th bit when
                                                 the word length is set to 9 data bits; 8th bit when the
                                                 word length is set to 8 data bits). */

  uint32_t Mode;                      /*!< Specifies whether the Receive or Transmit mode is enabled or disabled.
                                           This parameter can be a value of @ref UART_Mode */

  uint32_t HwFlowCtl;                 /*!< Specifies whether the hardware flow control mode is enabled or disabled.
                                           This parameter can be a value of @ref UART_Hardware_Flow_Control */

  uint32_t OverSampling;              /*!< Specifies whether the Over sampling 8 is enabled or disabled, to achieve higher speed (up to fPCLK/8).
                                           This parameter can be a value of @ref UART_Over_Sampling */
} UART_InitTypeDef;



typedef struct __UART_HandleTypeDef
{
  USART_TypeDef                 *Instance;        !< UART registers base address        

  UART_InitTypeDef              Init;             !< UART communication parameters      

  uint8_t                       *pTxBuffPtr;      !< Pointer to UART Tx transfer Buffer 

  uint16_t                      TxXferSize;       !< UART Tx Transfer size              

  __IO uint16_t                 TxXferCount;      !< UART Tx Transfer Counter           

  uint8_t                       *pRxBuffPtr;      !< Pointer to UART Rx transfer Buffer 

  uint16_t                      RxXferSize;       !< UART Rx Transfer size              

  __IO uint16_t                 RxXferCount;      !< UART Rx Transfer Counter           

  __IO HAL_UART_RxTypeTypeDef ReceptionType;      !< Type of ongoing reception          

  DMA_HandleTypeDef             *hdmatx;          !< UART Tx DMA Handle parameters      

  DMA_HandleTypeDef             *hdmarx;          !< UART Rx DMA Handle parameters      

  HAL_LockTypeDef               Lock;             !< Locking object                     

  __IO HAL_UART_StateTypeDef    gState;           !< UART state information related to global Handle management
                                                       and also related to Tx operations.
                                                       This parameter can be a value of @ref HAL_UART_StateTypeDef 

  __IO HAL_UART_StateTypeDef    RxState;          !< UART state information related to Rx operations.
                                                       This parameter can be a value of @ref HAL_UART_StateTypeDef 

  __IO uint32_t                 ErrorCode;        !< UART Error code                    

} UART_HandleTypeDef;


UART_HandleTypeDef huart2;

int main(void)
{
	huart2.instance
	
}
