/******************************************************************************
 * @file    uart.h
 * @author  King
 * @version V1.00
 * @date    20-May-2020
 * @brief   ......
 ******************************************************************************
 * @attention
 * 
 * THE EXISTING FIRMWARE IS ONLY FOR REFERENCE, WHICH IS DESIGNED TO PROVIDE
 * CUSTOMERS WITH CODING INFORMATION ABOUT THEIR PRODUCTS SO THEY CAN SAVE
 * TIME. THEREFORE, MINDMOTION SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES ABOUT ANY CLAIMS ARISING OUT OF THE CONTENT OF SUCH
 * HARDWARE AND/OR THE USE OF THE CODING INFORMATION CONTAINED HEREIN IN
 * CONNECTION WITH PRODUCTS MADE BY CUSTOMERS.
 * <H2><CENTER>&COPY; COPYRIGHT 2020 MINDMOTION </CENTER></H2>
******************************************************************************/


/* Define to prevent recursive inclusion ------------------------------------*/
#ifndef __UART_H__
#define __UART_H__


#ifdef __cplusplus
extern "C" {
#endif


#undef  EXTERN


#ifdef  __UART_C__
#define EXTERN
#else
#define EXTERN extern
#endif


/* Includes -----------------------------------------------------------------*/
#include "config.h"


/* Exported constants -------------------------------------------------------*/
#define UART_BUFFER_SIZE    100


/* Exported types -----------------------------------------------------------*/
typedef struct
{
    uint8_t  Buffer[UART_BUFFER_SIZE];
    uint16_t Length;
    uint16_t CurrentCount;
    uint8_t  CompleteFlag;
} UART_RxTx_TypeDef;


/* Exported macro -----------------------------------------------------------*/


/* Exported functions -------------------------------------------------------*/
EXTERN void UARTx_Configure(UART_TypeDef           *UARTx,
                            uint32_t                BaudRate,
                            UART_WordLength_TypeDef WordLength,
                            UART_Stop_Bits_TypeDef  StopBits,
                            UART_Parity_TypeDef     Parity);

EXTERN void UARTx_RxData(UART_TypeDef *UARTx, uint8_t Len);
EXTERN void UARTx_TxData(UART_TypeDef *UARTx, uint8_t *pBuf, uint8_t Len);


#ifdef __cplusplus
}
#endif


#endif


/******************* (C) COPYRIGHT 2020 ************************END OF FILE***/

