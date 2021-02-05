/******************************************************************************
 * @file    uart.c
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
#define __UART_C__


/* Includes -----------------------------------------------------------------*/
#include "uart.h"


/* Private typedef ----------------------------------------------------------*/
/* Private define -----------------------------------------------------------*/
/* Private macro ------------------------------------------------------------*/
#define UART_REC_LEN  			200  	//定义最大接收字节数 200
#define    UART_MAX_LEN    150            //最大接收长度 150个字节
u16 UART_RX_BUF[UART_REC_LEN];          //接收缓冲,最大USART_REC_LEN个字节.
u16 UART_RX_STA=0;                      //接收状态标记	 
u8  UART1_RecvBuff[UART_MAX_LEN] = {0};  //接收buff
u16 RecvBuffLen = 0;          						 //接收长度


/* Private variables --------------------------------------------------------*/
UART_RxTx_TypeDef UART1_TxStruct;
UART_RxTx_TypeDef UART1_RxStruct;

UART_RxTx_TypeDef UART2_TxStruct;
UART_RxTx_TypeDef UART2_RxStruct;


/* Private function prototypes ----------------------------------------------*/
/* Private functions --------------------------------------------------------*/


/* Exported variables -------------------------------------------------------*/
/* Exported function prototypes ---------------------------------------------*/


/**
  * @brief  Config UART1 or UART2
  * @param  None
  * @retval None
  */
void UARTx_Configure(UART_TypeDef           *UARTx,
                     uint32_t                BaudRate,
                     UART_WordLength_TypeDef WordLength,
                     UART_Stop_Bits_TypeDef  StopBits,
                     UART_Parity_TypeDef     Parity)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    UART_InitTypeDef UART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    if(UARTx == UART1)
    {
        /* Enable UART1 clock */
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_UART1, ENABLE);

        /* UART Configuration ad follow */
        UART_StructInit(&UART_InitStructure);
        UART_InitStructure.UART_BaudRate            = BaudRate;
        UART_InitStructure.UART_WordLength          = WordLength;
        UART_InitStructure.UART_StopBits            = StopBits;
        UART_InitStructure.UART_Parity              = Parity;
        UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_None;
        UART_InitStructure.UART_Mode                = UART_Mode_Rx | UART_Mode_Tx;
        UART_Init(UART1, &UART_InitStructure);

        UART_ITConfig(UART1, UART_IER_RX, ENABLE);
        UART_ITConfig(UART1, UART_IER_RXIDLE, ENABLE);

        /* Enable UART1 */
        UART_Cmd(UART1, ENABLE);

        /* Enable GPIO and SYSCFG clock */
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,    ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

        /* Enable PA9 & PA10 alternate function */
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,  GPIO_AF_1);
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);

        /* Configure UART Tx as alternate function push-pull */
        GPIO_StructInit(&GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
        GPIO_Init(GPIOA, &GPIO_InitStructure);

         /* Configure UART Rx as input floating */
        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
        GPIO_Init(GPIOA, &GPIO_InitStructure);

        /* Config UART1 Interrupt */
        NVIC_InitStructure.NVIC_IRQChannel = UART1_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPriority  = 1;
        NVIC_InitStructure.NVIC_IRQChannelCmd  = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    }
    else if(UARTx == UART2)
    {
        /* Enable UART2 clock */
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART2, ENABLE);

        /* UART Configuration ad follow */
        UART_StructInit(&UART_InitStructure);
        UART_InitStructure.UART_BaudRate            = BaudRate;
        UART_InitStructure.UART_WordLength          = WordLength;
        UART_InitStructure.UART_StopBits            = StopBits;
        UART_InitStructure.UART_Parity              = Parity;
        UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_None;
        UART_InitStructure.UART_Mode                = UART_Mode_Rx | UART_Mode_Tx;
        UART_Init(UART2, &UART_InitStructure);

        /* Enable UART2 */
        UART_Cmd(UART2, ENABLE);

        /* Enable GPIO and SYSCFG clock */
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,    ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

        /* Enable PA2 & PA3 alternate function */
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource2,  GPIO_AF_1);
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource3,  GPIO_AF_1);

        /* Configure UART Tx as alternate function push-pull */
        GPIO_StructInit(&GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
        GPIO_Init(GPIOA, &GPIO_InitStructure);

         /* Configure UART Rx as input floating */
        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
        GPIO_Init(GPIOA, &GPIO_InitStructure);

        /* Config UART2 Interrupt */
        NVIC_InitStructure.NVIC_IRQChannel = UART2_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPriority  = 1;
        NVIC_InitStructure.NVIC_IRQChannelCmd  = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    }
    else
    {
    }
}
/******************************************************************************
 * @brief       
 * @param       
 * @retval      
 * @attention   
******************************************************************************/
void uart1_SendBuff(u8 * buf, u16 len)
{
	while(len--)
	{
		while(UART_GetFlagStatus(UART1,UART_FLAG_TXEPT)==RESET); //等待发送结束
		UART_SendData(UART1, *buf++);
	}
	while(UART_GetFlagStatus(UART1,UART_FLAG_TXEPT)==RESET);
}


void UART1_IRQHandler(void)  //串口1中断服务程序
{
	if(UART_GetITStatus(UART1,UART_IER_RX) != RESET) //接收中断触发
	{		
			if(RecvBuffLen < UART_MAX_LEN)
			{
				 //读DR把数据一个字节一个字节读出，运行这个会自动清除中断标志位，	  
			   UART1_RecvBuff[RecvBuffLen++] = UART_ReceiveData(UART1);  
			}
			else 	 //一次性接收大于150个字节，数组从头开始接收
			{
			   UART_ReceiveData(UART1);              //清除空闲标志
			   RecvBuffLen = 0;
			}
	}	
	
	if(UART_GetITStatus(UART1, UART_IER_RXIDLE) != RESET) //空闲中断触发
	{	
		 UART_ReceiveData(UART1);                //清除空闲标志		
     if(RecvBuffLen != 0)                      //表示接收到数据
		 {
		    uart1_SendBuff(UART1_RecvBuff,RecvBuffLen); //把收到的数据发出来
		    RecvBuffLen = 0;	
		 }		      
	} 
}


/******************************************************************************
 * @brief       
 * @param       
 * @retval      
 * @attention   
******************************************************************************/
void UART2_IRQHandler(void)
{
    if(UART_GetITStatus(UART2, UART_IT_RXIEN) != RESET)
    {
        /* Receive UART2 Data */
        uint8_t data = UART_ReceiveData(UART2);

        /* Clear UART2 Rx Interrupt Flag */
        UART_ClearITPendingBit(UART2, UART_IT_RXIEN);

        if(UART2_RxStruct.CompleteFlag == 0)
        {
            UART2_RxStruct.Buffer[UART2_RxStruct.CurrentCount++] = data;

            if(UART2_RxStruct.CurrentCount == UART2_RxStruct.Length)
            {
                UART2_RxStruct.CompleteFlag = 1;

                /* Disable UART2 Rx Interrupt */
                UART_ITConfig(UART2, UART_IT_RXIEN, DISABLE);
            }
        }
    }

    if(UART_GetITStatus(UART2, UART_IT_TXIEN) != RESET)
    {
        /* Clear UART2 Tx Interrupt Flag */
        UART_ClearITPendingBit(UART2, UART_IT_TXIEN);

        if(UART2_TxStruct.CompleteFlag == 0)
        {
            /* Transmit Data */
            UART_SendData(UART2, UART2_TxStruct.Buffer[UART2_TxStruct.CurrentCount++]);

            if(UART2_TxStruct.CurrentCount == UART2_TxStruct.Length)
            {
                UART2_TxStruct.CompleteFlag = 1;

                /* Disable UART2 Tx Interrupt */
                UART_ITConfig(UART2, UART_IT_TXIEN, DISABLE);
            }
        }
    }
}


/******************************************************************************
 * @brief       
 * @param       
 * @retval      
 * @attention   
******************************************************************************/
void UARTx_RxData(UART_TypeDef *UARTx, uint8_t Len)
{
    uint8_t i = 0;

    if(Len != 0)
    {
        if(UARTx == UART1)
        {
            for(i = 0; i < Len; i++)
            {
                UART1_RxStruct.Buffer[i] = 0;
            }

            UART1_RxStruct.Length = Len;
            UART1_RxStruct.CurrentCount = 0;
            UART1_RxStruct.CompleteFlag = 0;

            /* Enable UART1 Rx Interrupt */
            UART_ITConfig(UARTx, UART_IT_RXIEN, ENABLE);
        }
        else
        {
            for(i = 0; i < Len; i++)
            {
                UART2_RxStruct.Buffer[i] = 0;
            }

            UART2_RxStruct.Length = Len;
            UART2_RxStruct.CurrentCount = 0;
            UART2_RxStruct.CompleteFlag = 0;

            /* Enable UART2 Rx Interrupt */
            UART_ITConfig(UARTx, UART_IT_RXIEN, ENABLE);
        }
    }
}


/**
  * @brief  
  * @param  None
  * @retval None
  */
void UARTx_TxData(UART_TypeDef *UARTx, uint8_t *pBuf, uint8_t Len)
{
    uint8_t i = 0;

    if(Len != 0)
    {
        if(UARTx == UART1)
        {
            for(i = 0; i < Len; i++)
            {
                UART1_TxStruct.Buffer[i] = pBuf[i];
            }

            UART1_TxStruct.Length = Len;
            UART1_TxStruct.CurrentCount = 0;
            UART1_TxStruct.CompleteFlag = 0;

            /* Enable UART1 Tx Interrupt */
            UART_ITConfig(UARTx, UART_IT_TXIEN, ENABLE);
        }
        else
        {
            for(i = 0; i < Len; i++)
            {
                UART2_TxStruct.Buffer[i] = pBuf[i];
            }

            UART2_TxStruct.Length = Len;
            UART2_TxStruct.CurrentCount = 0;
            UART2_TxStruct.CompleteFlag = 0;

            /* Enable UART1 Tx Interrupt */
            UART_ITConfig(UARTx, UART_IT_TXIEN, ENABLE);
        }
    }
}


/******************************************************************************
 * @brief       Retargets the C library printf function to the UART.
 * @param       
 * @retval      
 * @attention   
******************************************************************************/
int fputc(int ch, FILE *f)
{
    /* Send a character to the UART */
    UART_SendData(DEBUG_UART, (uint8_t)ch);

     /* Loop until the end of transmission */
    while(!UART_GetFlagStatus(DEBUG_UART, UART_FLAG_TXEPT));

    return ch;
}


/******************* (C) COPYRIGHT 2020 ************************END OF FILE***/

