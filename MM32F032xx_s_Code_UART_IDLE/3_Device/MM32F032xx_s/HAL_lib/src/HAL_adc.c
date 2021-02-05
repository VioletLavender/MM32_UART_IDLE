////////////////////////////////////////////////////////////////////////////////
/// @file     HAL_ADC.C
/// @author   AE TEAM
/// @brief    THIS FILE PROVIDES ALL THE ADC FIRMWARE FUNCTIONS.
////////////////////////////////////////////////////////////////////////////////
/// @attention
///
/// THE EXISTING FIRMWARE IS ONLY FOR REFERENCE, WHICH IS DESIGNED TO PROVIDE
/// CUSTOMERS WITH CODING INFORMATION ABOUT THEIR PRODUCTS SO THEY CAN SAVE
/// TIME. THEREFORE, MINDMOTION SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT OR
/// CONSEQUENTIAL DAMAGES ABOUT ANY CLAIMS ARISING OUT OF THE CONTENT OF SUCH
/// HARDWARE AND/OR THE USE OF THE CODING INFORMATION CONTAINED HEREIN IN
/// CONNECTION WITH PRODUCTS MADE BY CUSTOMERS.
///
/// <H2><CENTER>&COPY; COPYRIGHT MINDMOTION </CENTER></H2>
////////////////////////////////////////////////////////////////////////////////

// Define to prevent recursive inclusion
#define _HAL_ADC_C_

// Files includes
#include "hal_adc.h"
#include "hal_rcc.h"

////////////////////////////////////////////////////////////////////////////////
/// @addtogroup MM32_Hardware_Abstract_Layer
/// @{

////////////////////////////////////////////////////////////////////////////////
/// @addtogroup ADC_HAL
/// @{

////////////////////////////////////////////////////////////////////////////////
/// @addtogroup ADC_Exported_Functions
/// @{

////////////////////////////////////////////////////////////////////////////////
/// @brief  Deinitializes the ADCn peripheral registers to their default
///         reset values.
/// @param  ADCn: where n can be 1, 2  to select the ADC peripheral.
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
void ADC_DeInit(ADC_TypeDef* ADCn)
{
    switch (*(u32*)&ADCn) {
        case ADC1_BASE:
            exRCC_APB2PeriphReset(RCC_APB2ENR_ADC1);
            break;
        default:
            break;
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Initializes the ADCn peripheral according to the specified parameters
///         in the pInitStruct, Please use this function if you want to be
///         compatible with older versions of the library.
/// @param  ADCn: where n can be 1, 2  to select the ADC peripheral.
/// @param  pInitStruct: pointer to an ADC_InitTypeDef structure that contains
///         the configuration information for the specified ADC peripheral.
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
void ADC_Init(ADC_TypeDef* ADCn, ADC_InitTypeDef* pInitStruct)
{
    ADCn->ADCFG &= ~(ADC_CFGR_PRE | ADC_CFGR_RSLTCTL);
    ADCn->ADCFG |= (u32)(pInitStruct->ADC_PRESCARE) | pInitStruct->ADC_Resolution;

    ADCn->ADCR &= ~(ADC_CR_ALIGN | ADC_CR_MODE | ADC_CR_TRGSEL);
    ADCn->ADCR |= ((u32)pInitStruct->ADC_DataAlign) | pInitStruct->ADC_ExternalTrigConv | ((u32)pInitStruct->ADC_Mode);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Fills each pInitStruct member with its default value.
/// @param  pInitStruct : pointer to an ADC_InitTypeDef structure which will be
///         initialized.
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
void ADC_StructInit(ADC_InitTypeDef* pInitStruct)
{
    pInitStruct->ADC_Resolution         = ADC_Resolution_12b;
    pInitStruct->ADC_PRESCARE           = ADC_PCLK2_PRESCARE_2;
    pInitStruct->ADC_Mode               = ADC_CR_IMM;                           //ADC_Mode_Single;
    pInitStruct->ADC_ContinuousConvMode = DISABLE;                              // useless
    pInitStruct->ADC_ExternalTrigConv   = ADC1_ExternalTrigConv_T1_CC1;
    pInitStruct->ADC_DataAlign          = ADC_DataAlign_Right;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Enables or disables the specified ADC peripheral.
/// @param  ADCn: where n can be 1, 2 to select the ADC peripheral.
/// @param  state: new state of the ADCn peripheral.
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
void ADC_Cmd(ADC_TypeDef* ADCn, FunctionalState state)
{
    (state) ? (ADCn->ADCFG |= ADC_CFGR_ADEN) : (ADCn->ADCFG &= ~ADC_CFGR_ADEN);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Enables or disables the specified ADC DMA request.
/// @param  ADCn: where n can be 1 or 2 to select the ADC peripheral.
/// @param  state: New state of the selected ADC DMA transfer.
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
void ADC_DMACmd(ADC_TypeDef* ADCn, FunctionalState state)
{
    (state) ? (ADCn->ADCR |= ADC_CR_DMAEN) : (ADCn->ADCR &= ~ADC_CR_DMAEN);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Enables or disables the specified ADC interrupts.
/// @param  ADCn: where n can be 1, 2 to select the ADC peripheral.
/// @param  ADC_IT: specifies the ADC interrupt sources to be enabled or disabled.
/// @param  state: New state of the specified ADC interrupts.
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
void ADC_ITConfig(ADC_TypeDef* ADCn, ADCFLAG_TypeDef ADC_IT, FunctionalState state)
{
    if (ADC_IT == ADC_IT_EOC)
        (state) ? (ADCn->ADCR |= ADC_CR_ADIE) : (ADCn->ADCR &= ~ADC_CR_ADIE);
    else
        (state) ? (ADCn->ADCR |= ADC_CR_ADWIE) : (ADCn->ADCR &= ~ADC_CR_ADWIE);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Enables or disables the selected ADC software start conversion .
/// @param  ADCn: where n can be 1, 2 to select the ADC peripheral.
/// @param  state: New state of the selected ADC software start conversion.
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
void ADC_SoftwareStartConvCmd(ADC_TypeDef* ADCn, FunctionalState state)
{
    (state) ? (ADCn->ADCR |= ADC_CR_ADST) : (ADCn->ADCR &= ~ADC_CR_ADST);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Gets the selected ADC Software start conversion Status.
/// @param  ADCn: where n can be 1, 2 to select the ADC peripheral.
/// @retval  The new state of ADC software start conversion (SET or RESET).
////////////////////////////////////////////////////////////////////////////////
FlagStatus ADC_GetSoftwareStartConvStatus(ADC_TypeDef* ADCn)
{
    return (((ADCn->ADCR & ADC_CR_ADST) != (u32)RESET) ? SET : RESET);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Enable the selected ADC channel and configure its sample time. Please
///         use this function if you want to be compatible with older versions
///         of the library.
/// @param  ADCn: where n can be 1, 2 to select the ADC peripheral.
/// @param  channel: the ADC channel to configure.
/// @param  sampleTime: the ADC Channel n Sample time to configure.
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
void ADC_RegularChannelConfig(ADC_TypeDef* ADCn, u32 channel, u8 rank, u32 sampleTime)//ADCSAM_TypeDef
{
    ADCn->ADCFG &= ~ADC_CFGR_SAMCTL;
    if(sampleTime < 0x10)
        sampleTime = sampleTime << 10;
    ADCn->ADCFG |= sampleTime;
    ADCn->ADCHS &= ~(1 << channel);
    ADCn->ADCHS |=  (1 << channel);


    if (channel & ADC_CHSR_CHT)
        ADC_TempSensorVrefintCmd(ENABLE);
    else if (channel & ADC_CHSR_CHV)
        ADC_TempSensorVrefintCmd(ENABLE);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Enables or disables the ADCn conversion through external trigger.
/// @param  ADCn: where n can be 1, 2 to select the ADC peripheral.
/// @param  state: New state of the selected ADC external trigger.
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
void ADC_ExternalTrigConvCmd(ADC_TypeDef* ADCn, FunctionalState state)
{
    (state) ? (ADCn->ADCR |= ADC_CR_TRGEN) : (ADCn->ADCR &= ~ADC_CR_TRGEN);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Returns the last ADCn conversion result data for regular channel.
/// @param  ADCn: where n can be 1, 2 to select the ADC peripheral.
/// @retval The data conversion value.
////////////////////////////////////////////////////////////////////////////////
u32 ADC_GetConversionValue(ADC_TypeDef* ADCn)
{
    return ADCn->ADDATA;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Returns the last ADC conversion result data in dual mode.
/// @param  None
/// @retval The Data conversion value.
////////////////////////////////////////////////////////////////////////////////
u32 ADC_GetDualModeConversionValue()
{
    return (*(u32*)ADC1_BASE);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Configures the ADCn external trigger for injected channels conversion.
/// @param  ADCn: where n can be 1, 2 to select the ADC peripheral.
/// @param  ADC_ExternalTrigInjecConv: Configuring the external trigger source
///         for the ADC.
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
void ADC_ExternalTrigInjectedConvConfig(ADC_TypeDef* ADCn, EXTERTRIG_TypeDef ADC_ExternalTrigInjecConv)
{
    ADCn->ADCR &= ~ADC_CR_TRGSEL;
    ADCn->ADCR |=  ADC_ExternalTrigInjecConv;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Enables or disables the ADCn injected channels conversion through
///         external trigger
/// @param  ADCn: where n can be 1, 2 to select the ADC peripheral.
/// @param  state: New state of the selected ADC external trigger start of
///         external trigger conversion.
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
void ADC_ExternalTrigInjectedConvCmd(ADC_TypeDef* ADCn, FunctionalState state)
{
    (state) ? (ADCn->ADCR |= ADC_CR_TRGEN) : (ADCn->ADCR &= ~ADC_CR_TRGEN);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Enables or disables the analog watchdog.
/// @param  ADCn: where n can be 1, 2 to select the ADC peripheral.
/// @param  state: New state of the selected ADC analog watchdog.
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
void ADC_AnalogWatchdogCmd(ADC_TypeDef* ADCn, FunctionalState state)
{
    (state) ? (ADCn->ADCFG |= ADC_CFGR_ADWEN) : (ADCn->ADCFG &= ~ADC_CFGR_ADWEN);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Configures the high and low thresholds of the analog watchdog.
/// @param  ADCn: where n can be 1, 2 to select the ADC peripheral.
/// @param  HighThreshold: the ADC analog watchdog High threshold value.
///         This parameter must be a 12bit value.
/// @param  LowThreshold: the ADC analog watchdog Low threshold value.
///         This parameter must be a 12bit value.
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
void ADC_AnalogWatchdogThresholdsConfig(ADC_TypeDef* ADCn, u16 HighThreshold, u16 LowThreshold)
{
    u32 tempThreshold;
    tempThreshold = HighThreshold;
    ADCn->ADCMPR    = (tempThreshold << 16) | LowThreshold;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Configures the analog watchdog guarded single channel
/// @param  ADCn: where n can be 1, 2 to select the ADC peripheral.
/// @param  channel: the ADC channel to configure for the analog watchdog.
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
void ADC_AnalogWatchdogSingleChannelConfig(ADC_TypeDef* ADCn, ADCCHANNEL_TypeDef channel)
{
    ADCn->ADCR &= ~ADC_CR_CMPCH;
    ADCn->ADCR |= (channel << ADC_CR_CMPCH_Pos);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Enables or disables the temperature sensor and Vrefint channel.
/// @param  state: New state of the temperature sensor.
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
void ADC_TempSensorVrefintCmd(FunctionalState state)
{

    (state) ? (ADC1->ADCFG |=  (ADC_CFGR_TEN | ADC_CFGR_VEN))
    : (ADC1->ADCFG &= ~(ADC_CFGR_TEN | ADC_CFGR_VEN));
}


////////////////////////////////////////////////////////////////////////////////
/// @brief  Enables or disables the temperature sensor .
/// @param  state: New state of the temperature sensor.
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
void ADC_TempSensorCmd(FunctionalState state)
{
    ADC_TempSensorVrefintCmd(state);
}
////////////////////////////////////////////////////////////////////////////////
/// @brief  Enables or disables the Vrefint channel.
/// @param  state: New state of the Vrefint channel.
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
void ADC_VrefintCmd(FunctionalState state)
{
    ADC_TempSensorVrefintCmd(state);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Enables or disables the temperature sensor and Vrefint channel.
/// @param  chs: temperature sensor bit & Vrefint bit.
/// @param  state: New state of the temperature sensor.
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
void exADC_TempSensorVrefintCmd(u32 chs, FunctionalState state)
{

    if (chs & ADC_CHSR_CHT) {
        (state) ? (ADC1->ADCFG |=  ADC_CFGR_TEN)
        : (ADC1->ADCFG &= ~ADC_CFGR_TEN);
    }
    else if (chs & ADC_CHSR_CHV) {
        (state) ? (ADC1->ADCFG |=  ADC_CFGR_VEN)
        : (ADC1->ADCFG &= ~ADC_CFGR_VEN);
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Checks whether the specified ADC flag is set or not.
/// @param  ADCn: where n can be 1, 2 to select the ADC peripheral.
/// @param  ADC_FLAG: specifies the flag to check.
/// @retval The New state of ADC_FLAG (SET or RESET).
////////////////////////////////////////////////////////////////////////////////
FlagStatus ADC_GetFlagStatus(ADC_TypeDef* ADCn, ADCFLAG_TypeDef ADC_FLAG)
{
    return (ADC_FLAG == ADC_IT_EOC) ? ((ADCn->ADSTA & ADC_SR_ADIF) ? SET : RESET) : ((ADCn->ADSTA & ADC_SR_ADWIF) ? SET : RESET);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Clears the ADCn's pending flags.
/// @param  ADCn: where n can be 1, 2 to select the ADC peripheral.
/// @param  ADC_FLAG: specifies the flag to clear.
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
void ADC_ClearFlag(ADC_TypeDef* ADCn, ADCFLAG_TypeDef ADC_FLAG)
{
    (ADC_FLAG == ADC_IT_EOC) ? (ADCn->ADSTA |= ADC_SR_ADIF) : (ADCn->ADSTA |= ADC_SR_ADWIF);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Checks whether the specified ADCn's interrupt has occurred or not.
/// @param  ADCn: where n can be 1, 2 to select the ADC peripheral.
/// @param  ADC_IT: specifies the ADC interrupt source to check.
/// @retval The new state of ADC_IT (SET or RESET).
////////////////////////////////////////////////////////////////////////////////
ITStatus ADC_GetITStatus(ADC_TypeDef* ADCn, ADCFLAG_TypeDef ADC_IT)
{
    return (ADC_IT == ADC_IT_EOC) ? ((ADCn->ADSTA & ADC_SR_ADIF) ? SET : RESET) : ((ADCn->ADSTA & ADC_SR_ADWIF) ? SET : RESET);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Clears the ADCn's interrupt pending bits.
/// @param  ADCn: where n can be 1, 2 to select the ADC peripheral.
/// @param  ADC_IT: specifies the ADC interrupt pending bit to clear.
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
void ADC_ClearITPendingBit(ADC_TypeDef* ADCn, ADCFLAG_TypeDef ADC_IT)
{
    (ADC_IT == ADC_IT_EOC) ? (ADCn->ADSTA |= ADC_SR_ADIF) : (ADCn->ADSTA |= ADC_SR_ADWIF);
}
////////////////////////////////////////////////////////////////////////////////
/// @brief  Configures the ADCn any channels conversion rank and channel.
/// @param  ADCn: where n can be 1, 2 to select the ADC peripheral.
/// @param  Rank: Rank can be 0x0~0xf for the convert sequence.
/// @param  ADC_Channel: Configuring the target channel to be converted.
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
void ADC_ANY_CH_Config(ADC_TypeDef* ADCn, u8 Rank, ADCCHANNEL_TypeDef ADC_Channel)
{
    Rank = Rank & 0xF;
    if(Rank < 8) {
        ADCn->CHANY0 &= ~(0x0F << (4 * Rank));
        ADCn->CHANY0 |= (ADC_Channel << (4 * Rank));
    }
    else {
        ADCn->CHANY1 &= ~(0x0F << (4 * (Rank - 8)));
        ADCn->CHANY1 |= (ADC_Channel << (4 * (Rank - 8)));
    }
}
////////////////////////////////////////////////////////////////////////////////
/// @brief  Configures the ADCn any channels conversion Max rank number
/// @param  ADCn: where n can be 1, 2 to select the ADC peripheral.
/// @param  Num: Configuring the max rank number for the ADC.
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
void ADC_ANY_NUM_Config(ADC_TypeDef* ADCn, u8 Num)
{
    if(Num > 15) Num = 15;                                                      //15 ? 16 need to be confirmed
    ADCn->ANYCFG = Num;
}
////////////////////////////////////////////////////////////////////////////////
/// @brief  Enables or disables the ANY channel converter.
/// @param  state: enable or disable the ANY channel converter mode.
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
void ADC_ANY_Cmd(ADC_TypeDef* ADCn, FunctionalState state)
{
    (state) ? (ADCn->ANYCR |= ADC1_CHANY_CR_MDEN) : (ADCn->ANYCR &= ~ADC1_CHANY_CR_MDEN);
}



/// @}

/// @}

/// @}

