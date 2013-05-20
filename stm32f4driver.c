/*
** enzidriver.c - enzi device driver for mruby
**
** Copyright(c) FUKUOKA CSK CORPORATION
** Copyright(c) Manycolors Inc
*/

#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"

#include "stm32f4_discovery.h"

#include "stm32f4xx_conf.h"

#include "stm32f4driver.h"

#include "xprintf.h"

extern __IO uint32_t TimingDelay;
extern __IO uint32_t SystemTick;

// set up duty cycle as 65536 / PWM_DUTY

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
 
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static pinStatus arduino_pins[PIN_MAX + 1];

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment = 4
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;


/* Private function prototypes -----------------------------------------------*/
// GPIO
static GPIO_TypeDef* get_GPIO_TypeDef(uint16_t arduino_pin_num);
static uint16_t get_GPIO_Pin(uint16_t arduino_pin_num);
static uint32_t get_RCC_AHB1Periph_GPIOx(uint16_t arduino_pin_num);

// Digital
static void DigitalOut_Config(uint16_t arduino_pin_num);
static void DigitalIn_Config(uint16_t arduino_pin_num);

// Analog Read
static ADC_TypeDef* get_ADC_TypeDef(uint16_t arduino_pin_num);
static uint32_t get_ADC_RCC_APB2Periph(uint16_t arduino_pin_num);
static uint8_t get_ADC_Channel(uint16_t arduino_pin_num);
static void AnalogIn_Config(uint16_t arduino_pin_num);

// Analog Write
static void PWM_Config_GPIOx(uint16_t pin, uint8_t GPIO_AF_TIMx);
static void PWM_Config_TIM_TimeBase(TIM_TypeDef* TIMx, uint16_t period);
static void PWM_Config_TIM_OC(TIM_TypeDef* TIMx, uint8_t tim_ch);
static void PWM_Config(uint16_t arduino_pin_num, uint16_t period);
static void PWM_Update(uint16_t *dutycycles);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */

static GPIO_TypeDef* get_GPIO_TypeDef(uint16_t arduino_pin_num)
{
  GPIO_TypeDef *GPIOx = NULL;

  if (arduino_pin_num > PIN_MAX)
    return NULL;

  switch(arduino_pins[arduino_pin_num].port_num)
  {
  case PORT_A: GPIOx = GPIOA; break;
  case PORT_B: GPIOx = GPIOB; break;
  case PORT_C: GPIOx = GPIOC; break;
  case PORT_D: GPIOx = GPIOD; break;
  case PORT_E: GPIOx = GPIOE; break;
  case PORT_F: GPIOx = GPIOF; break;
  case PORT_G: GPIOx = GPIOG; break;
  default:     GPIOx = NULL;  break;
  }

  return GPIOx;
}

static uint16_t get_GPIO_Pin(uint16_t arduino_pin_num)
{
  if (arduino_pin_num > PIN_MAX)
    return 0; /* unselected pins */

  return (0x0001 << arduino_pins[arduino_pin_num].pin_num);
}

static uint32_t get_RCC_AHB1Periph_GPIOx(uint16_t arduino_pin_num)
{
  uint32_t RCC_AHB1Periph = 0;

  if (arduino_pin_num > PIN_MAX)
    return 0;

  switch(arduino_pins[arduino_pin_num].port_num)
  {
  case PORT_A: RCC_AHB1Periph = RCC_AHB1Periph_GPIOA; break;
  case PORT_B: RCC_AHB1Periph = RCC_AHB1Periph_GPIOB; break;
  case PORT_C: RCC_AHB1Periph = RCC_AHB1Periph_GPIOC; break;
  case PORT_D: RCC_AHB1Periph = RCC_AHB1Periph_GPIOD; break;
  case PORT_E: RCC_AHB1Periph = RCC_AHB1Periph_GPIOE; break;
  case PORT_F: RCC_AHB1Periph = RCC_AHB1Periph_GPIOF; break;
  case PORT_G: RCC_AHB1Periph = RCC_AHB1Periph_GPIOG; break;
  default:     RCC_AHB1Periph = 0;  break;
  }

  return RCC_AHB1Periph;

}

static void DigitalOut_Config(uint16_t arduino_pin_num)
{
  if (arduino_pins[arduino_pin_num].pin_status == PIN_UNDEFINED) {
    setup_pin(arduino_pin_num);
  }

  if(arduino_pins[arduino_pin_num].pin_status != PIN_DIGITAL_OUT){
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_TypeDef* GPIOx = get_GPIO_TypeDef(arduino_pin_num);
    uint16_t GPIO_Pin = get_GPIO_Pin(arduino_pin_num);
    uint32_t RCC_AHB1Periph_GPIOx = get_RCC_AHB1Periph_GPIOx(arduino_pin_num);

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOx, ENABLE);
    GPIO_Init(GPIOx, &GPIO_InitStructure);

    arduino_pins[arduino_pin_num].pin_status = PIN_DIGITAL_OUT;
  }
}

static void DigitalIn_Config(uint16_t arduino_pin_num)
{
  if (arduino_pins[arduino_pin_num].pin_status == PIN_UNDEFINED) {
    setup_pin(arduino_pin_num);
  }

  if(arduino_pins[arduino_pin_num].pin_status != PIN_DIGITAL_IN){
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_TypeDef* GPIOx = get_GPIO_TypeDef(arduino_pin_num);
    uint16_t GPIO_Pin = get_GPIO_Pin(arduino_pin_num);
    uint32_t RCC_AHB1Periph_GPIOx = get_RCC_AHB1Periph_GPIOx(arduino_pin_num);

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOx, ENABLE);
    GPIO_Init(GPIOx, &GPIO_InitStructure);

    arduino_pins[arduino_pin_num].pin_status = PIN_DIGITAL_IN;
  }
}

static ADC_TypeDef* get_ADC_TypeDef(uint16_t arduino_pin_num)
{
  ADC_TypeDef *ADCx = NULL;

  if (arduino_pin_num > PIN_MAX)
    return NULL;

  switch(arduino_pin_num)
  {
  case 14: ADCx = ADC3; break;
  case 15: ADCx = ADC3; break;
  case 16: ADCx = ADC2; break;
  case 17: ADCx = ADC2; break;
  case 18: ADCx = ADC1; break;
  case 19: ADCx = ADC1; break;
  default: ADCx = NULL; break;
  }

  return ADCx;
}

static uint32_t get_ADC_RCC_APB2Periph(uint16_t arduino_pin_num)
{
  uint32_t RCC_APB2Periph = 0;

  if (arduino_pin_num > PIN_MAX)
    return 0;

  switch(arduino_pin_num)
  {
  case 14: RCC_APB2Periph = RCC_APB2Periph_ADC3; break;
  case 15: RCC_APB2Periph = RCC_APB2Periph_ADC3; break;
  case 16: RCC_APB2Periph = RCC_APB2Periph_ADC2; break;
  case 17: RCC_APB2Periph = RCC_APB2Periph_ADC2; break;
  case 18: RCC_APB2Periph = RCC_APB2Periph_ADC1; break;
  case 19: RCC_APB2Periph = RCC_APB2Periph_ADC1; break;
  default: RCC_APB2Periph = NULL; break;
  }

  return RCC_APB2Periph;
}

static uint8_t get_ADC_Channel(uint16_t arduino_pin_num)
{
  uint8_t ADC_Channel = 0;

  if (arduino_pin_num > PIN_MAX)
    return 0;

  switch(arduino_pin_num)
  {
  case 14: ADC_Channel = ADC_Channel_10; break;
  case 15: ADC_Channel = ADC_Channel_11; break;
  case 16: ADC_Channel = ADC_Channel_8;  break;
  case 17: ADC_Channel = ADC_Channel_9;  break;
  case 18: ADC_Channel = ADC_Channel_14; break;
  case 19: ADC_Channel = ADC_Channel_15; break;
  default: ADC_Channel = 0; break;
  }

  return ADC_Channel;
}

static void AnalogIn_Config(uint16_t arduino_pin_num)
{
  if (arduino_pins[arduino_pin_num].pin_status == PIN_UNDEFINED) {
    setup_pin(arduino_pin_num);
  }

  if(arduino_pins[arduino_pin_num].pin_status != PIN_ANALOG_IN){
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_TypeDef* GPIOx = get_GPIO_TypeDef(arduino_pin_num);
    uint16_t GPIO_Pin = get_GPIO_Pin(arduino_pin_num);
    uint32_t RCC_AHB1Periph_GPIOx = get_RCC_AHB1Periph_GPIOx(arduino_pin_num);

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOx, ENABLE);
    GPIO_Init(GPIOx, &GPIO_InitStructure);

    /* ADCx Common Init ******************************************************/
    ADC_CommonInitTypeDef ADC_CommonInitStructure;

    ADC_CommonStructInit(&ADC_CommonInitStructure);
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div6; /* Can't use Div2 and Div4 */
    ADC_CommonInit(&ADC_CommonInitStructure);

    /* ADCx Init *************************************************************/
    ADC_InitTypeDef ADC_InitStructure;
    ADC_TypeDef* ADCx = get_ADC_TypeDef(arduino_pin_num);
    uint32_t RCC_APB2Periph = get_ADC_RCC_APB2Periph(arduino_pin_num);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph, ENABLE);
    ADC_StructInit(&ADC_InitStructure);
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_10b;
    ADC_Init(ADCx, &ADC_InitStructure);

    /* Enable DMA request after last transfer (Single-ADC mode) */
//    ADC_DMARequestAfterLastTransferCmd(ADCx, ENABLE);

    /* Enable ADCx DMA */
//    ADC_DMACmd(ADCx, ENABLE);

    /* Enable ADCx  */
    ADC_Cmd(ADCx, ENABLE);

    arduino_pins[arduino_pin_num].pin_status = PIN_ANALOG_IN;
  }
}


static void PWM_Config_GPIOx(uint16_t pin, uint8_t GPIO_AF_TIMx)
{
  GPIO_TypeDef* GPIOx                = get_GPIO_TypeDef(pin);
  uint16_t      GPIO_Pin             = get_GPIO_Pin(pin);
  uint32_t      RCC_AHB1Periph_GPIOx = get_RCC_AHB1Periph_GPIOx(pin);
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOx, ENABLE);

  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin;

  GPIO_Init(GPIOx, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOx, arduino_pins[pin].pin_num, GPIO_AF_TIMx);
}

static void PWM_Config_TIM_TimeBase(TIM_TypeDef* TIMx, uint16_t period)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBase;
  uint16_t PrescalerValue = 0;

  /* Compute the prescaler value */
  PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 28000000) - 1;

  /* Time base configuration */
  TIM_TimeBase.TIM_Period        = period;
  TIM_TimeBase.TIM_Prescaler     = PrescalerValue;
  TIM_TimeBase.TIM_ClockDivision = 0;
  TIM_TimeBase.TIM_CounterMode   = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIMx, &TIM_TimeBase);
}

static void PWM_Config_TIM_OC(TIM_TypeDef* TIMx, uint8_t tim_ch)
{
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse       = 0;
  TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;

  switch (tim_ch)
  {
  case 1:
    TIM_OC1Init(TIMx, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIMx, TIM_OCPreload_Enable);
  case 2:
    TIM_OC2Init(TIMx, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIMx, TIM_OCPreload_Enable);
  case 3:
    TIM_OC3Init(TIMx, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIMx, TIM_OCPreload_Enable);
  case 4:
    TIM_OC4Init(TIMx, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIMx, TIM_OCPreload_Enable);
  default:
    break;
  }
}

static void PWM_Config(uint16_t arduino_pin_num, uint16_t period)
{
  if (arduino_pins[arduino_pin_num].pin_status == PIN_UNDEFINED) {
    setup_pin(arduino_pin_num);
  }

  if( arduino_pin_num == 3 ||
      arduino_pin_num == 5 ||
      arduino_pin_num == 6 ||
      arduino_pin_num == 8 ||
      arduino_pin_num == 9 ||
      arduino_pin_num == 10){

    if (arduino_pins[arduino_pin_num].pin_status == PIN_ANALOG_OUT) {
      return; /* configured */
    }

    TIM_TypeDef* TIMx = 0;
    uint8_t GPIO_AF_TIMx = 0;
    uint8_t tim_ch = 0;

    switch (arduino_pin_num)
    {
    case 3: /* PA1 - TIM5 - ch2 */
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
      TIMx         = TIM5;
      GPIO_AF_TIMx = GPIO_AF_TIM5;
      tim_ch       = 2;
      break;

    case 5: /* PG12- TIM4 - ch3 */
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
      TIMx         = TIM4;
      GPIO_AF_TIMx = GPIO_AF_TIM4;
      tim_ch       = 3;
      break;

    case 6: /* PE6 - TIM9 - ch2 */
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
      TIMx         = TIM9;
      GPIO_AF_TIMx = GPIO_AF_TIM9;
      tim_ch       = 2; /* CCR2 */
      break;

    case 8: /* PE5 - TIM9 - ch1 */
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
      TIMx         = TIM9;
      GPIO_AF_TIMx = GPIO_AF_TIM9;
      tim_ch       = 1; /* CCR1 */
      break;

    case 9: /* PB11- TIM2 - ch4 */
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
      TIMx         = TIM2;
      GPIO_AF_TIMx = GPIO_AF_TIM2;
      tim_ch       = 4;
      break;

    case 10: /* PB9 - TIM11- ch1 */
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);
      TIMx         = TIM11;
      GPIO_AF_TIMx = GPIO_AF_TIM11;
      tim_ch       = 1;
      break;

    default:
      break;
    }

    PWM_Config_GPIOx(arduino_pin_num, GPIO_AF_TIMx);
    PWM_Config_TIM_TimeBase(TIMx, period);
    PWM_Config_TIM_OC(TIMx, tim_ch);

    TIM_ARRPreloadConfig(TIMx, ENABLE);

    TIM_Cmd(TIMx, ENABLE);

    arduino_pins[arduino_pin_num].pin_status = PIN_ANALOG_OUT;
  }
}

static void PWM_Update(uint16_t *dutycycles)
{
  uint16_t pin;

  for (pin=0; pin<=PIN_MAX; pin++)
  {
    if (arduino_pins[pin].pin_status == PIN_ANALOG_OUT) {
      uint16_t val = dutycycles[pin];

      // 3, 5, 6, 9, 10, 11
      //
      //  3 - PA1 - TIM5 - ch2
      //  5 - PG12- TIM4 - ch3
      //  6 - PE6 - TIM9 - ch2
      //  8 - PE5 - TIM9 - ch1
      //  9 - PB11- TIM2 - ch4
      // 10 - PB9 - TIM11- ch1
      switch (pin) {
        case 3:  TIM5->CCR2  = val;  break;
        case 5:  TIM4->CCR3  = val;  break;
        case 6:  TIM9->CCR2  = val;  break;
        case 8:  TIM9->CCR1  = val;  break;
        case 9:  TIM2->CCR4  = val;  break;
        case 10: TIM11->CCR1 = val;  break;
        default: break;
      }
    }
  }
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in 10 ms.
  * @retval None
  */
/*
void Delay(uint32_t nTime)
{
  if (SysTick_Config(SystemCoreClock/1000))
  {
    while(1);
  }
  TimingDelay = (__IO uint32_t)nTime;
  while(TimingDelay != 0);
}
*/

uint32_t Millis(void)
{
  return SystemTick;
}

void analogWrite(uint16_t pin, uint16_t dutycycle)
{
  static uint16_t dutycycles[PIN_MAX + 1] = {0};

  PWM_Config(pin, PWM_DUTY);

  dutycycles[pin] = dutycycle;

  PWM_Update(dutycycles);
}

/*
 *  for DigitalIO Class
 */
void LEDon(void)
{
  STM_EVAL_LEDOn(LED0);
}

void LEDoff(void)
{
  STM_EVAL_LEDOff(LED0);
}

void digitalWrite(uint16_t pin, uint16_t val)
{
  DigitalOut_Config(pin);

  GPIO_TypeDef* GPIOx = get_GPIO_TypeDef(pin);
  uint16_t GPIO_Pin = get_GPIO_Pin(pin);

  GPIO_WriteBit(GPIOx, GPIO_Pin, val);
}

uint16_t digitalRead(uint16_t pin)
{
  uint16_t val=0;

  DigitalIn_Config(pin);

  GPIO_TypeDef* GPIOx = get_GPIO_TypeDef(pin);
  uint16_t GPIO_Pin = get_GPIO_Pin(pin);

  val = GPIO_ReadInputDataBit(GPIOx, GPIO_Pin);

  return (val > 0) ? 1 : 0;
}

uint16_t analogRead(uint16_t pin){
  ADC_TypeDef* ADCx = get_ADC_TypeDef(pin);
  uint8_t ADC_Channel = get_ADC_Channel(pin);

  if (ADCx == NULL)
    return 0;

  AnalogIn_Config(pin);

  /* ADCx regular channel configuration **************************************/
  ADC_RegularChannelConfig(ADCx, ADC_Channel, 1, ADC_SampleTime_3Cycles);

  // Start ADCx Software Conversion
  ADC_SoftwareStartConv( ADCx );
  // Wait until convjuk ersion completion
  while(ADC_GetFlagStatus(ADCx, ADC_FLAG_EOC) == RESET);
  // Get the conversion value
  return ADC_GetConversionValue(ADCx);
}

/*
 *  USB Initialization
 */
void initUSB(void)
{
  USBD_Init(&USB_OTG_dev,
#ifdef USE_USB_OTG_HS
  USB_OTG_HS_CORE_ID,
#else
  USB_OTG_FS_CORE_ID,
#endif
  &USR_desc,
  &USBD_CDC_cb,
  &USR_cb);
}

void setup_pin(uint16_t arduino_pin_num)
{
  pinStatus *ps = &arduino_pins[arduino_pin_num];

  if (ps->pin_status != PIN_UNDEFINED)
    return;

  // if a pis is not defined.
  if (arduino_pin_num <= PIN_MAX)
  {
    switch (arduino_pin_num)
    {
    case 0:  ps->port_num = PORT_A; ps->pin_num = 3;  break;
    case 1:  ps->port_num = PORT_A; ps->pin_num = 2;  break;
    case 2:  ps->port_num = PORT_G; ps->pin_num = 7;  break;
    case 3:  ps->port_num = PORT_A; ps->pin_num = 1;  break;
    case 4:  ps->port_num = PORT_G; ps->pin_num = 12; break;
    case 5:  ps->port_num = PORT_B; ps->pin_num = 8;  break;
    case 6:  ps->port_num = PORT_E; ps->pin_num = 6;  break;
    case 7:  ps->port_num = PORT_G; ps->pin_num = 15; break;
    case 8:  ps->port_num = PORT_E; ps->pin_num = 5;  break;
    case 9:  ps->port_num = PORT_B; ps->pin_num = 11; break;
    case 10: ps->port_num = PORT_B; ps->pin_num = 9;  break;
    case 11: ps->port_num = PORT_C; ps->pin_num = 3;  break;
    case 12: ps->port_num = PORT_C; ps->pin_num = 2;  break;
    case 13: ps->port_num = PORT_B; ps->pin_num = 10; break;
    case 14: ps->port_num = PORT_C; ps->pin_num = 0;  break;
    case 15: ps->port_num = PORT_C; ps->pin_num = 1;  break;
    case 16: ps->port_num = PORT_B; ps->pin_num = 0;  break;
    case 17: ps->port_num = PORT_B; ps->pin_num = 1;  break;
    case 18: ps->port_num = PORT_C; ps->pin_num = 4;  break;
    case 19: ps->port_num = PORT_C; ps->pin_num = 5;  break;
    }

    ps->pin_status  = PIN_USED;
  }
  else
  {
    ps->port_num    = PORT_UNKNOWN;
    ps->pin_status  = PIN_UNKNOWN;
    ps->pin_num     = 0xff;
  }
}

//#define DEBUG_ANALOG_IO
#ifdef DEBUG_ANALOG_IO

#include <xprintf.h>
#include <enzi/enziutil.h>

/* dump to console */
void debug_analog_read(void)
{
  uint16_t ai_pin;
  uint16_t val[6];

  Delay(500);
  xprintf("%s start.\n\n", __func__);

  xprintf("A0   A1   A2   A3   A4   A5\n");

  while(1)
  {
    for (ai_pin=ANALOG_PIN_MIN; ai_pin<=ANALOG_PIN_MAX;ai_pin++)
    {
      val[ai_pin - ANALOG_PIN_MIN] = analogRead(ai_pin);
    }

    xprintf("%4d %4d %4d %4d %4d %4d\n", val[0],val[1],val[2],val[3],val[4],val[5]);
  }

  xprintf("\n");
  xprintf("%s end.\n", __func__);
}

void debug_analog_write(void)
{
  int i, times;
  uint16_t ao_pin_num = 6;
  uint16_t ao_pin[6] = {3,5,6,8,9,10};

  Delay(500);
  xprintf("%s start.\n", __func__);

  for (times=1; times<=10; times++)
  {
    for (i=0; i<ao_pin_num; i++)
    {
      uint16_t percentage, duty;

      percentage = (times+i) * 10;
      while (percentage > 100) {percentage -= 100;}

      duty = (PWM_DUTY-1) * (percentage/100.0);
      analogWrite(ao_pin[i], duty);
    }
    Delay(1);
    for (i=0; i<ao_pin_num; i++)
    {
      analogWrite(ao_pin[i], 0);
    }
    Delay(1);
  }

  xprintf("\n");
  xprintf("%s end.\n", __func__);
}

#endif /* DEBUG_ANALOG_IO */

//#define DEBUG_DIGITAL_IO
#ifdef DEBUG_DIGITAL_IO

#include <enzi/enziutil.h>

static void dump_din_header()
{
  uint16_t pin;

  for (pin=DIGITAL_PIN_MIN; pin<=DIGITAL_PIN_MAX; pin++)
  {
    xprintf("%3d", pin);
  }
  xprintf("\n");
}
static void dump_din_val(uint16_t dout_pin_num)
{
  int pin;

  for (pin=DIGITAL_PIN_MIN; pin<=DIGITAL_PIN_MAX; pin++)
  {
    if (pin == dout_pin_num)
      xprintf("  -");
    else
      xprintf("%3d", digitalRead(pin));
  }
  xprintf("\n");
}

/* dump to console */
void debug_digital_io(void)
{
  Delay(1000);
  /* digital io test */
  xprintf("dio-test start.\n");

  {
    int cnt;
    uint16_t dout_pin_num;

    xprintf("Read :               ");
    dump_din_header();

    for (dout_pin_num=DIGITAL_PIN_MIN; dout_pin_num<=DIGITAL_PIN_MAX; dout_pin_num++)
    {
      uint16_t val=0;

      for (cnt=0; cnt<3; cnt++)
      {
        xprintf("dout (pin:%2d,val:%2d) ", dout_pin_num,val);
        digitalWrite(dout_pin_num, val);
        Delay(100);

        dump_din_val(dout_pin_num);

        val = (val)?0:1;
      }
    }
  }

  xprintf("dio-test end.\n");
}

/* Not dump to console */
void debug_digital_io2(void)
{
  Delay(1000);
  /* digital io test */
  xprintf("dio-test start.\n");

  {
    int cnt;
    uint16_t dout_pin_num;

    for (dout_pin_num=DIGITAL_PIN_MIN; dout_pin_num<=DIGITAL_PIN_MAX; dout_pin_num++)
    {
      uint16_t val=0;

      for (cnt=0; cnt<9; cnt++)
      {
        digitalWrite(dout_pin_num, val);

        Delay(10);

        val = (val)?0:1;
      }
    }
  }

  xprintf("dio-test end.\n");
}

#endif /* DEBUG_DIGITAL_IO */
