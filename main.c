/**
  ******************************************************************************
  * @file    main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

#define INCLUDE_MRUBY

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"
#include "stm32f4_discovery.h"

#include "stm32f4xx.h"

#include "hw_config.h"

//Library config for this project!!!!!!!!!!!
#include "stm32f4xx_conf.h"
#include "stm32f4xx_flash.h"
#include "stm32f4driver.h"
#include "xprintf.h"

#ifdef INCLUDE_MRUBY
#include "mruby.h"
#include "mruby/proc.h"
#endif

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment = 4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;

__IO uint32_t TimingDelay;
__IO uint32_t SystemTick;


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  uint32_t sts = 0;
#ifdef INCLUDE_MRUBY
  mrb_state *mrb = NULL;
  extern const uint8_t appbin[];
#endif

  xdev_out(VCP_put_char);
  xdev_in(VCP_get_char);

  /* Initialize LEDs and User_Button on STM32F4-Discovery */
  STM_EVAL_LEDInit(LED0);
  STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_GPIO);

  /* USB initialization */
  initUSB();

  /* SRAM initialization */
  SRAM_Init();

#ifdef INCLUDE_MRUBY
  /* initialize mruby */
  mrb = mrb_open();

  /* start mruby application */
  mrb_load_irep(mrb, appbin);
  if (mrb->exc) {
    mrb_p(mrb, mrb_obj_value(mrb->exc));
  }

  /* finalize mruby */
  mrb_close(mrb);
#endif

  /* application done. */
  while (1) {
    STM_EVAL_LEDToggle(LED0);
    Delay(sts ? 100 : 1000);
  }
  return 0;
}

/**
  * @brief  System tick handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler()
{
  SystemTick++;
  if (TimingDelay != 0){
    TimingDelay --;
  }
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in 10 ms.
  * @retval None
  */
void Delay(uint32_t nTime)
{
  if (SysTick_Config(SystemCoreClock/1000))
  {
    while(1);
  }
  TimingDelay = (__IO uint32_t)nTime;
  while(TimingDelay != 0);
}
