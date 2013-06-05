#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_soc.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf_gpiote.h"
#include "nrf51_bitfields.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_tps.h"
#include "ble_ias.h"
#include "ble_lls.h"
#include "ble_bas.h"
#include "ble_conn_params.h"
#include "ble_sensorsim.h"
#include "ble_stack_handler.h"
#include "app_timer.h"
#include "ble_bondmngr.h"
#include "ble_ias_c.h"
#include "ble_error_log.h"
#include "app_gpiote.h"
#include "app_button.h"
#include "ble_radio_notification.h"
#include "ble_flash.h"

#include "nrf51.h"
#include "nrf_delay.h"
#include "app_util.h"

#include "cherry.h"


#define ALERT_LEVEL_MILD_LED_PIN_NO       NRF6310_LED_2                                     /**< Is on when we are in Mild Alert state. */
#define ALERT_LEVEL_HIGH_LED_PIN_NO       NRF6310_LED_3                                     /**< Is on when we are in High Alert state. */
#define ADV_INTERVAL_SLOW_LED_PIN_NO      NRF6310_LED_4                                     /**< Is on when we are doing slow advertising. */
#define PEER_SRV_DISC_LED_PIN_NO          NRF6310_LED_5                                     /**< Is on when the Immediate Alert Service has been discovered at the peer. */
#define ADV_WHITELIST_LED_PIN_NO          NRF6310_LED_6                                     /**< Is on when we are doing advertising with whitelist. */

#define MAX_RTC_TASKS_DELAY               47                                                /**< Maximum delay until an RTC task is executed. */
#define APP_GPIOTE_MAX_USERS              1                                                 /**< Maximum number of users of the GPIOTE handler. */

#define LERBIT_CHERRY_LED_PIN_NO               8
#define LERBIT_CHERRY_BUZZER_PIN_NO            9
#define LERBIT_CHERRY_BUTTON_PIN_NO            0
#define L_CLOCK_ONE_HZ_SAMPLE                  (32768UL)
#if 0
#define LERBIT_CHERRY_LED_PWM_MAX_SEMPLE       (32768UL)
#define LERBIT_CHERRY_LED_PWM_SEMPLE           (LERBIT_CHERRY_LED_PWM_MAX_SEMPLE/16)
#define LERBIT_CHERRY_BUZZER_PWM_MAX_SEMPLE    (32768UL)
#define LERBIT_CHERRY_BUZZER_PWM_SEMPLE        (LERBIT_CHERRY_BUZZER_PWM_MAX_SEMPLE/2)
#endif

int lerbit_cherry_led_flash_cnt = 5;
int lerbit_cherry_pwm_led_max_sample = L_CLOCK_ONE_HZ_SAMPLE * 5;
int lerbit_cherry_pwm_led_sample = L_CLOCK_ONE_HZ_SAMPLE / 8;
int lerbit_cherry_pwm_buzzer_max_sample = 0;
int lerbit_cherry_pwm_buzzer_sample = 0;

typedef enum {
  LERBIT_LED_Advertising= 0,
  LERBIT_LED_Connected,
  LERBIT_LED_Mild_Alert,
  LERBIT_LED_High_Alert,
  LERBIT_LED_Slow_Advertising,
  LERBIT_LED_Immediate_Alert,
  LERBIT_LED_Whitelist_Advertising,
  LERBIT_LED_Asserted
} LERBIT_LED_STATUS;

void RTC1_IRQHandler(void)
{
    static int led_flash_cnt_temp;
    // Clear all events (also unexpected ones)
    if (NRF_RTC1->EVENTS_COMPARE[0] != 0) {
      NRF_RTC1->EVENTS_COMPARE[0] = 0;
      led_flash_cnt_temp = (lerbit_cherry_led_flash_cnt - 1) * 2;
      NRF_RTC1->CC[1] = NRF_RTC1->CC[0] + lerbit_cherry_pwm_led_sample;
      NRF_RTC1->CC[0] += lerbit_cherry_pwm_led_max_sample;
    }

    if (NRF_RTC1->EVENTS_COMPARE[1] != 0) {
      NRF_RTC1->EVENTS_COMPARE[1] = 0;
      if (led_flash_cnt_temp) {
        if (led_flash_cnt_temp % 2)
          NRF_RTC1->CC[1] += lerbit_cherry_pwm_led_sample;
        else 
          NRF_RTC1->CC[1] += lerbit_cherry_pwm_led_sample * 3;
        led_flash_cnt_temp--;
      }
    }

    if (NRF_RTC1->EVENTS_COMPARE[2] != 0) {
      NRF_RTC1->EVENTS_COMPARE[2] = 0;
      NRF_RTC1->CC[3] = NRF_RTC1->CC[2] + lerbit_cherry_pwm_buzzer_sample;
      NRF_RTC1->CC[2] += lerbit_cherry_pwm_buzzer_max_sample;
    }

    if (NRF_RTC1->EVENTS_COMPARE[3] != 0) {
      NRF_RTC1->EVENTS_COMPARE[3] = 0;
      //NRF_RTC1->CC[3] += lerbit_cherry_pwm_buzzer_sample;
    }

    if (NRF_RTC1->EVENTS_TICK != 0) {
      NRF_RTC1->EVENTS_TICK = 0;
    }

    if (NRF_RTC1->EVENTS_OVRFLW != 0) {
      NRF_RTC1->EVENTS_OVRFLW = 0;
    }


}


static void lerbit_rtc1_init(void)
{
    NRF_CLOCK->LFCLKSRC = 1;
	  NRF_CLOCK->TASKS_LFCLKSTART = 1;
	  
	  NRF_RTC1->PRESCALER = 0;
    NVIC_SetPriority(RTC1_IRQn, APP_IRQ_PRIORITY_LOW);
}

/**@brief Starts the RTC1 timer.
 */
static void lerbit_rtc1_start(void)
{
	  NRF_RTC1->CC[0] = lerbit_cherry_pwm_led_max_sample;
	  NRF_RTC1->CC[1] = 0;
	  
	  NRF_RTC1->CC[2] = lerbit_cherry_pwm_buzzer_max_sample;
	  NRF_RTC1->CC[3] = 0;

    NRF_RTC1->EVTENSET = (RTC_EVTEN_COMPARE0_Enabled << RTC_EVTEN_COMPARE0_Pos)
	                      |(RTC_EVTEN_COMPARE1_Enabled << RTC_EVTEN_COMPARE1_Pos)
	                      |(RTC_EVTEN_COMPARE2_Enabled << RTC_EVTEN_COMPARE2_Pos)
	                      |(RTC_EVTEN_COMPARE3_Enabled << RTC_EVTEN_COMPARE3_Pos);
    NRF_RTC1->INTENSET = (RTC_EVTEN_COMPARE0_Enabled << RTC_EVTEN_COMPARE0_Pos)
	                      |(RTC_EVTEN_COMPARE1_Enabled << RTC_EVTEN_COMPARE1_Pos)
	                      |(RTC_EVTEN_COMPARE2_Enabled << RTC_EVTEN_COMPARE2_Pos)
	                      |(RTC_EVTEN_COMPARE3_Enabled << RTC_EVTEN_COMPARE3_Pos);

    NVIC_ClearPendingIRQ(RTC1_IRQn);
    NVIC_EnableIRQ(RTC1_IRQn);

    NRF_RTC1->TASKS_START = 1;
    nrf_delay_us(MAX_RTC_TASKS_DELAY);
}


/**@brief Stops the RTC1 timer.
 */
static void lerbit_rtc1_stop(void)
{
    NVIC_DisableIRQ(RTC1_IRQn);

    NRF_RTC1->EVTENCLR = (RTC_EVTEN_COMPARE0_Enabled << RTC_EVTEN_COMPARE0_Pos)
	                      |(RTC_EVTEN_COMPARE1_Enabled << RTC_EVTEN_COMPARE1_Pos)
	                      |(RTC_EVTEN_COMPARE2_Enabled << RTC_EVTEN_COMPARE2_Pos)
	                      |(RTC_EVTEN_COMPARE3_Enabled << RTC_EVTEN_COMPARE3_Pos);
    NRF_RTC1->INTENCLR = (RTC_EVTEN_COMPARE0_Enabled << RTC_EVTEN_COMPARE0_Pos)
	                      |(RTC_EVTEN_COMPARE1_Enabled << RTC_EVTEN_COMPARE1_Pos)
	                      |(RTC_EVTEN_COMPARE2_Enabled << RTC_EVTEN_COMPARE2_Pos)
	                      |(RTC_EVTEN_COMPARE3_Enabled << RTC_EVTEN_COMPARE3_Pos);

    NRF_RTC1->TASKS_STOP = 1;
    nrf_delay_us(MAX_RTC_TASKS_DELAY);
}

static void lerbit_pwm_init(void)
{
    //GPIO
	  nrf_gpio_cfg_output(LERBIT_CHERRY_LED_PIN_NO);
	  nrf_gpio_cfg_output(LERBIT_CHERRY_BUZZER_PIN_NO);

    nrf_gpiote_task_config(0, LERBIT_CHERRY_LED_PIN_NO, NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW);
    nrf_gpiote_task_config(1, LERBIT_CHERRY_BUZZER_PIN_NO, NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW);

	  //PPI
    NRF_PPI->CH[0].EEP = (uint32_t)&NRF_RTC1->EVENTS_COMPARE[0];
    NRF_PPI->CH[0].TEP = (uint32_t)&NRF_GPIOTE->TASKS_OUT[0];
    NRF_PPI->CH[1].EEP = (uint32_t)&NRF_RTC1->EVENTS_COMPARE[1];
    NRF_PPI->CH[1].TEP = (uint32_t)&NRF_GPIOTE->TASKS_OUT[0];

    NRF_PPI->CH[2].EEP = (uint32_t)&NRF_RTC1->EVENTS_COMPARE[2];
    NRF_PPI->CH[2].TEP = (uint32_t)&NRF_GPIOTE->TASKS_OUT[1];
    NRF_PPI->CH[3].EEP = (uint32_t)&NRF_RTC1->EVENTS_COMPARE[3];
    NRF_PPI->CH[3].TEP = (uint32_t)&NRF_GPIOTE->TASKS_OUT[1];
    
    NRF_PPI->CHEN = (PPI_CHEN_CH0_Enabled << PPI_CHEN_CH0_Pos)
                  | (PPI_CHEN_CH1_Enabled << PPI_CHEN_CH1_Pos)
                  | (PPI_CHEN_CH2_Enabled << PPI_CHEN_CH2_Pos)
                  | (PPI_CHEN_CH3_Enabled << PPI_CHEN_CH3_Pos);

}

static void lerbit_cherry_led_contrl(LERBIT_LED_STATUS led_status)
{
  switch(led_status)
  {
    case LERBIT_LED_Advertising:
      break;
    case LERBIT_LED_Connected:
      break;
    case LERBIT_LED_Mild_Alert:
      break;
    case LERBIT_LED_High_Alert:
      break;
    case LERBIT_LED_Slow_Advertising:
      break;
    case LERBIT_LED_Immediate_Alert:
      break;
    case LERBIT_LED_Whitelist_Advertising:
      break;
    case LERBIT_LED_Asserted:
      break;
    default:
      break;
  }
}

/**@brief Button event handler.
 *
 * @param[in]   pin_no   The pin number of the button pressed.
 */
static void button_event_handler(uint8_t pin_no)
{
    uint32_t err_code;

    switch (pin_no)
    {
        case LERBIT_CHERRY_BUTTON_PIN_NO:
            break;
        default:
            APP_ERROR_HANDLER(pin_no);
    }
}

/**@brief Initialize GPIOTE handler module.
 */
static void gpiote_init(void)
{
    APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
}


/**@brief Initialize button handler module.
 */
static void buttons_init(void)
{
    static app_button_cfg_t buttons[] =
    {
        {LERBIT_CHERRY_BUTTON_PIN_NO,           false, NRF_GPIO_PIN_NOPULL, button_event_handler}
    };
    
//    APP_BUTTON_INIT(buttons, sizeof(buttons) / sizeof(buttons[0]), BUTTON_DETECTION_DELAY, false);
}

/**@brief LEDs initialization.
 *
 * @details Initializes all LEDs used by this application.
 */
static void leds_init(void)
{
    GPIO_LED_CONFIG(PEER_SRV_DISC_LED_PIN_NO);
}

int main(void)
{
    int count = 0, loop = 0;
    leds_init();
    lerbit_rtc1_init();
    lerbit_pwm_init();
    lerbit_rtc1_start();

    nrf_gpio_pin_set(PEER_SRV_DISC_LED_PIN_NO);
    
	  for(loop = 0; loop < 200; loop++) {
        count = 30000;
        while(count--){}
		}


    nrf_gpio_pin_clear(PEER_SRV_DISC_LED_PIN_NO);

    while(1){}
}
