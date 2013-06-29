#include <stdio.h>
#include "spi_master.h"
#include "twi_master.h"
#include "nrf_gpio.h"
#include "lerbit_oled.h"
#include "app_util.h"
#include "app_timer.h"
#include "app_gpiote.h" /* app_button use this module */
#include "app_button.h"
#include "nordic_common.h" /* UNUSED_PARAMETER */
#include "lerbit_ped.h"

#define LERBIT_OLED_RES_PIN 8
#define LERBIT_OLED_I2C_ADDR 0x78
#define LERBIT_OLED_W (LERBIT_OLED_I2C_ADDR)
#define LERBIT_OLED_R (LERBIT_OLED_I2C_ADDR | TWI_READ_BIT)

#define LERBIT_APPLE_PWRO_PIN_NO 1
#define LERBIT_APPLE_LEDS_1_PIN_NO 28
#define LERBIT_APPLE_LEDS_2_PIN_NO 29
#define LERBIT_APPLE_LEDS_3_PIN_NO 30
#define LERBIT_APPLE_LEDS_4_PIN_NO 0

/* Timer */
#define APP_TIMER_PRESCALER               0    /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS              6    /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE           4    /**< Size of timer operation queues. */

#define ACC_TIMER_INTERVAL                APP_TIMER_TICKS(40, APP_TIMER_PRESCALER)        /**< Acc sensor timer interval (ticks). */

static app_timer_id_t                     m_acc_timer_id;   /**< Acc sensor timer. */

typedef enum {
    LERBIT_SYS_DIS_INIT,
    LERBIT_SYS_DIS_UPDATE,
    LERBIT_SYS_IDLE 
} lerbit_sys_status_t;

static lerbit_sys_status_t lerbit_sys_status = LERBIT_SYS_DIS_INIT;
uint8_t lerbit_oled_gram[128][4]; /* 128 * 32 */

/**@brief Error handler function, which is called when an error has occurred. 
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze 
 *          how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name. 
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
  /* ?? */

  // This call can be used for debug purposes during development of an application.
  // @note CAUTION: Activating this code will write the stack to flash on an error.
  //                This function should NOT be used in a final product.
  //                It is intended STRICTLY for development/debugging purposes.
  //                The flash write will happen EVEN if the radio is active, thus interrupting
  //                any communication.
  //                Use with care. Un-comment the line below to use.
  // ble_debug_assert_handler(error_code, line_num, p_file_name);

  // On assert, the system can only recover on reset
  NVIC_SystemReset();
}

void lerbit_oled_write_command(uint8_t cmd)
{
  uint8_t twi_buf[2];
  twi_buf[0] = 0x80; /* Control Byte */
  twi_buf[1] = cmd; /* Command Value */

	twi_master_transfer(LERBIT_OLED_W, twi_buf, 2, 1);
}

void lerbit_oled_write_data(uint8_t data)
{
  uint8_t twi_buf[2];
  twi_buf[0] = 0xC0; /* Control Byte */
  twi_buf[1] = data; /* Data Value */

	twi_master_transfer(LERBIT_OLED_W, twi_buf, 2, 1);
}

void lerbit_oled_refresh_gram(void)
{
	uint8_t i, n;
	for(i = 0; i < 4; i++)
	{
		lerbit_oled_write_command(0xb0 + i);    //设置页地址（0~7）
		lerbit_oled_write_command(0x00);      //设置显示位置—列低地址
		lerbit_oled_write_command(0x10);      //设置显示位置—列高地址
		for(n = 0; n < 128; n++)
      lerbit_oled_write_data(lerbit_oled_gram[n][i]);
	}
}

void lerbit_oled_clear(void)
{
	uint8_t i, n;
  for(i = 0; i < 4; i++)
    for(n = 0; n < 128; n++)
      lerbit_oled_gram[n][i]=0x00;
	lerbit_oled_refresh_gram();//更新显示
}

//画点
//x:0~127
//y:0~31
//t:1 填充 0,清空
void lerbit_oled_drawpoint(uint8_t x,uint8_t y, uint8_t t)
{
	uint8_t pos, bx, temp=0;

	if (x > 127 || y > 31)
    return;//超出范围了.

	pos = 3 - y / 8;
	bx = y % 8;
	temp = 1 << (7 - bx);

	if(t)
    lerbit_oled_gram[x][pos] |= temp;
	else
    lerbit_oled_gram[x][pos] &= ~temp;
}

//在指定位置显示一个字符,包括部分字符
//x:0~127
//y:0~63
//mode:0,反白显示;1,正常显示
//size:选择字体 16/12
void lerbit_oled_showchar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size, uint8_t mode)
{
	uint8_t temp, t, t1;
	uint8_t y0 = y;
	chr = chr - ' ';//得到偏移后的值

  for(t = 0; t < size; t++)
  {
    if(size == 12)
      temp=asc2_1206[chr][t]; //调用1206字体
    else
      temp=asc2_1608[chr][t];	//调用1608字体

    for(t1 = 0; t1 < 8; t1++)
    {
      if(temp & 0x80)
        lerbit_oled_drawpoint(x, y, mode);
      else
        lerbit_oled_drawpoint(x, y, !mode);
      temp <<= 1;
      y++;
      if((y - y0) == size)
      {
        y=y0;
        x++;
        break;
      }
    }
  }
}

//显示字符串
//x,y:起点坐标  
//*p:字符串起始地址
//用16字体
void lerbit_oled_showstring(uint8_t x, uint8_t y, const uint8_t *p)
{
#define MAX_CHAR_POSX 122
#define MAX_CHAR_POSY 58          
    while(*p != '\0')
    {       
        if(x > MAX_CHAR_POSX) {
          x = 0;
          y += 16;
        }

        if(y > MAX_CHAR_POSY) {
          y=x=0;
          lerbit_oled_clear();
        }

        lerbit_oled_showchar(x ,y, *p, 16, 1);	 
        x += 8;
        p++;
    }  
}	   

void value_to_string(uint8_t *str, uint8_t value)
{
  uint8_t temp_value;

  temp_value = value >> 4;
  if (temp_value > 9)
    str[0] = (temp_value - 10) + 'A';
  else
    str[0] = temp_value + '0';

  temp_value = value & 0x0F;
  if (temp_value > 9)
    str[1] = (temp_value - 10) + 'A';
  else
    str[1] = temp_value + '0';
}

void lerbit_disp_strcopy(uint8_t *str_buf, const uint8_t *str_data, int len)
{
  if (len > 0 && len < 21)
    while(len--) { 
      str_buf[len] = str_data[len];
    }
}

void lerbit_oled_init(void)
{

  lerbit_oled_write_command(0xAE); //Display OFF (sleep mode)
  lerbit_oled_write_command(0xD5); //Set Display Clock Divide Ratio/Oscillator Frequency
  lerbit_oled_write_command(0x80);
  lerbit_oled_write_command(0xA8); //Set Multiplex Ratio
  lerbit_oled_write_command(0x1F); //32
  lerbit_oled_write_command(0xD3); //Set Display Offset
  lerbit_oled_write_command(0x00);
  lerbit_oled_write_command(0x40); //Set Display Start Line
  lerbit_oled_write_command(0x8D); //Set Display Offset
  lerbit_oled_write_command(0x14); //14h: Enable Charge Pump //10h: Disable Charge Pump
  lerbit_oled_write_command(0xA1); //Set Segment Re-map
  lerbit_oled_write_command(0xC0); //Set COM Output Scan Direction
  lerbit_oled_write_command(0xDA); //Set COM Pins Hardware Configuration
  lerbit_oled_write_command(0x02);
  lerbit_oled_write_command(0x81); //Set Contrast Control
  lerbit_oled_write_command(0x68);
  lerbit_oled_write_command(0xD9); //Set Pre-charge Period
  lerbit_oled_write_command(0xF1);
  lerbit_oled_write_command(0xDB); //Set VCOMH Deselect Level
  lerbit_oled_write_command(0x40);
  lerbit_oled_write_command(0xA4); //Output follows RAM content
  lerbit_oled_write_command(0xA6); //Set Normal Display
  lerbit_oled_clear(); // Clear GRAM to clear display
	lerbit_oled_write_command(0xAF); //Set Display ON (normal mode)
}

void lerbit_acc_reg_read(uint8_t reg_addr, uint8_t *reg_value)
{
	uint8_t spi_tx_data[2], spi_rx_data[2];

  spi_tx_data[0] = (reg_addr & 0x3F) | 0x80; 
  spi_tx_data[1] = 0x00;
  spi_master_tx_rx((uint32_t *)NRF_SPI0, 2, spi_tx_data, spi_rx_data);

  *reg_value = spi_rx_data[1];
}

void lerbit_acc_reg_write(uint8_t reg_addr, uint8_t reg_value)
{
	uint8_t spi_tx_data[2], spi_rx_data[2];


  spi_tx_data[0] = reg_addr & 0x3F; 
  spi_tx_data[1] = reg_value;
  spi_master_tx_rx((uint32_t *)NRF_SPI0, 2, spi_tx_data, spi_rx_data);	
}

/*
static void buttons_init(void)
{
  static app_button_cfg_t buttons[] =
  {
    {LERBIT_CHERRY_BUTTON_PIN_NO,           false, NRF_GPIO_PIN_PULLUP, button_event_handler}
  };

  APP_BUTTON_INIT(buttons, sizeof(buttons) / sizeof(buttons[0]), BUTTON_DETECTION_DELAY, false);
}
*/

static void acc_timer_timeout_handler(void * p_context)
{
  uint8_t data;
  int16_t acc_data_X, acc_data_Y;

  UNUSED_PARAMETER(p_context);
  if (lerbit_sys_status == LERBIT_SYS_IDLE) {
    lerbit_sys_status = LERBIT_SYS_DIS_UPDATE;

    lerbit_acc_reg_read(0x32, &data); /* X */
    acc_data_X = data;
    lerbit_acc_reg_read(0x33, &data);
    acc_data_X |= data << 8;

    lerbit_acc_reg_read(0x34, &data); /* Y */
    acc_data_Y = data;
    lerbit_acc_reg_read(0x35, &data);
    acc_data_Y |= data << 8;

    lerbit_ped_monitor(acc_data_X, acc_data_Y);
  }
}


static void timers_init(void)
{
  uint32_t err_code;

  // Initialize timer module
  APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);

  // Create battery timer
  err_code = app_timer_create(&m_acc_timer_id,
      APP_TIMER_MODE_REPEATED,
      acc_timer_timeout_handler);
  APP_ERROR_CHECK(err_code);

  // Start Acc timer
  err_code = app_timer_start(m_acc_timer_id, ACC_TIMER_INTERVAL, NULL);
  APP_ERROR_CHECK(err_code);
}


static void leds_init(void)
{
	GPIO_LED_CONFIG(LERBIT_APPLE_PWRO_PIN_NO); /* PWRO */
	GPIO_LED_CONFIG(LERBIT_OLED_RES_PIN); /* OLED_RES */
	GPIO_LED_CONFIG(LERBIT_APPLE_LEDS_1_PIN_NO);
	GPIO_LED_CONFIG(LERBIT_APPLE_LEDS_2_PIN_NO);
	GPIO_LED_CONFIG(LERBIT_APPLE_LEDS_3_PIN_NO);
	GPIO_LED_CONFIG(LERBIT_APPLE_LEDS_4_PIN_NO);
}

int main(void)
{
  uint8_t data = 0, disp_count = 0;
	unsigned int loop = 3000000; // about 1s
	uint8_t disp_buff[20] = {0};
		
	twi_master_init();
	spi_master_init(SPI0, SPI_MODE3, 0);
  timers_init();

  /* Start Clock */
  NRF_CLOCK->LFCLKSRC = 1;
  NRF_CLOCK->TASKS_LFCLKSTART = 1;
	
	leds_init();
  lerbit_ped_init();
	
	/* OLED RESET */
	nrf_gpio_pin_set(LERBIT_OLED_RES_PIN);
	loop = 30; /* 10us */
	while(loop--) {}
	nrf_gpio_pin_clear(LERBIT_OLED_RES_PIN);
	loop = 30; /* 10us */
	while(loop--) {}
	nrf_gpio_pin_set(LERBIT_OLED_RES_PIN);
  
  lerbit_oled_init();

  lerbit_oled_showstring(12, 0, (const uint8_t *)"Lerbit Studio");
  lerbit_oled_showstring(16, 16, (const uint8_t *)"Welcome You!");
  lerbit_oled_refresh_gram();
	
	/* light leds */
	loop = 250000; /* 0.1s */
	while(loop--) {}
	nrf_gpio_pin_set(LERBIT_APPLE_LEDS_1_PIN_NO);
	loop = 250000; /* 0.1s */
	while(loop--) {}
	nrf_gpio_pin_set(LERBIT_APPLE_LEDS_2_PIN_NO);
	nrf_gpio_pin_clear(LERBIT_APPLE_LEDS_1_PIN_NO);
	loop = 250000; /* 0.1s */
	while(loop--) {}
  nrf_gpio_pin_set(LERBIT_APPLE_LEDS_3_PIN_NO);
	nrf_gpio_pin_clear(LERBIT_APPLE_LEDS_2_PIN_NO);
	loop = 250000; /* 0.1s */
	while(loop--) {}
  nrf_gpio_pin_set(LERBIT_APPLE_LEDS_4_PIN_NO);
	nrf_gpio_pin_clear(LERBIT_APPLE_LEDS_3_PIN_NO);
	loop = 250000; /* 0.1s */
	while(loop--) {}
	nrf_gpio_pin_clear(LERBIT_APPLE_LEDS_4_PIN_NO);
		
	loop = 1500000; /* 1s */
	while(loop--) {}

	nrf_gpio_pin_set(LERBIT_APPLE_PWRO_PIN_NO); /* set PWRO */
	
  lerbit_oled_clear();
		
  lerbit_acc_reg_write(0x31, 0x00);
  lerbit_acc_reg_write(0x2C, 0x08);
  lerbit_acc_reg_write(0x2D, 0x08);
  lerbit_acc_reg_write(0x2E, 0x80); // ENABLE DATA_READY INTERRUPT.

  lerbit_sys_status = LERBIT_SYS_IDLE;

  while(1) {
    if(lerbit_sys_status == LERBIT_SYS_DIS_UPDATE) {
      lerbit_sys_status = LERBIT_SYS_IDLE;
      lerbit_acc_reg_read(0x00, &data);
      sprintf((char *)disp_buff, "DeviceId: %02X %02X", data, disp_count++);
      /*
      lerbit_disp_strcopy(disp_buff, "DeviceID:      ", 16);
      value_to_string(&disp_buff[10], data);
      value_to_string(&disp_buff[14], disp_count++);
      */
      lerbit_oled_showstring(0, 0 * 16, (const uint8_t *)disp_buff);
      data = 0;
      lerbit_disp_strcopy(disp_buff, "X:     ,Y:     ", 16);
      lerbit_acc_reg_read(0x32, &data); /* X */
      value_to_string(&disp_buff[5], data);
      lerbit_acc_reg_read(0x33, &data);
      value_to_string(&disp_buff[3], data);
      lerbit_acc_reg_read(0x34, &data); /* Y */
      value_to_string(&disp_buff[13], data);
      lerbit_acc_reg_read(0x35, &data);
      value_to_string(&disp_buff[11], data);
      lerbit_oled_showstring(0, 1 * 16, (const uint8_t *)disp_buff);
      lerbit_oled_refresh_gram(); 
    }
  }

  /* while(1) {} */
}
