#include "spi_master.h"
#include "twi_master.h"
#include "nrf_gpio.h"
#include "lerbit_oled.h"
#include "app_util.h"

#define LERBIT_OLED_RES_PIN 8
#define LERBIT_OLED_I2C_ADDR 0x78
#define LERBIT_OLED_W (LERBIT_OLED_I2C_ADDR)
#define LERBIT_OLED_R (LERBIT_OLED_I2C_ADDR | TWI_READ_BIT)

#define LERBIT_APPLE_PWRO_PIN_NO 1
#define LERBIT_APPLE_LEDS_1_PIN_NO 28
#define LERBIT_APPLE_LEDS_2_PIN_NO 29
#define LERBIT_APPLE_LEDS_3_PIN_NO 30
#define LERBIT_APPLE_LEDS_4_PIN_NO 0



uint8_t lerbit_oled_gram[128][4]; /* 128 * 32 */

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


  spi_tx_data[0] = reg_addr & 0x3F; 
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
  uint8_t data;
	unsigned int loop = 3000000; // about 1s
		
	twi_master_init();
	spi_master_init(SPI0, SPI_MODE3, 0);
	
	leds_init();
	
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
	
	lerbit_oled_showstring(0, 0 * 16, (const uint8_t *)"0123456789ABCDEF");
  lerbit_oled_showstring(0, 1 * 16, (const uint8_t *)"1123456789ABCDEF");
  lerbit_oled_refresh_gram(); 

		
  data = 0x00;
  lerbit_acc_reg_write(0x31, data);
  data = 0x08;
  lerbit_acc_reg_write(0x2C, data);
  data = 0x08;
  lerbit_acc_reg_write(0x2D, data);
  data = 0x80;
  lerbit_acc_reg_write(0x2E, data);

  lerbit_acc_reg_read(0x00, &data);

  while(1) {}
}
