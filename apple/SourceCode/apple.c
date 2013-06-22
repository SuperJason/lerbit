#include "spi_master.h"
#include "twi_master.h"
#include "nrf_gpio.h"
#include "lerbit_oled.h"

#define LERBIT_OLED_RES_PIN 8
#define LERBIT_OLED_I2C_ADDR 0x78
#define LERBIT_OLED_W (LERBIT_OLED_I2C_ADDR)
#define LERBIT_OLED_R (LERBIT_OLED_I2C_ADDR | TWI_READ_BIT)


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
  twi_buf[0] = 0x00; /* Control Byte */
  twi_buf[1] = data; /* Data Value */

	twi_master_transfer(LERBIT_OLED_W, twi_buf, 2, 1);
}

void lerbit_oled_refresh_gram(void)
{
	uint8_t i, n;
	for(i = 0; i < 8; i++)
	{
		lerbit_oled_write_command(0xb0 + i);    //设置页地址（0~7）
		lerbit_oled_write_command(0x00);      //设置显示位置—列低地址
		lerbit_oled_write_command(0x10);      //设置显示位置—列高地址
		for(n = 0; n < 128; n++)
      lerbit_oled_write_command(lerbit_oled_gram[n][i]);
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
	lerbit_oled_write_command(0xAE); //关闭显示
	lerbit_oled_write_command(0xD5); //设置时钟分频因子,震荡频率
	lerbit_oled_write_command(80);   //[3:0],分频因子;[7:4],震荡频率
	lerbit_oled_write_command(0xA8); //设置驱动路数
	lerbit_oled_write_command(0x3F); //默认0x3F(1/64)
	lerbit_oled_write_command(0xD3); //设置显示偏移
	lerbit_oled_write_command(0x00); //默认为0

	lerbit_oled_write_command(0x40); //设置显示开始行 [5:0],行数.

	lerbit_oled_write_command(0x8D); //电荷泵设置
	lerbit_oled_write_command(0x14); //bit2，开启/关闭
	lerbit_oled_write_command(0x20); //设置内存地址模式
	lerbit_oled_write_command(0x02); //[1:0],00，列地址模式;01，行地址模式;10,页地址模式;默认10;
	lerbit_oled_write_command(0xA1); //段重定义设置,bit0:0,0->0;1,0->127;
	lerbit_oled_write_command(0xC0); //设置COM扫描方向;bit3:0,普通模式;1,重定义模式 COM[N-1]->COM0;N:驱动路数
	lerbit_oled_write_command(0xDA); //设置COM硬件引脚配置
	lerbit_oled_write_command(0x12); //[5:4]配置

	lerbit_oled_write_command(0x81); //对比度设置
	lerbit_oled_write_command(0xEF); //1~255;默认0X7F (亮度设置,越大越亮)
	lerbit_oled_write_command(0xD9); //设置预充电周期
	lerbit_oled_write_command(0xf1); //[3:0],PHASE 1;[7:4],PHASE 2;
	lerbit_oled_write_command(0xDB); //设置VCOMH 电压倍率
	lerbit_oled_write_command(0x30); //[6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;

	lerbit_oled_write_command(0xA4); //全局显示开启;bit0:1,开启;0,关闭;(白屏/黑屏)
	lerbit_oled_write_command(0xA6); //设置显示方式;bit0:1,反相显示;0,正常显示
	lerbit_oled_write_command(0xAF); //开启显示
	lerbit_oled_clear();
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

int main(void)
{
  uint8_t data;
		
	twi_master_init();
	spi_master_init(SPI0, SPI_MODE3, 0);

  lerbit_oled_init();

  lerbit_oled_showstring(12, 11, (const uint8_t *)"Lerbit Studio");
  lerbit_oled_showstring(16, 37, (const uint8_t *)"Welcome You!");
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
