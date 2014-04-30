#include "rtthread.h"
#include "drv_ssd1306.h"

#define SCL_SET() GPIO_SetBits(GPIOC,GPIO_Pin_14)
#define SCL_CLR() GPIO_ResetBits(GPIOC,GPIO_Pin_14)
#define SDA_SET() GPIO_SetBits(GPIOC,GPIO_Pin_13)
#define SDA_CLR() GPIO_ResetBits(GPIOC,GPIO_Pin_13)
#define RST_SET() GPIO_SetBits(GPIOB,GPIO_Pin_7)
#define RST_CLR() GPIO_ResetBits(GPIOB,GPIO_Pin_7)
#define DC_SET() GPIO_SetBits(GPIOB,GPIO_Pin_9)
#define DC_CLR() GPIO_ResetBits(GPIOB,GPIO_Pin_9)

#define  xsize 128
#define ysize 64
  /*Һ����������*/
static sFONT *LCD_Currentfonts;
static uint8_t framebuffer[xsize * ysize / 8];

static void ssd1306_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB| RCC_APB2Periph_GPIOC,ENABLE);

	PWR_BackupAccessCmd(ENABLE);
        
        RCC_LSEConfig(RCC_LSE_OFF);
        
//        BKP_TamperPinCmd(DISABLE);        
//          RCC_LSICmd(DISABLE);
//  BKP_TamperPinCmd(DISABLE);
        PWR_BackupAccessCmd(DISABLE);
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7|GPIO_Pin_9;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13|GPIO_Pin_14;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
 SCL_SET();
 SDA_SET();
 DC_CLR();
 RST_SET();
}
static void ssd1306_write_command(uint8_t command)
{
	uint8_t i;
	DC_CLR();
  for(i=0;i<8;i++)
  {
   if(command&0x80)
   {
    SDA_SET();
   }else
   {
    SDA_CLR();
   }
   SCL_CLR();
   SCL_SET();
   command<<=1;
  }
}
static  void ssd1306_write_data(uint8_t data)
{
	uint8_t i;
	DC_SET();
  for(i=0;i<8;i++)
  {
   if(data&0x80)
   {
    SDA_SET();
   }else
   {
    SDA_CLR();
   }
   SCL_CLR();
   SCL_SET();
   data<<=1;
  }
}
/**
 * \brief Read data from the controller
 *
 * \note The controller does not support read in serial mode.
 *
 * \retval 8 bit data read from the controller
 */
//static uint8_tssd1306_read_data(void)
//{
//	return 0;
//}

/**
 * \brief Read status from the controller
 *
 * \note The controller does not support read in serial mode.
 *
 * \retval 8 bit status read from the controller
 */
//static  uint8_t ssd1306_get_status(void)
//{
//	return 0;
//}

static void ssd1306_hard_reset(void)
{
  RST_CLR();
	rt_thread_delay(SSD1306_LATENCY); // At least 3us
	RST_SET();
	rt_thread_delay(SSD1306_LATENCY); // At least 3us
}

//static void ssd1306_sleep_enable(void)
//{
//	ssd1306_write_command(SET_DISPLAY_OFF);
//}


//static void ssd1306_sleep_disable(void)
//{
//	ssd1306_write_command(SET_DISPLAY_ON);
//}

static  void ssd1306_set_page_address(uint8_t address)
{
	// Make sure that the address is 4 bits (only 8 pages)
	address &= 0x0F;
	ssd1306_write_command(SET_PAGE_START_ADDRESS(address));
}


static  void ssd1306_set_column_address(uint8_t address)
{
	// Make sure the address is 7 bits
	address &= 0x7F;
	ssd1306_write_command(SET_HIGH_COL(address >> 4));
	ssd1306_write_command(SET_LOW_COL(address & 0x0F));
}

//static  void ssd1306_set_display_start_line_address(uint8_t address)
//{
//	// Make sure address is 6 bits
//	address &= 0x3F;
//	ssd1306_write_command(SET_START_LINE(address));
//}

static  void ssd1306_display_on(void)
{
	ssd1306_write_command(SET_DISPLAY_ON);
}

//static  void ssd1306_display_off(void)
//{
//	ssd1306_write_command(SET_DISPLAY_OFF);
//}


static  uint8_t ssd1306_set_contrast(uint8_t contrast)
{
	ssd1306_write_command(SET_CONTRAST_CONTROL_FOR_BANK0);
	ssd1306_write_command(contrast);
	return contrast;
}


//static  void ssd1306_display_invert_enable(void)
//{
//	ssd1306_write_command(SET_INVERSE_DISPLAY);
//}

static void ssd1306_display_invert_disable(void)
{
	ssd1306_write_command(SET_NORMAL_DISPLAY);
}
 void ssd1306_update(void)
{ 	uint8_t page = 0;
	uint8_t col = 0;
	for (page = 0; page < 8; ++page)
	{
		ssd1306_set_page_address(page);
		ssd1306_set_column_address(0);
		for (col = 0; col < 128; ++col)
		{
			ssd1306_write_data(framebuffer[page*128+col]);
		}
	}
}

static void ssd1306_clear(uint8_t data)
{
	uint8_t page = 0;
	uint8_t col = 0;

	for (page = 0; page < 8; ++page)
	{
		ssd1306_set_page_address(page);
		ssd1306_set_column_address(0);
		for (col = 0; col < 128; ++col)
		{
			ssd1306_write_data(data);
		}
	}
}

void ssd1306_hw_init(void)
{
	ssd1306_gpio_init();
	// Do a hard reset of the OLED display controller
	ssd1306_hard_reset();

	// Initialize the interface
	ssd1306_gpio_init();

	// 1/32 Duty (0x0F~0x3F)
	ssd1306_write_command(SET_MULTIPLEX_RATIO);
	ssd1306_write_command(0x3F);

	// Shift Mapping RAM Counter (0x00~0x3F)
	ssd1306_write_command(SET_DISPLAY_OFFSET);
	ssd1306_write_command(0x00);

	// Set Mapping RAM Display Start Line (0x00~0x3F)
	ssd1306_write_command(SET_START_LINE(0x00));

	// Set Column Address 0 Mapped to SEG0
	ssd1306_write_command(SET_SEGMENT_RE_MAP_COL0_SEG0);

	// Set COM/Row Scan Scan from COM63 to 0
	ssd1306_write_command(SET_COM_OUTPUT_SCAN_UP);

	// Set COM Pins hardware configuration
	ssd1306_write_command(SET_COM_PINS);
	ssd1306_write_command(0x12);

	ssd1306_set_contrast(0xCF);

	// Disable Entire display On
	ssd1306_write_command(ENTIRE_DISPLAY_AND_GDDRAM_ON);

	ssd1306_display_invert_disable();

	// Set Display Clock Divide Ratio / Oscillator Frequency (Default => 0x80)
	ssd1306_write_command(SET_DISPLAY_CLOCK_DIVIDE_RATIO);
	ssd1306_write_command(0x80);

	// Enable charge pump regulator
	ssd1306_write_command(SET_CHARGE_PUMP_SETTING);
	ssd1306_write_command(0x14);

	// Set VCOMH Deselect Level
	ssd1306_write_command(SET_VCOMH_DESELECT_LEVEL);
	ssd1306_write_command(0x40); // Default => 0x20 (0.77*VCC)

	// Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
	ssd1306_write_command(SET_PRE_CHARGE_PERIOD);
	ssd1306_write_command(0xF1);

	ssd1306_display_on();
	ssd1306_clear(WHITE);
}

void glcd_init(void)
{
	ssd1306_hw_init();
	glcd_set_font(&Font8x16);
}
/**************************************************************************************************************
 * ��������glcd_set_font()
 * ����  ��sFONT *fonts Ҫ���õ�����
 * ���  ��void
 * ����  ������LCD������
 * ����  ���ⲿ����        
 *****************************************************************************************************************/
void glcd_set_font(sFONT *fonts)
{
  LCD_Currentfonts = fonts;
}
/**************************************************************************************************************
 * ��������glcd_get_font()
 * ����  ��void
 * ���  ��sFONT * ��ȡ����
 * ����  ������LCD������
 * ����  ���ⲿ����        
 *****************************************************************************************************************/
sFONT* glcd_get_font(void)
{
  return LCD_Currentfonts;
}
void glcd_draw_pixel(uint16_t x, uint16_t y,color_t color)
{ 
	if( x > xsize ||  y > ysize)
	{
		return;  
	}
	{
	 uint8_t page=y/8;
 uint8_t row=y%8;
 if(color==BLACK)
	 {
	  framebuffer[page*128+x]&=~(0x01<<row);
   }else
	 {
     framebuffer[page*128+x]|=(0x01<<row);
   }
  }
}
void glcd_clear(color_t color)
{
ssd1306_clear(color);
}

/**************************************************************************************************************
 * ��������glcd_draw_hline()
 * ����  ��uint16_t Xpos, uint16_t Ypos, uint16_t Length ���X��Y���꼰����
 * ���  ��void
 * ����  ����ˮƽ��
 * ����  ���ⲿ����        
 *****************************************************************************************************************/
void glcd_draw_hline(uint16_t Xpos, uint16_t Ypos, uint16_t Length,color_t color)
{  
	 uint8_t page=Ypos/8;
 uint8_t row=Ypos%8;
 while(Length--){
 if(color==BLACK)
	 {
	  framebuffer[page*128+Xpos]&=~(0x01<<row);
   }else
	 {
     framebuffer[page*128+Xpos]|=(0x01<<row);
   }
	 Xpos++;
 }
}
/**************************************************************************************************************
 * ��������glcd_draw_vline()
 * ����  ��uint16_t Xpos, uint16_t Ypos, uint16_t Length ���X��Y���꼰����
 * ���  ��void
 * ����  ������ֱ��
 * ����  ���ⲿ����        
 *****************************************************************************************************************/
void glcd_draw_vline(uint16_t Xpos, uint16_t Ypos, uint16_t Length,color_t color)
{
 while(Length--){
	  uint8_t page=Ypos/8;
 uint8_t row=Ypos%8;
 if(color==BLACK)
	 {
	  framebuffer[page*128+Xpos]&=~(0x01<<row);
   }else
	 {
     framebuffer[page*128+Xpos]|=(0x01<<row);
   }
	 Ypos++;
 }
}
/**************************************************************************************************************
 * ��������glcd_draw_rect()
 * ����  ��uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint8_t Height �������Ͻǵ�����꼰��͸�
 * ���  ��void
 * ����  �������κ���
 * ����  ���ⲿ����        
 *****************************************************************************************************************/
void glcd_draw_rect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint8_t Height,color_t color)
{
  glcd_draw_hline(Xpos, Ypos, Width,color);
  glcd_draw_hline(Xpos, Ypos+ Height, Width,color); 
  glcd_draw_vline(Xpos, Ypos, Height,color);
  glcd_draw_vline(Xpos+ Width,Ypos, Height,color);
 
}
/**************************************************************************************************************
 * ��������glcd_draw_circle()
 * ����  ��uint16_t Xpos, uint16_t Ypos, uint16_t Radius Բ������㼰�뾶
 * ���  ��void
 * ����  ����Բ����
 * ����  ���ⲿ����        
 *****************************************************************************************************************/
void glcd_draw_circle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius,color_t color)
{
  int32_t  D;/* Decision Variable */ 
  uint32_t  CurX;/* Current X Value */
  uint32_t  CurY;/* Current Y Value */ 
  
  D = 3 - (Radius << 1);
  CurX = 0;
  CurY = Radius;
  
  while (CurX <= CurY)
  {
	  glcd_draw_pixel(Xpos + CurX, Ypos + CurY,color);
    glcd_draw_pixel(Xpos + CurX, Ypos - CurY,color);
     glcd_draw_pixel(Xpos - CurX, Ypos + CurY,color);
     glcd_draw_pixel(Xpos - CurX, Ypos - CurY,color);
     glcd_draw_pixel(Xpos + CurY, Ypos + CurX,color);
     glcd_draw_pixel(Xpos + CurY, Ypos - CurX,color);
     glcd_draw_pixel(Xpos - CurY, Ypos + CurX,color);
     glcd_draw_pixel(Xpos - CurY, Ypos - CurX,color);
    if (D < 0)
    { 
      D += (CurX << 2) + 6;
    }
    else
    {
      D += ((CurX - CurY) << 2) + 10;
      CurY--;
    }
    CurX++;
  }
}
/**************************************************************************************************************
 * ��������glcd_fill_rect()
 * ����  ��uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height ���������Ͻǵ㡢��͸�
 * ���  ��void
 * ����  ����һ�����ľ���
 * ����  ���ⲿ����        
 *****************************************************************************************************************/
void glcd_fill_rect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height,color_t color)
{ 
  glcd_draw_hline(Xpos, Ypos, Width,color);
  glcd_draw_hline(Xpos, Ypos+ Height, Width,color);
  
  glcd_draw_vline(Xpos, Ypos, Height,color);
  glcd_draw_vline(Xpos+Width, Ypos, Height,color);
  Width --;
  Height--;
  Xpos++;
  while(Height--)
  {
    glcd_draw_hline(Xpos, ++Ypos, Width,color);    
  }
}
/**************************************************************************************************************
 * ��������glcd_fill_circle()
 * ����  ��uint16_t Xpos, uint16_t Ypos, uint16_t Radius ���Բ��Բ�ĺͰ뾶
 * ���  ��void
 * ����  ����һ�����Բ
 * ����  ���ⲿ����        
 *****************************************************************************************************************/
void glcd_fill_circle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius,color_t color)
{
  int32_t  D;    /* Decision Variable */ 
  uint32_t  CurX;/* Current X Value */
  uint32_t  CurY;/* Current Y Value */ 
  D = 3 - (Radius << 1);

  CurX = 0;
  CurY = Radius;

  while (CurX <= CurY)
  {
    if(CurY > 0) 
    {
      glcd_draw_hline(Xpos - CurY, Ypos - CurX, 2*CurY,color);
      glcd_draw_hline(Xpos - CurY, Ypos + CurX, 2*CurY,color);
    }

    if(CurX > 0) 
    {
      glcd_draw_hline(Xpos - CurX, Ypos -CurY, 2*CurX,color);
      glcd_draw_hline(Xpos - CurX, Ypos + CurY, 2*CurX,color);
    }
    if (D < 0)
    { 
      D += (CurX << 2) + 6;
    }
    else
    {
      D += ((CurX - CurY) << 2) + 10;
      CurY--;
    }
    CurX++;
  }
  glcd_draw_circle(Xpos, Ypos, Radius,color);
}
/**************************************************************************************************************
 * ��������glcd_draw_uniline()
 * ����  ��uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2 ��ʼ��������յ�����
 * ���  ��void
 * ����  �������ⷽ���ֱ��
 * ����  ���ⲿ����        
 *****************************************************************************************************************/
void glcd_draw_uniline(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,color_t color)
{
  int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0, 
  yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0, 
  curpixel = 0;
  
  deltax = ABS(x2 - x1);        /* The difference between the x's */
  deltay = ABS(y2 - y1);        /* The difference between the y's */
  x = x1;                       /* Start x off at the first pixel */
  y = y1;                       /* Start y off at the first pixel */
  
  if (x2 >= x1)                 /* The x-values are increasing */
  {
    xinc1 = 1;
    xinc2 = 1;
  }
  else                          /* The x-values are decreasing */
  {
    xinc1 = -1;
    xinc2 = -1;
  }
  
  if (y2 >= y1)                 /* The y-values are increasing */
  {
    yinc1 = 1;
    yinc2 = 1;
  }
  else                          /* The y-values are decreasing */
  {
    yinc1 = -1;
    yinc2 = -1;
  }
  
  if (deltax >= deltay)         /* There is at least one x-value for every y-value */
  {
    xinc1 = 0;                  /* Don't change the x when numerator >= denominator */
    yinc2 = 0;                  /* Don't change the y for every iteration */
    den = deltax;
    num = deltax / 2;
    numadd = deltay;
    numpixels = deltax;         /* There are more x-values than y-values */
  }
  else                          /* There is at least one y-value for every x-value */
  {
    xinc2 = 0;                  /* Don't change the x for every iteration */
    yinc1 = 0;                  /* Don't change the y when numerator >= denominator */
    den = deltay;
    num = deltay / 2;
    numadd = deltax;
    numpixels = deltay;         /* There are more y-values than x-values */
  }
  
  for (curpixel = 0; curpixel <= numpixels; curpixel++)
  {
    glcd_draw_pixel(x, y,color);             /* Draw the current pixel */
    num += numadd;              /* Increase the numerator by the top of the fraction */
    if (num >= den)             /* Check if numerator >= denominator */
    {
      num -= den;               /* Calculate the new numerator value */
      x += xinc1;               /* Change the x as appropriate */
      y += yinc1;               /* Change the y as appropriate */
    }
    x += xinc2;                 /* Change the x as appropriate */
    y += yinc2;                 /* Change the y as appropriate */
  }
}

/**************************************************************************************************************
 * ��������glcd_draw_char()
 * ����  ��const uint16_t *c   �ַ�����
 * ���  ��void
 * ����  ��LCD��һ���ַ�
 * ����  ���ⲿ����        
 *****************************************************************************************************************/
void glcd_draw_char(uint16_t Xpos, uint16_t Ypos, const uint16_t *c,color_t color)
{
  uint32_t index = 0, i = 0;
  uint16_t  x = 0,y=0;
  y = Ypos;
  
  for(index = 0; index < LCD_Currentfonts->Height; index++)
  { 
	 x=Xpos;
    for(i = 0; i < LCD_Currentfonts->Width; i++)
    {
  
      if((((c[index] & ((0x80 << ((LCD_Currentfonts->Width / 12 ) * 8 ) ) >> i)) == 0x00) &&(LCD_Currentfonts->Width <= 12))||
        (((c[index] & (0x1 << i)) == 0x00)&&(LCD_Currentfonts->Width > 12 )))

      {
		  glcd_draw_pixel(x++,y,!color);
      }
      else
      {
       glcd_draw_pixel(x++,y,color);
      } 
    }
    y++;
   
  }

}
/**************************************************************************************************************
 * ��������glcd_display_char()
 * ����  ��uint16_t Xpos, uint16_t Ypos, uint8_t Ascii ��ʾ��λ�ú��ַ�
 * ���  ��void
 * ����  ��LCD��ʾһ���ַ�
 * ����  ���ⲿ����        
 *****************************************************************************************************************/
void glcd_display_char(uint16_t Xpos, uint16_t Ypos, uint8_t Ascii,color_t color)
{
  Ascii -= 32;
  glcd_draw_char(Xpos, Ypos, &LCD_Currentfonts->table[Ascii * LCD_Currentfonts->Height],color);
}
/**************************************************************************************************************
 * ��������glcd_draw_string()
 * ����  ��u16 xpos, u16 ypos, u8 *ptr ��ʾ��λ�ú��ַ���
 * ���  ��void
 * ����  ��LCD��ʾһ���ַ�
 * ����  ���ⲿ����        
 *****************************************************************************************************************/
void glcd_draw_string(uint16_t xpos, uint16_t ypos, uint8_t *ptr,color_t color)
{
  	uint16_t refypos=xpos;
  	while(*ptr!=0)
  	{
		glcd_display_char(refypos,ypos,*ptr,color);
    	refypos+=LCD_Currentfonts->Width;
    	ptr++;
  	}
}
void glcd_test()
{
 glcd_init();
 glcd_draw_string(0,0,"<<SSD1306 OLED>>",WHITE);
	glcd_draw_string(0,16,"Hello,SAM4N",WHITE);
	glcd_draw_string(0,32,"www.eeboard.com",WHITE);
	glcd_draw_string(0,48,"oled font test",WHITE);
	glcd_update();
}
