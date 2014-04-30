#ifndef __SSD1306_H
#define __SSD1306_H

#include <stm32f10x.h>


#include "fonts.h"


#define ABS(X)  ((X) > 0 ? (X) : -(X))
typedef enum
{
BLACK=0x00,
WHITE=0xff
}color_t;

//! \name Fundamental Command defines
//@{
#define SET_LOW_COL(column)             (0x00 | (column))
#define SET_HIGH_COL(column)            (0x10 | (column))
#define SET_MEMORY_ADDRESSING_MODE      0x20
#define SET_COLUMN_ADDRESS              0x21
#define SET_PAGE_ADDRESS                0x22
#define SET_START_LINE(line)            (0x40 | (line))
#define SET_CONTRAST_CONTROL_FOR_BANK0  0x81
#define SET_CHARGE_PUMP_SETTING         0x8D
#define SET_SEGMENT_RE_MAP_COL0_SEG0    0xA0
#define SET_SEGMENT_RE_MAP_COL127_SEG0  0xA1
#define ENTIRE_DISPLAY_AND_GDDRAM_ON    0xA4
#define ENTIRE_DISPLAY_ON               0xA5
#define SET_NORMAL_DISPLAY              0xA6
#define SET_INVERSE_DISPLAY             0xA7
#define SET_MULTIPLEX_RATIO             0xA8
#define SET_DISPLAY_ON                  0xAF
#define SET_DISPLAY_OFF                 0xAE
#define SET_PAGE_START_ADDRESS(page)    (0xB0 | (page & 0x07))
#define SET_COM_OUTPUT_SCAN_UP          0xC0
#define SET_COM_OUTPUT_SCAN_DOWN        0xC8
#define SET_DISPLAY_OFFSET              0xD3
#define SET_DISPLAY_CLOCK_DIVIDE_RATIO  0xD5
#define SET_PRE_CHARGE_PERIOD           0xD9
#define SET_COM_PINS                    0xDA
#define SET_VCOMH_DESELECT_LEVEL        0xDB
#define NOP                             0xE3
//@}
//! \name Graphic Acceleration Command defines
//@{
#define SCROLL_H_RIGHT                  0x26
#define SCROLL_H_LEFT                   0x27
#define CONTINUOUS_SCROLL_V_AND_H_RIGHT 0x29
#define CONTINUOUS_SCROLL_V_AND_H_LEFT  0x2A
#define DEACTIVATE_SCROLL               0x2E
#define ACTIVATE_SCROLL                 0x2F
#define SET_VERTICAL_SCROLL_AREA        0xA3
//@}


/**
 * \name Interface selection
 *
 * The OLED controller support both serial and parallel mode, that means there
 * is a number of possible ways of interfacing the controller using different
 * peripherals. The different interfaces can be selected using different
 * defines. This driver supports the serial communication mode using an
 * USART in Master SPI mode by defining \ref SSD1306_USART_SPI_INTERFACE, and a
 * normal SPI in Master Mode by defining \ref SSD1306_SPI_INTERFACE.
 *
 * \note The current driver only support serial mode.
 */

#define SSD1306_LATENCY 2




void glcd_test(void);
void ssd1306_hw_init(void);
void ssd1306_update(void);
void glcd_init(void);
void draw_line(void);
void glcd_clear(color_t color);
#define glcd_update() ssd1306_update()
/**************************************************************************************************************
 * ��������glcd_set_font()
 * ����  ��sFONT *fonts Ҫ���õ�����
 * ���  ��void
 * ����  ������LCD������
 * ����  ���ⲿ����        
 *****************************************************************************************************************/
void glcd_set_font(sFONT *fonts);
/**************************************************************************************************************
 * ��������glcd_get_font()
 * ����  ��void
 * ���  ��sFONT * ��ȡ����
 * ����  ������LCD������
 * ����  ���ⲿ����        
 *****************************************************************************************************************/
sFONT* glcd_get_font(void);
/**************************************************************************************************************
 * ��������glcd_draw_hline()
 * ����  ��uint16_t Xpos, uint16_t Ypos, uint16_t Length ���X��Y���꼰����
 * ���  ��void
 * ����  ����ˮƽ��
 * ����  ���ⲿ����        
 *****************************************************************************************************************/
void glcd_draw_hline(uint16_t Xpos, uint16_t Ypos, uint16_t Length,color_t color);
/**************************************************************************************************************
 * ��������glcd_draw_vline()
 * ����  ��uint16_t Xpos, uint16_t Ypos, uint16_t Length ���X��Y���꼰����
 * ���  ��void
 * ����  ������ֱ��
 * ����  ���ⲿ����        
 *****************************************************************************************************************/
void glcd_draw_vline(uint16_t Xpos, uint16_t Ypos, uint16_t Length,color_t color);
/**************************************************************************************************************
 * ��������glcd_draw_rect()
 * ����  ��uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint8_t Height �������Ͻǵ�����꼰��͸�
 * ���  ��void
 * ����  �������κ���
 * ����  ���ⲿ����        
 *****************************************************************************************************************/
void glcd_draw_rect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint8_t Height,color_t color);
/**************************************************************************************************************
 * ��������glcd_draw_circle()
 * ����  ��uint16_t Xpos, uint16_t Ypos, uint16_t Radius Բ������㼰�뾶
 * ���  ��void
 * ����  ����Բ����
 * ����  ���ⲿ����        
 *****************************************************************************************************************/
void glcd_draw_circle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius,color_t color);
/**************************************************************************************************************
 * ��������glcd_fill_rect()
 * ����  ��uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height ���������Ͻǵ㡢��͸�
 * ���  ��void
 * ����  ����һ�����ľ���
 * ����  ���ⲿ����        
 *****************************************************************************************************************/
void glcd_fill_rect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height,color_t color);
/**************************************************************************************************************
 * ��������glcd_fill_circle()
 * ����  ��uint16_t Xpos, uint16_t Ypos, uint16_t Radius ���Բ��Բ�ĺͰ뾶
 * ���  ��void
 * ����  ����һ�����Բ
 * ����  ���ⲿ����        
 *****************************************************************************************************************/
void glcd_fill_circle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius,color_t color);
/**************************************************************************************************************
 * ��������glcd_draw_uniline()
 * ����  ��uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2 ��ʼ��������յ�����
 * ���  ��void
 * ����  �������ⷽ���ֱ��
 * ����  ���ⲿ����        
 *****************************************************************************************************************/
void glcd_draw_uniline(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,color_t color);

/**************************************************************************************************************
 * ��������glcd_draw_char()
 * ����  ��const uint16_t *c   �ַ�����
 * ���  ��void
 * ����  ��LCD��һ���ַ�
 * ����  ���ⲿ����        
 *****************************************************************************************************************/
void glcd_draw_char(uint16_t Xpos, uint16_t Ypos, const uint16_t *c,color_t color);
/**************************************************************************************************************
 * ��������glcd_display_char()
 * ����  ��uint16_t Xpos, uint16_t Ypos, uint8_t Ascii ��ʾ��λ�ú��ַ�
 * ���  ��void
 * ����  ��LCD��ʾһ���ַ�
 * ����  ���ⲿ����        
 *****************************************************************************************************************/
void glcd_display_char(uint16_t Xpos, uint16_t Ypos, uint8_t Ascii,color_t color);
/**************************************************************************************************************
 * ��������glcd_draw_string()
 * ����  ��u16 xpos, u16 ypos, u8 *ptr ��ʾ��λ�ú��ַ���
 * ���  ��void
 * ����  ��LCD��ʾһ���ַ�
 * ����  ���ⲿ����        
 *****************************************************************************************************************/
void glcd_draw_string(uint16_t xpos, uint16_t ypos, uint8_t *ptr,color_t color);

#endif 
