#ifndef __LCD_H__
#define __LCD_H__

#include <stdio.h>

#define PIN_clear(port, pin)                port &= ~( 1 << pin )
#define PIN_set(port, pin)                  port |= ( 1 << pin )

#define DISP_STATE_NOP              0
#define DISP_STATE_WAIT             1
#define DISP_STATE_PUTHDATA         2
#define DISP_STATE_PUTLDATA         3
#define DISP_STATE_ENTOGGLE         4
#define DISP_STATE_CLEAR            5
#define DISP_STATE_HOME             6
#define DISP_STATE_ENWAIT           7
#define DISP_STATE_MCURSOR          8

#define DISP_FRONTBUFFER    (unsigned char)   0
#define DISP_BACKBUFFER     (unsigned char)   80

#if ! defined (ROM_READ)

#define ROM_READ pgm_read_byte

#endif



void lcd_command( uint8_t command );
void lcd_write_nibble( uint8_t data );
static void lcd_init( void );
// TODO change defined pointer to void pointer
void put_data_to_lcd_buffer(void * data, uint8_t length, uint8_t row, uint8_t col, uint8_t buffer, uint8_t from_flash);
void put_one_char(unsigned char character, uint8_t length, uint8_t row, uint8_t col, uint8_t buffer);
void disp_clear_buffer(uint8_t buffer);
uint8_t disp_swap_buffers(void);
static uint8_t disp_active_buffer_get( void );
static void process_lcd_FSM( void );


unsigned char disp_linear_buff [160];

struct pins {
    volatile uint8_t RS;
    volatile uint8_t EN;
    volatile uint8_t D4;
    volatile uint8_t D5;
    volatile uint8_t D6;
    volatile uint8_t D7;
} lcd_pins;

#if defined  (LCD_CONTROL_ONE_PORT)

struct ports {
    volatile uint8_t *LCD_PORT;
} lcd_ports;

#define RS_PORT *lcd_ports.LCD_PORT
#define EN_PORT *lcd_ports.LCD_PORT
#define D4_PORT *lcd_ports.LCD_PORT
#define D5_PORT *lcd_ports.LCD_PORT
#define D6_PORT *lcd_ports.LCD_PORT
#define D7_PORT *lcd_ports.LCD_PORT

#define ___PORT_DEFINED___

#endif

#if defined (LCD_CONTROL_TWO_PORT)

struct ports {
    volatile uint8_t *LCD_RS_EN_PORT;
    volatile uint8_t *LCD_DATA_PORT;
} lcd_ports;

#define RS_PORT *lcd_ports.LCD_RS_EN_PORT
#define EN_PORT *lcd_ports.LCD_RS_EN_PORT
#define D4_PORT *lcd_ports.LCD_DATA_PORT
#define D5_PORT *lcd_ports.LCD_DATA_PORT
#define D6_PORT *lcd_ports.LCD_DATA_PORT
#define D7_PORT *lcd_ports.LCD_DATA_PORT

#define ___PORT_DEFINED___

#endif

#if defined (LCD_CONTROL_THREE_PORTS)

struct ports {
    volatile uint8_t *LCD_RS_PORT;
    volatile uint8_t *LCD_EN_PORT;
    volatile uint8_t *LCD_DATA_PORT;
} lcd_ports;

#define RS_PORT *lcd_ports.LCD_RS_PORT
#define EN_PORT *lcd_ports.LCD_EN_PORT
#define D4_PORT *lcd_ports.LCD_DATA_PORT
#define D5_PORT *lcd_ports.LCD_DATA_PORT
#define D6_PORT *lcd_ports.LCD_DATA_PORT
#define D7_PORT *lcd_ports.LCD_DATA_PORT

#define ___PORT_DEFINED___

#endif

#ifndef ___PORT_DEFINED___

struct ports {
    volatile uint8_t *LCD_RS_PORT;
    volatile uint8_t *LCD_EN_PORT;
    volatile uint8_t *LCD_DATA4_PORT;
    volatile uint8_t *LCD_DATA5_PORT;
    volatile uint8_t *LCD_DATA6_PORT;
    volatile uint8_t *LCD_DATA7_PORT;
} lcd_ports;

#define RS_PORT *lcd_ports.LCD_RS_PORT
#define EN_PORT *lcd_ports.LCD_EN_PORT
#define D4_PORT *lcd_ports.LCD_DATA4_PORT
#define D5_PORT *lcd_ports.LCD_DATA5_PORT
#define D6_PORT *lcd_ports.LCD_DATA6_PORT
#define D7_PORT *lcd_ports.LCD_DATA7_PORT

#define ___PORT_DEFAULT___

#endif


#endif //   __LCD_H__