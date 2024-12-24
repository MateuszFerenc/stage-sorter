#ifndef __LCD_H__
#define __LCD_H__

#include "misc_def.h"

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



void lcd_command( uint8_t command );
void lcd_write_nibble( uint8_t data );
void lcd_init( void );
// TODO change defined pointer to void pointer
void put_data_to_lcd_buffer(void * data, uint8_t length, uint8_t row, uint8_t col, uint8_t buffer, uint8_t from_flash);
void put_one_char(unsigned char character, uint8_t length, uint8_t row, uint8_t col, uint8_t buffer);
void disp_clear_buffer(uint8_t buffer);
uint8_t disp_swap_buffers(void);
uint8_t disp_active_buffer_get( void );
void process_lcd_FSM( void );
unsigned char *get_buffer_address( void );


#define RS_PORT PORTD
#define EN_PORT PORTD
#define D4_PORT PORTC
#define D5_PORT PORTC
#define D6_PORT PORTC
#define D7_PORT PORTC
#define RS_PIN PD3
#define EN_PIN PD6
#define D4_PIN PC0
#define D5_PIN PC1
#define D6_PIN PC2
#define D7_PIN PC3

#ifndef RS_PORT
#error "LCD RS port is not defined!"
#endif

#ifndef EN_PORT
#error "LCD EN port is not defined!"
#endif

#ifndef D4_PORT
#error "LCD D4 port is not defined!"
#endif

#ifndef D5_PORT
#error "LCD D5 port is not defined!"
#endif

#ifndef D6_PORT
#error "LCD D6 port is not defined!"
#endif

#ifndef D7_PORT
#error "LCD D7 port is not defined!"
#endif

#ifndef RS_PIN
#error "LCD RS pin is not defined!"
#endif

#ifndef EN_PIN
#error "LCD EN pin is not defined!"
#endif

#ifndef D4_PIN
#error "LCD D4 pin is not defined!"
#endif

#ifndef D5_PIN
#error "LCD D5 pin is not defined!"
#endif

#ifndef D6_PIN
#error "LCD D6 pin is not defined!"
#endif

#ifndef D7_PIN
#error "LCD D7 pin is not defined!"
#endif

#endif //   __LCD_H__