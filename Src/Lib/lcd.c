#include "lcd.h"

uint8_t currentCol, currentRow, lcdRowStart [4];

uint8_t disp_buffers_dirty = 0;        // buffer "dirty" bits, one means buffer updated and ready to display
// bits: 7-4: disp_linear_buff[79:159] = 7 - 4th line .. 4 - 1st line, 3-0: disp_linear_buff[0:79] = 3 - 4th line .. 0 - 1st line

uint8_t disp_operation = DISP_STATE_NOP;

uint8_t disp_active_buffer = DISP_FRONTBUFFER;
void *disp_buffer_pointer = NULL;


void wait_us( uint16_t us );


static void process_lcd_FSM( void ){
    static uint8_t disp_state = DISP_STATE_NOP, disp_last_state = DISP_STATE_NOP;
    static uint8_t disp_delay = 0, disp_temp_data = 0, disp_column_counter = 0;

    if ( disp_state == DISP_STATE_NOP ){
        if ( ( disp_buffers_dirty || disp_column_counter ) && disp_operation == DISP_STATE_NOP ){
            if ( disp_column_counter == 0 ){
                unsigned buff_switch_offset = ( disp_active_buffer == DISP_BACKBUFFER )? 4 : 0;
                for (unsigned char buff_idx = buff_switch_offset; buff_idx < ( 4 + buff_switch_offset ); buff_idx++ ){
                    if ( ( disp_buffers_dirty >> buff_idx ) & 1 ){
                        disp_buffer_pointer = (unsigned char *)disp_linear_buff + (unsigned char)(buff_idx * 20);
                        disp_column_counter = 20;
                        disp_buffers_dirty &= ~( 1 << buff_idx );
                        disp_state = DISP_STATE_MCURSOR;
                        currentCol = 0;
                        currentRow = buff_idx % 4;
                        disp_temp_data = 0x80 | lcdRowStart[currentRow];
                        break;
                    }
                }
            } else {
                PIN_set(RS_PORT, lcd_pins.RS);
                
                disp_state = DISP_STATE_PUTHDATA;
                disp_temp_data = *((uint8_t *)disp_buffer_pointer++);
                disp_column_counter--;
            }
        } else {
            disp_state = disp_operation;
        }
    }

    if ( disp_state == DISP_STATE_MCURSOR ){
        disp_state = DISP_STATE_PUTHDATA;
        PIN_set(RS_PORT, lcd_pins.RS);
        disp_temp_data = 0x80 | ( lcdRowStart[currentRow] + currentCol);      // lcd home command
    }

    if ( disp_state == DISP_STATE_HOME ){
        disp_state = DISP_STATE_PUTHDATA;
        currentCol = 0;
        currentRow = 0;
        PIN_set(RS_PORT, lcd_pins.RS);
        disp_temp_data = 0x02;      // lcd home command
    }

    if ( disp_state == DISP_STATE_CLEAR ){
        disp_state = DISP_STATE_PUTHDATA;
        PIN_set(RS_PORT, lcd_pins.RS);
        disp_temp_data = 0x01;      // lcd clear command
    }

    if ( disp_state == DISP_STATE_PUTHDATA || disp_state == DISP_STATE_PUTLDATA ){
        disp_last_state = disp_state;

        PIN_clear(D4_PORT, lcd_pins.D4);
        PIN_clear(D5_PORT, lcd_pins.D5);
        PIN_clear(D6_PORT, lcd_pins.D6);
        PIN_clear(D7_PORT, lcd_pins.D7);

        if ( disp_state == DISP_STATE_PUTHDATA )
            disp_temp_data >>= 4;

        if ( disp_temp_data & 1 )
            PIN_set(D4_PORT, lcd_pins.D4);
        if ( disp_temp_data & 2 )
            PIN_set(D5_PORT, lcd_pins.D5);
        if ( disp_temp_data & 4 )
            PIN_set(D6_PORT, lcd_pins.D6);
        if ( disp_temp_data & 8 )
            PIN_set(D7_PORT, lcd_pins.D7);

        //PORTC &= 0xF0;
        //PORTC |= ( disp_state == DISP_STATE_PUTHDATA ) ? ( disp_temp_data >> 4 ) : ( disp_temp_data & 0x0F );

        disp_state = DISP_STATE_WAIT;
        disp_delay = 2;
    }

    if ( disp_state == DISP_STATE_WAIT ){
        if ( --disp_delay == 0 ){
            disp_state = DISP_STATE_ENTOGGLE;
        }
    }

    if ( disp_state == DISP_STATE_ENWAIT ){
        if ( --disp_delay == 0 ){
            if ( disp_last_state == DISP_STATE_PUTHDATA )
                disp_state = DISP_STATE_PUTLDATA;
            else {
                disp_state = DISP_STATE_NOP;
                disp_operation = DISP_STATE_NOP;
            }
        }
    }

    if ( disp_state == DISP_STATE_ENTOGGLE ){
        disp_state = DISP_STATE_ENWAIT;

        PIN_clear(EN_PORT, lcd_pins.EN);
        PIN_set(EN_PORT, lcd_pins.EN);
        PIN_clear(EN_PORT, lcd_pins.EN);

        disp_delay = 2;
    }
}


void put_data_to_lcd_buffer( void * data, uint8_t length, uint8_t row, uint8_t col, uint8_t buffer, uint8_t from_flash){
    unsigned char position = (unsigned char)(buffer + (row * 20) + col), temp;
    for (unsigned char offset = 0; length > 0 ; length--, offset++ ){
        if ( from_flash )
            temp = ROM_READ(data + offset);
        else
            temp = *((unsigned char *)data + offset);
        if ( temp < 32)
            break;
        *( disp_linear_buff + position ) = temp;
        position++;
    }
    disp_buffers_dirty |= 1 << ( ( buffer / 20 ) + row );
}

uint8_t disp_swap_buffers( void ){
    if ( disp_active_buffer == DISP_FRONTBUFFER ){
        disp_active_buffer = DISP_BACKBUFFER;
        disp_buffers_dirty = 0xF0;
    } else {
        disp_active_buffer = DISP_FRONTBUFFER;
        disp_buffers_dirty = 0x0F;
    }
    return disp_active_buffer;
}

static uint8_t disp_active_buffer_get( void ){
    return disp_active_buffer;
}

void disp_clear_buffer(uint8_t buffer){
    for (unsigned char offset = (unsigned char)buffer; offset < (80 + buffer); offset++)
        *( disp_linear_buff + offset ) = (unsigned char)' ';
    if ( buffer == DISP_FRONTBUFFER )
        disp_buffers_dirty = 0x0F;
    else
        disp_buffers_dirty = 0xF0;
}

void put_one_char(unsigned char character, uint8_t length, uint8_t row, uint8_t col, uint8_t buffer){
    unsigned char temp[20];
    if ( col + length > 20 )
        length -= col;
    for ( uint8_t chr = 0; chr < length; chr++)
        temp[chr] = character;
    put_data_to_lcd_buffer(&temp, length, row, col, buffer, 0);
}


void lcd_command(uint8_t command){
    PIN_clear(RS_PORT, lcd_pins.RS);
    lcd_write_nibble(command >> 4);
    lcd_write_nibble(command);
}

void lcd_write_nibble(uint8_t data){
    PIN_clear(D4_PORT, lcd_pins.D4);
    PIN_clear(D5_PORT, lcd_pins.D5);
    PIN_clear(D6_PORT, lcd_pins.D6);
    PIN_clear(D7_PORT, lcd_pins.D7);

    if ( data & 1 )
        PIN_set(D4_PORT, lcd_pins.D4);
    if ( data & 2 )
        PIN_set(D5_PORT, lcd_pins.D5);
    if ( data & 4 )
        PIN_set(D6_PORT, lcd_pins.D6);
    if ( data & 8 )
        PIN_set(D7_PORT, lcd_pins.D7);

    wait_us(200);

    PIN_clear(EN_PORT, lcd_pins.EN);
    PIN_set(EN_PORT, lcd_pins.EN);
    PIN_clear(EN_PORT, lcd_pins.EN);

    wait_us(200);
}

static void lcd_init(void){
    for ( uint8_t enable_4b_mode = 0; enable_4b_mode < 3; enable_4b_mode++){
        lcd_write_nibble(0x03);
        wait_ms(5);
    }
    lcd_write_nibble(0x02);
    wait_ms(1);
    
    lcd_command(0x28);
    
    lcd_command(0x01);     // lcd clear
    wait_ms(2);

    lcd_command(0x0C);
    lcd_command(0x06);
    
    lcdRowStart[0] = 0x00;
    lcdRowStart[1] = 0x40;
    lcdRowStart[2] = 20;            // Number of columns
    lcdRowStart[3] = 0x50 + 4;        // plus number of rows
    
    lcd_command(0x02);      // lcd home
    wait_ms(2);

    PIN_set(RS_PORT, lcd_pins.RS);
    lcd_write_nibble('O' >> 4);
    lcd_write_nibble('O');
    lcd_write_nibble('K' >> 4);
    lcd_write_nibble('K');
    wait_ms(120);
}

void wait_us( uint16_t us ){
    while(us--)
        asm volatile ("nop"::);
}