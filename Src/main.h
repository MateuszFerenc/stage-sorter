// Include section start

#define LCD_CONTROL_TWO_PORT
#define ROM_READ pgm_read_byte

#include "misc_def.h"
#include "Lib/lcd.h"
#include "strings.h"

// Include section end


// Preprocessor definitions start

// sense stages ADC channels PORTA
#define metal_sense_adc 0
//#define glass_sense_adc 1     NOT USED
#define color_sense_adc 2

#define PWM0_pin        PB3
#define PWM0_port       PORTB
#define PWM1_pin        PD5
#define PWM1_port       PORTD
#define PWM2_pin        PD7
#define PWM2_port       PORTD

#define PWM0            0
#define PWM1            1
#define PWM2            2


#define RED_LED_pin         PB0
#define RED_LED_port        PORTB
#define GREEN_LED_pin       PB1
#define GREEN_LED_port      PORTB
#define BLUE_LED_pin        PB2
#define BLUE_LED_port       PORTB
#define WHITE_LED_pin       PA7
#define WHITE_LED_port      PORTA

#define BUZZER_pin          PD2
#define BUZZER_port         PORTD

#define MENU_STATE_DRAW_MAIN                        (uint8_t) 0
#define MENU_STATE_START                            (uint8_t) 1
#define MENU_STATE_SELECT                           (uint8_t) 3
#define MENU_STATE_CONFIG                           (uint8_t) 4
#define MENU_STATE_PWROFF                           (uint8_t) 5
#define MENU_STATE_S_ACTIVE                         (uint8_t) 6
#define MENU_STATE_SEL_ACTIVE                       (uint8_t) 7
#define MENU_STATE_C_ACTIVE                         (uint8_t) 8
#define MENU_STATE_C_A_PROGRAM                      (uint8_t) 10
#define MENU_STATE_C_A_SYSTEM                       (uint8_t) 11
#define MENU_STATE_C_A_EXIT                         (uint8_t) 12
#define MENU_STATE_C_A_PROGRAM_ACTIVE               (uint8_t) 13
#define MENU_STATE_C_A_SYSTEM_ACTIVE                (uint8_t) 14
#define MENU_STATE_C_A_PRG_A_view                   (uint8_t) 15
#define MENU_STATE_C_A_PRG_A_config                 (uint8_t) 16
#define MENU_STATE_C_A_PRG_A_save                   (uint8_t) 17
#define MENU_STATE_C_A_PRG_A_exit                   (uint8_t) 18
#define MENU_STATE_C_A_SYS_ACTIVE                   (uint8_t) 19
#define MENU_STATE_C_A_PRG_A_view_A                 (uint8_t) 20
#define MENU_STATE_C_A_PRG_A_config_A               (uint8_t) 21
#define MENU_STATE_DRAW_CONFIG                      (uint8_t) 22
#define MENU_STATE_DRAW_C_PROG                      (uint8_t) 23
#define MENU_STATE_DRIVE                            (uint8_t) 255

#define STAGE_STATE_WAIT                (uint8_t) 0
#define STAGE_STATE_IN                  (uint8_t) 1
#define STAGE_STATE_MEASURE             (uint8_t) 2
#define STAGE_STATE_OUT                 (uint8_t) 3
#define STAGE_STATE_DEFAULT             (uint8_t) 4

#define EEP_VAR_NOP                     (uint8_t) 0x00
#define EEP_VAR_ROM2RAM                 (uint8_t) 0xF0
#define EEP_VAR_RAM2ROM                 (uint8_t) 0x0F



#define KEYPAD_KEY_A                    (uint8_t) 32
#define KEYPAD_KEY_B                    (uint8_t) 64
#define KEYPAD_KEY_C                    (uint8_t) 96
#define KEYPAD_KEY_D                    (uint8_t) 128
#define KEYPAD_KEY_0                    (uint8_t) 23
#define KEYPAD_KEY_1                    (uint8_t) 1
#define KEYPAD_KEY_2                    (uint8_t) 2
#define KEYPAD_KEY_3                    (uint8_t) 4
#define KEYPAD_KEY_4                    (uint8_t) 8
#define KEYPAD_KEY_5                    (uint8_t) 9
#define KEYPAD_KEY_6                    (uint8_t) 11
#define KEYPAD_KEY_7                    (uint8_t) 15
#define KEYPAD_KEY_8                    (uint8_t) 16
#define KEYPAD_KEY_9                    (uint8_t) 18
#define KEYPAD_KEY_star                 (uint8_t) 22
#define KEYPAD_KEY_hash                    (uint8_t) 25

#define KEYPAD_ALT_UP                   KEYPAD_KEY_A + KEYPAD_KEY_2
#define KEYPAD_ALT_DOWN                 KEYPAD_KEY_A + KEYPAD_KEY_8
#define KEYPAD_ALT_LEFT                 KEYPAD_KEY_A + KEYPAD_KEY_4
#define KEYPAD_ALT_RIGHT                KEYPAD_KEY_A + KEYPAD_KEY_6
#define KEYPAD_ALT_SELECT               KEYPAD_KEY_A + KEYPAD_KEY_5



#define MASK_LOAD_PARAMETERS_SOURCE                                 (uint8_t) 3
#define MASK_LOAD_PARAMETERS_MODE                                   (uint8_t) 12

#define __LOAD_PARAMETERS_DIRECT                             (uint8_t) 0
#define __LOAD_PARAMETERS_VIA_RAM                            (uint8_t) 4
#define __LOAD_PARAMETERS_VIA_ROM                            (uint8_t) 8

#define LOAD_PARAMETERS_SRC_RAM_DIRECT             __LOAD_PARAMETERS_DIRECT + (uint8_t) 0
#define LOAD_PARAMETERS_SRC_ROM_DIRECT             __LOAD_PARAMETERS_DIRECT + (uint8_t) 1
#define LOAD_PARAMETERS_SRC_EEP_DIRECT             __LOAD_PARAMETERS_DIRECT + (uint8_t) 2

#define LOAD_PARAMETERS_SRC_RAM_VIA_RAM_TABLE           __LOAD_PARAMETERS_VIA_RAM + LOAD_PARAMETERS_SRC_RAM_DIRECT
#define LOAD_PARAMETERS_SRC_ROM_VIA_RAM_TABLE           __LOAD_PARAMETERS_VIA_RAM + LOAD_PARAMETERS_SRC_ROM_DIRECT
#define LOAD_PARAMETERS_SRC_EEP_VIA_RAM_TABLE           __LOAD_PARAMETERS_VIA_RAM + LOAD_PARAMETERS_SRC_EEP_DIRECT

#define LOAD_PARAMETERS_SRC_RAM_VIA_ROM_TABLE           __LOAD_PARAMETERS_VIA_ROM + LOAD_PARAMETERS_SRC_RAM_DIRECT
#define LOAD_PARAMETERS_SRC_ROM_VIA_ROM_TABLE           __LOAD_PARAMETERS_VIA_ROM + LOAD_PARAMETERS_SRC_ROM_DIRECT
#define LOAD_PARAMETERS_SRC_EEP_VIA_ROM_TABLE           __LOAD_PARAMETERS_VIA_ROM + LOAD_PARAMETERS_SRC_EEP_DIRECT

#define BYTE_LOAD_DISPLAY_PARAMETERS                        (uint8_t) 0
#define WORD_LOAD_DISPLAY_PARAMETERS                        (uint8_t) 1

// Preprocessor definitions end

/* J1 - Metal stage
1 - metal sense         (PA0)
2 - object present 0    (PA4)
3 - stage 0 motor       (PB3)
*/

/* J2 - Glass stage
1 - glass sense         (N.C.)
2 - object present 1    (PA5)
3 - stage 1 motor       (PD5)
*/

/* J3 - Color stage
1 - color sense         (PA2)
2 - object present 2    (PA6)
3 - stage 2 motor       (PD7)
4 - Red LED             (PB0)
5 - Green LED           (PB1)
6 - Blue RED            (PB2) 
7 - White LED           (PA7)
*/

/* J4 - LCD (HD44780 compatible)
1 - Gnd
2 - Vcc
3 - Contrast
4 - RS                  (PD3)
5 - Gnd
6 - E                   (PD6)
7 - Gnd
8 - Gnd
9 - Gnd
10 - Gnd
11 - D4                 (PC0)
12 - D5                 (PC1)
13 - D6                 (PC2)
14 - D7                 (PC3)
15 - LED +
16 - LED -
*/

/* J6 - UART
1 - Gnd
2 - RXD                 (PD0)
3 - TXD                 (PD1)
*/

/* J7 - Keypad
1 - R0                  (PC4)
2 - R1                  (PC5)
3 - R2                  (PC6)
4 - R3                  (PC7)
5 - C0                  (PB4)
6 - C1                  (PB5)
7 - C2                  (PB6)
8 - C3                  (PB7)
*/

/* J9 - Feed stage
1 - feed                (PD4)
2 - feed sense          (PA3)
*/

/*
Buzzer                  (PD2)
*/

/*
LCD Backlight           (PA1)
*/

// Functions declarations start

void USART_Transmit( char data );
unsigned char USART_Receive( void );
void USART_Flush( void );
void USART_text( char* text );

void wait_ms( uint16_t ms );
void wait_us( uint8_t us );
uint8_t blink_init(uint8_t row, uint8_t col, uint8_t length, uint8_t period);
void blink_stop( void );
uint8_t read_keypad( void );
unsigned char get_keypad_character( void );
static void fake_shutdown( void );
void print_settings_options( unsigned char buffer );
void check_eeprom_variables( void );
void display_parameters( uint8_t max_amount, uint8_t offset, uint8_t max_offset, void * names_ptr, void * values_ptr, uint8_t names_len, uint8_t values_len, uint8_t values_pos, uint8_t values_src, uint8_t values_word_select);
void display_prompt(uint8_t type, void * text, uint8_t text_length, uint8_t bufferr);

// Functions declarations end

//  Variables start
uint16_t metal_sense_buffer [10] = { 0 }, color_sense_buffer [10] = { 0 }, metal_sense_value = 0, color_sense_value = 0;
//uint16_t glass_sense_buffer[10] = { 0 }, glass_sense_value = 0;
uint16_t system_counter = 0, sec_counter = 0;
uint8_t sec = 0, min = 0, hour = 0;
uint8_t adc_hold = 0, adc_read_count = 0;


uint8_t compare_PWM0, compare_PWM1, compare_PWM2;
volatile uint8_t compbuff_PWM0, compbuff_PWM1, compbuff_PWM2;
uint8_t key_code = 0;           // 0 - no key pressed, function keys - A = 32, B = 64, C = 96, D = 128, other characters 1:31, 0 = 23, 1 = 1, 2 = 2, 3 = 4, 4 = 8, 5 = 9, 6 = 11, 7 = 15, 8 = 16, 9 = 18, * = 22, # = 25 



uint8_t menu_state = MENU_STATE_DRAW_MAIN, last_menu_state = MENU_STATE_DRAW_MAIN;

uint16_t prompt_time = 0;

uint8_t blink_position = 0;             // bits 7 - blink state, 6:5 - row, 4:0 - column
uint8_t blink_conf = 0;                 // bits 7:4 - period [0 - off, 1-15], 3:0 - length [1-16 characters], period = 0.625ms * ( 2 ^ ( 7 + blink_conf[7:4] ) )

unsigned char blink_buffer [16];

uint8_t sorting_state = 0;      // bits 7:4 - error code, 3:0 - sorting stage info

uint8_t EEPROM_FSM_state = EEP_VAR_NOP;    // 0xF0 : ROM -> RAM, 0x0F : RAM -> ROM
uint8_t EEPROM_variable_count = 0;


// Sorter process dependent variables, values loaded from "S_"
uint8_t stage1_servo_accept, stage1_servo_default, stage1_servo_reject;
//  uint8_t stage2_servo_accept, stage2_servo_default, stage2_servo_reject;
uint8_t stage3_servo_accept, stage3_servo_default, stage3_servo_reject;

uint8_t stage1_in_wait, stage1_measure_hold, stage1_out_wait;
// uint8_t stage2_in_wait, stage2_measure_hold, stage2_out_wait;
uint8_t stage3_in_wait, stage3_measure_hold, stage3_out_wait, stage3_color_switch_hold;

//  Sorter default editable set points loadable from EEPROM, "S_" prefix means set point
uint8_t S_stage1_servo_accept, S_stage1_servo_default, S_stage1_servo_reject;
//  uint8_t stage2_servo_accept, stage2_servo_default, stage2_servo_reject;
uint8_t S_stage3_servo_accept, S_stage3_servo_default, S_stage3_servo_reject;

uint8_t S_stage1_in_wait, S_stage1_measure_hold, S_stage1_out_wait;
// uint8_t stage2_in_wait, stage2_measure_hold, stage2_out_wait;
uint8_t S_stage3_in_wait, S_stage3_measure_hold, S_stage3_out_wait;
uint8_t S_stage3_color_switch_hold; // 3:2 [ 0 - no color, 01 - red, 10 - green, 11 - blue ]

uint8_t parameter_disp_config = 0;   // bit 7 - display [ 0 - ready to display, 1 - already displayed, waiting for refresh], bits 6:0 - starting offset of parameter [0 - 127]
uint8_t value_selected = 0;     // bit 7 - select [ 0 - ready to select, 1 - already selected, waiting for refresh], bits 6:0 - starting offset of parameter [0 - 127]

uint8_t active_program = 0;     // 0 - no program is active, any value other than zero means ID of active program

//uint8_t program_id = 0, program_name[7] = "TEST", program_stage1_conf = 0, program_stage3_conf = 5, program_stage1_val[2] = {0x02, 0x58}, program_stage3_red[2] = {0x03, 20}, program_stage3_grn[2] = { 0, 0}, program_stage3_blu[2] = { 0, 0};

//  Variables end


// EEPROM data region start, "E_" prefix means eeprom data region

// Stage Servos Limits
const EEMEM uint8_t  E_stage1_servo_accept = 65, E_stage1_servo_default = 53, E_stage1_servo_reject = 43;
//uint8_t EEMEM E_stage2_servo_accept = 0, E_stage2_servo_default = 0, E_stage2_servo_reject = 0;
const EEMEM uint8_t E_stage3_servo_accept = 68, E_stage3_servo_default = 56, E_stage3_servo_reject = 46;

const EEMEM uint8_t E_stage1_in_wait = 50, E_stage1_measure_hold = 20, E_stage1_out_wait = 30;
//uint8_t EEMEM E_stage2_in_wait = 2, E_stage2_measure_hold = 100, E_stage2_out_wait = 2;
const EEMEM uint8_t E_stage3_in_wait = 30, E_stage3_measure_hold = 90, E_stage3_out_wait = 30, E_stage3_color_switch_hold = 30;

//const EEMEM uint8_t program_content_array[ EEP_PRG_AMOUNT * EEP_PRG_SIZE ] = { 0, 'T', 'E', 'S', 'T', ' ', ' ', '', 0, 1, 0, 0, 0x3, 0x20, 0, 0, 0, 0};
// test program [id: 0, name: TEST, stage1_conf: 0 (accept all), stage2_conf: 1 (accept greater), stage1_val: 0, stage3_val_red: 800dec (320hex), stage3_val_grn: 0, stage3_val_blu: 0]

// EEPROM data region end


//  Constants start

// const uint8_t * const program_parameters_array [ 8 ] PROGMEM = {
//     &program_id, &program_name[0], &program_stage1_conf, &program_stage3_conf, &program_stage1_val[0], &program_stage3_red[0], &program_stage3_grn[0], &program_stage3_blu[0]
// };

const uint8_t * const eeprom_variables_pointer_array [ EEPROM_VARIABLES_COUNT ] PROGMEM = { 
    &E_stage1_servo_accept, &E_stage1_servo_default, &E_stage1_servo_reject, 
    &E_stage3_servo_accept, &E_stage3_servo_default, &E_stage3_servo_reject,
    &E_stage1_in_wait, &E_stage1_measure_hold, &E_stage1_out_wait,
    &E_stage3_in_wait, &E_stage3_measure_hold, &E_stage3_out_wait,
    &E_stage3_color_switch_hold
    //  &E_stage2_servo_accept, &E_stage2_servo_default, &E_stage2_servo_reject,
    //  &E_stage2_in_wait, &E_stage2_measure_hold, &E_stage2_out_wait, 
};

const uint8_t * const setpoint_variables_pointer_array [ EEPROM_VARIABLES_COUNT ] PROGMEM = {
    &S_stage1_servo_accept, &S_stage1_servo_default, &S_stage1_servo_reject,
    //   &stage2_servo_accept, &stage2_servo_default, &stage2_servo_reject,
    &S_stage3_servo_accept, &S_stage3_servo_default, &S_stage3_servo_reject,
    &S_stage1_in_wait, &S_stage1_measure_hold, &S_stage1_out_wait,
    //  &stage2_in_wait, &stage2_measure_hold, &stage2_out_wait,
    &S_stage3_in_wait, &S_stage3_measure_hold, &S_stage3_out_wait, &S_stage3_color_switch_hold
};

//const uint8_t program_memory_sizes [ 8 ] PROGMEM = { 1, 6, 1, 1, 2, 2, 2, 2 };



//  Constans end