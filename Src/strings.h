#ifndef __STRINGS_H__
#define __STRINGS_H__


#include <avr/pgmspace.h>
#include "misc_def.h"



const unsigned char keypad_num0_keys [5] PROGMEM = "-12-3";
const unsigned char keypad_num1_keys [5] PROGMEM = "-45-6";
const unsigned char keypad_num2_keys [5] PROGMEM = "-78-9";
const unsigned char keypad_num3_keys [5] PROGMEM = "-*0-#";
const unsigned char keypad_func_keys [5] PROGMEM = "-ABCD";

const unsigned char key_map[26] PROGMEM = {
        '-', '1', '2', '-', '3', '-', '-', '-', 
        '4', '5', '-', '6', '-', '-', '-', 
        '7', '8', '-', '9', '-', '-', '-', 
        '*', '0', '-', '#'
    };


// Menu 0 strings
const unsigned char text_start [5] PROGMEM = "START";
const unsigned char text_stop [4] PROGMEM = "STOP";
const unsigned char text_select_program [14] PROGMEM = "select program";
const unsigned char text_configure [9] PROGMEM = "configure";
const unsigned char text_power_off [9] PROGMEM = "power off";

const unsigned char text_no_program [11] PROGMEM = "No program!";
const unsigned char text_program_loaded [14] PROGMEM = "Program loaded";

// Menu 1 strings
const unsigned char text_select_hint [20] PROGMEM = "1.LD 2.DEL 3.SV 4.NW";

// Menu 2 strings
const unsigned char text_configure0 [10] PROGMEM = "Configure:";
#define text_program *(text_select_program + 7)

const unsigned char text_system [6] PROGMEM = "system";
const unsigned char text_exit [4] PROGMEM = "exit";
const unsigned char text_status0 [7] PROGMEM = "Status:";
const unsigned char text_ok [2] PROGMEM = "OK";
const unsigned char text_err [3] PROGMEM = "Err";
const unsigned char text_info [4] PROGMEM = "Info";
const unsigned char text_warning [7] PROGMEM = "Warning";
const unsigned char text_stopped [7] PROGMEM = "stopped";
const unsigned char text_running [7] PROGMEM = "running";
const unsigned char text_runtime0 [8] PROGMEM = "Runtime:";

// Menu 3 strings
const unsigned char text_parameters0 [11] PROGMEM = "Parameters:";
const unsigned char text_view [4] PROGMEM = "view";
const unsigned char text_save [4] PROGMEM = "save";

// Menu 4 strings

// Other strings
const unsigned char text_goodbye [10] PROGMEM = "Goodbye :)";
const unsigned char sorter_version [11] PROGMEM = "Sorter v" TOSTRING(stable_version) "." TOSTRING(beta_version);
const unsigned char compilation_date [11] PROGMEM = __DATE__;
const unsigned char dev_name [14] PROGMEM = "Mateusz Ferenc";

// EEPROM_VARIABLES_COUNT * 13, 13 due to one parameter name will be limited to 13 characters
const unsigned char system_parameters_display_names [ EEPROM_VARIABLES_COUNT * 13 ] PROGMEM = 
    "stg1_serv_acc"\
    "stg1_serv_def"\
    "stg1_serv_rej"\
    "stg3_serv_acc"\
    "stg3_serv_def"\
    "stg3_serv_rej"\
    "stg1_in_wait "\
    "stg1_meas_hld"\
    "stg1_out_wait"\
    "stg3_in_wait "\
    "stg3_meas_hld"\
    "stg3_out_wait"\
    "stg3_color_sw";

//const uint8_t program_content_array [ 8 + ( 10 * EEP_PRG_SIZE ) ] PROGMEM = { 
const uint8_t program_content_array [  11 * EEP_PRG_SIZE  ] PROGMEM = { 
// ID, NAME,                            , stg1, stg3, stg1 val  , stg3 red  , stg3 green, stg3 blue
    0, 'D', 'I', 'S', 'A', 'B', 'L', 'E', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    1, 'T', 'E', 'S', 'T', ' ', ' ', ' ', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    2, 'R', 'E', 'D', ' ', ' ', ' ', ' ', 0x00, 0x05, 0x00, 0x00, 0x03, 0x20, 0x03, 0x52, 0x03, 0x20,
    3, 'G', 'R', 'E', 'E', 'N', ' ', ' ', 0x00, 0x59, 0x00, 0x00, 0x03, 0x3E, 0x03, 0x34, 0x03, 0x20,
    4, 'B', 'L', 'U', 'E', ' ', ' ', ' ', 0x00, 0x65, 0x00, 0x00, 0x03, 0x20, 0x03, 0x34, 0x03, 0x3E,
    5, 'M', 'E', 'T', '0', 'Y', 'E', 'S', 0x02, 0x03, 0x02, 0x84, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    6, 'M', 'E', 'T', '1', 'N', 'O', ' ', 0x01, 0x03, 0x02, 0x84, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    7, 'M', 'E', 'T', '2', 'N', 'O', ' ', 0x01, 0x00, 0x02, 0x84, 0x03, 0x20, 0x00, 0x00, 0x00, 0x00,
    8, 'E', 'M', 'P', 'T', 'Y', ' ', ' ', 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    9, 'E', 'M', 'P', 'T', 'Y', ' ', ' ', 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    10, 'E', 'M', 'P', 'T', 'Y', ' ', ' ', 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

const unsigned char program_parameters_display_names [ 12 * 8 ] PROGMEM = 
    "ID          "\
    "NAME        "\
    "Stage1 conf "\
    "Stage3 conf "\
    "Stage1 val  "\
    "Stage3 RED  "\
    "Stage3 GREEN"\
    "Stage3 BLUE ";

const unsigned char program_stage_config_display_names [ 10 * 7 ] PROGMEM = 
    "Accept all"\
    "Reject all"\
    "greater   "\
    "lesser    "\
    "AND select"\
    "OR  select"\
    "ignore    ";


#endif // __STRINGS_H__