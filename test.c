#include "test.h"

void wait_ms( uint16_t ms ){
    while(ms--)
        _delay_ms(1);
}

void wait_us( uint8_t us ){
    while(us--)
        _delay_us(1);
}