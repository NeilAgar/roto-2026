#pragma once
#include "pico/stdlib.h"
#include "hardware/uart.h"

class TMC2209 {
public:
    TMC2209(uart_inst_t* uart, uint tx_pin, uint baud_rate = 115200, uint8_t addr = 0);
    void enable();
    void setup(uint16_t microsteps, uint16_t current_ma);

private:
    uart_inst_t* _uart;
    uint _tx_pin;
    uint _baud_rate;
    uint8_t _addr;

    void write_register(uint8_t reg, uint32_t val);
    uint8_t calc_crc(uint8_t* data, uint8_t len);
};
