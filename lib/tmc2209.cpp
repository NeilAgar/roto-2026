#include "tmc2209.hpp"
#include "hardware/gpio.h"
#include <cmath>

TMC2209::TMC2209(uart_inst_t* uart, uint tx_pin, uint baud_rate, uint8_t addr)
    : _uart(uart), _tx_pin(tx_pin), _baud_rate(baud_rate), _addr(addr) {}

void TMC2209::enable() {
    uart_init(_uart, _baud_rate);
    gpio_set_function(_tx_pin, GPIO_FUNC_UART);
}

void TMC2209::setup(uint16_t microsteps, uint16_t current_ma) {
    // 1. GCONF: Disable analog scaling (I_scale_analog = 0)
    // We write 0x00 to GCONF to use internal reference and UART current scaling
    write_register(0x00, 0x00000000); 

    // 2. CHOPCONF
    // TOFF (3..0) = 5 (Enable driver, required for operation)
    // MRES (27..24):
    // 256=0, 128=1, 64=2, 32=3, 16=4, 8=5, 4=6, 2=7, 1=8
    uint8_t mres = 0;
    switch(microsteps) {
        case 256: mres = 0; break;
        case 128: mres = 1; break;
        case 64: mres = 2; break;
        case 32: mres = 3; break;
        case 16: mres = 4; break;
        case 8: mres = 5; break;
        case 4: mres = 6; break;
        case 2: mres = 7; break;
        case 1: mres = 8; break;
        default: mres = 4; break; // Default to 16
    }
    
    uint32_t chopconf = (uint32_t)mres << 24;
    chopconf |= 5; // TOFF = 5
    // Enable interpolation to 256 microsteps for smoother operation (intpol = 1, bit 28)
    chopconf |= (1 << 28);
    write_register(0x6C, chopconf);

    // 3. IHOLD_IRUN
    // CS calculation
    // CS = (32 * 1.414 * I_rms * (R_sense + 0.02)) / 0.325 - 1
    // Assuming R_sense = 110 mOhm = 0.11 Ohm
    
    float i_rms = (float)current_ma / 1000.0f;
    float cs_float = (32.0f * 1.414f * i_rms * (0.11f + 0.02f)) / 0.325f - 1.0f;
    
    if (cs_float < 0) cs_float = 0;
    if (cs_float > 31) cs_float = 31;
    uint8_t cs = (uint8_t)cs_float;
    
    uint32_t ihold_irun = 0;
    ihold_irun |= cs; // IRUN
    ihold_irun |= (8 << 8); // IHOLD (lower current for hold, e.g. ~half of run)
    ihold_irun |= (10 << 16); // IHOLDDELAY
    write_register(0x10, ihold_irun);
}

void TMC2209::write_register(uint8_t reg, uint32_t val) {
    uint8_t frame[8];
    frame[0] = 0x05; // Sync
    frame[1] = _addr; // Slave
    frame[2] = reg | 0x80; // Register + Write
    frame[3] = (val >> 24) & 0xFF;
    frame[4] = (val >> 16) & 0xFF;
    frame[5] = (val >> 8) & 0xFF;
    frame[6] = val & 0xFF;
    frame[7] = calc_crc(frame, 7);
    
    uart_write_blocking(_uart, frame, 8);
}

uint8_t TMC2209::calc_crc(uint8_t* data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t currentByte = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (currentByte & 0x01)) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc = (crc << 1);
            }
            currentByte = currentByte >> 1;
        }
    }
    return crc;
}
