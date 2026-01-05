#pragma once

#include <array>
#include "hardware/pio.h"
#include <concepts>
#include <utility>
#include <assert.h>

// find for what state machine a relative interrupt was thrown
inline uint sm_from_interrupt(const uint32_t irq_val, const uint32_t ir) {
    uint i;
    for (i = 0; i < 4; i++) { // should be sm 0 .. 3
        if (irq_val & 1 << i)
        { // is bit set?
            break;
        }
    }
    assert(i != 4); // develop check there has to be a bit
    return i;
}

// calculate relative IRQ flag for a state machine
// 10 (REL): the state machine ID (0…3) is added to the IRQ flag index, by way of
// modulo-4 addition on the two LSBs
inline uint relative_interrupt(const uint32_t ir, const uint sm) {
    uint32_t retval = ir & 0x03; // last 2 bits
    retval += sm;                // add relative value (is sm)
    retval = retval % 4;         // mod 4
    retval |= ir & 0xfffffffc;
    return retval;
}

// utility calculates the index for the object that serves a state machine in handlers_
inline size_t index_for(PIO pio, uint sm) { return PIO_NUM(pio) * 4 + sm; }

// utility to do math on pio_interrupt_source enum
inline pio_interrupt_source interrupt_source(const uint32_t ir){
    return static_cast<pio_interrupt_source>(std::to_underlying(pis_interrupt0) + ir);
}

// creating non-inline wrapper for some API calls, because GCC 15.1 for RISCV complains about exposing local TU

inline void _pio_set_irq0_source_enabled(PIO pio, pio_interrupt_source_t source, bool enabled) {
    pio_set_irq0_source_enabled(pio, source, enabled);
} 
inline void _pio_set_irq1_source_enabled(PIO pio, pio_interrupt_source_t source, bool enabled) {
    pio_set_irq1_source_enabled(pio, source, enabled);
}
inline void _pio_interrupt_clear(PIO pio, uint ir) {
    pio_interrupt_clear(pio, ir);
}

namespace pio_irq {

/*
PIO interrupts can only call functions without parameters. They can't call object members.
This static embedded class matches interrupts to the relevant object.
These handler objects have to implement the () operation.
*/
// guard that handler H has to support () operator.
template <std::invocable H, uint32_t interrupt_number> class pio_irq {
private:
    pio_irq() = delete; // static class. prevent instantiating.
public:
    static void register_interrupt(uint irq_channel, PIO pio, uint sm, bool enable) {
        assert (irq_channel < 2); // develop check that we use 0 or 1 only
        uint irq_num = PIO0_IRQ_0 + 2 * PIO_NUM(pio) + irq_channel;
        irq_handler_t handler = nullptr;

        if (irq_channel == 0) {
            _pio_set_irq0_source_enabled(pio, interrupt_source(relative_interrupt(interrupt_number, sm)), true);
        } else {
            _pio_set_irq1_source_enabled(pio, interrupt_source(relative_interrupt(interrupt_number, sm)), true);
        }

        switch (PIO_NUM(pio)) {
        case 0:
            handler = interrupt_handler_PIO0;
            break;
        case 1:
            handler = interrupt_handler_PIO1;
            break;
#if (NUM_PIOS > 2) // pico 2       
        case 2:
            handler = interrupt_handler_PIO2;
            break;
#endif            
        }

        irq_add_shared_handler(irq_num, handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY );  //Set the handler in the NVIC
        if (enable) {
            irq_set_enabled(irq_num, true);
        }
    }    

    // if an object is currently handling the pio + sm combination, it will
    // be replaced and will no longer receive interrupts
    // return false as warning if an existing combination is replaced
    static bool register_handler(PIO pio, uint sm, H *handler, bool set)      {
        size_t idx = index_for(pio, sm);
        H *old = handlers_[idx];
        handlers_[idx] = set ? handler : nullptr;
        return set ? old == nullptr : true;
    }

private:
    // forwards the interrupt to the correct handler object
    static void interrupt_handler(PIO pio) {
        if (pio->irq == 0U) {
            return; // we can't handle IRQs that don't have sm info
        }
        assert (pio->irq); // there should always be a sm
        uint sm = sm_from_interrupt(pio->irq, interrupt_number);
        uint ir = relative_interrupt(interrupt_number, sm);
        _pio_interrupt_clear(pio, ir); // I clear even if no handler
        H *handler =  handlers_[index_for(pio, sm)];
        if (handler != nullptr) {
            (*handler)();
        }
    }

    inline static void interrupt_handler_PIO0() { interrupt_handler(pio0); }
    inline static void interrupt_handler_PIO1() { interrupt_handler(pio1); }
#if (NUM_PIOS > 2) // pico 2
    inline static void interrupt_handler_PIO2() { interrupt_handler(pio2); }
#endif

    // keep pointer of objects that serve the state machines
    // 2-D array with slot for all possible state machines: PIO0[0..3], PIO1[0..3], ...
    inline static std::array<H *, NUM_PIOS * 4> handlers_;
};

// lib currently supports 1 (base) class without considerations.
// Behaviour when more than one handler is registered for the same sm/interrupt number combination is undefined. 
// If you use one handler class and one interrupt number, the library guarantees that there will be no conflict.
// else, take care that there's no overlap

} // namespace pio_irq
