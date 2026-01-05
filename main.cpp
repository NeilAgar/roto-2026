#include <stdio.h>
#include <array>
#include <cmath>
#include "pico/stdlib.h"
#include "hardware/pio.h"

#include "stepper.hpp"
#include "tmc2209.hpp"
#include "config.h"
#include "ws2812.pio.h"
#include "user_commands.h"

// config what PIO, SM and IRQ channel to use
const auto pio_motors = pio0;
const uint sm_left = 0;
const uint sm_right = 1;
const uint sm_led = 2;
const uint pio_irq_num = 0;

using motor_t = stepper::stepper_callback_controller;
motor_t motor_left(pio_motors, sm_left);
motor_t motor_right(pio_motors, sm_right);

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

void put_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio_motors, sm_led, pixel_grb << 8u);
}

uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)(r) << 16) | ((uint32_t)(g) << 8) | (uint32_t)(b);
}

void init_pio() {
    // do this only once per used pio
    motor_t::pio_program(pio_motors);

    // initialise / enable the motor state machine
    motor_left.register_pio_interrupt(pio_irq_num, true);
    motor_left.pio_init(PIN_DIR_LEFT, PIN_STEP_LEFT, 3.f);

    motor_right.register_pio_interrupt(pio_irq_num, true);
    motor_right.pio_init(PIN_DIR_RIGHT, PIN_STEP_RIGHT, 3.f);

    // initialize WS2812
    uint offset = pio_add_program(pio_motors, &ws2812_program);
    ws2812_program_init(pio_motors, sm_led, offset, PIN_LED, 800000, false);
}

void on_complete_left(const motor_t &stepper) {
    // printf("Left motor executed command %d\n", motor_left.commands());
}

void on_complete_right(const motor_t &stepper) {
    // printf("Right motor executed command %d\n", motor_right.commands());
}

// helper for calculating delay from speed (steps/s)
uint32_t get_delay_for_speed(float speed) {
    if (speed < 1.0f) speed = 1.0f;
    // Constant K = 20833333 
    return (uint32_t)(20833333.0f / speed);
}

// helper to update speed in-band
void set_speed_in_band(float speed) {
    uint32_t delay = get_delay_for_speed(speed);
    stepper::command cmd(0, false); // 0 steps triggers update_delay in modified PIO
    
    motor_left.take_steps(cmd);
    pio_sm_put_blocking(pio_motors, sm_left, delay);
    
    motor_right.take_steps(cmd);
    pio_sm_put_blocking(pio_motors, sm_right, delay);
}

// Simulation function to EXACTLY match the main loop execution
float simulate_move_time(uint32_t steps, float v_max, float v_min, float accel) {
    if (steps == 0) return 0.0f;

    // S-Curve Heuristic: We need more distance to accelerate than trapezoidal!!11!!
    // Remove the divisor 2 in trapezoidal profiling for more space for the S-ramp
    float steps_accel = (v_max*v_max - v_min*v_min) / (accel);
    
    if (steps_accel * 2.0f > (float)steps) {
        steps_accel = (float)steps / 2.0f;
    }

    uint32_t steps_issued = 0;
    uint64_t total_ticks = 0;

    while (steps_issued < steps) {
        float current_speed = v_min;
        
        // S-Curve Calculation
        if (steps_issued < steps_accel) {
            float ratio = (float)steps_issued / steps_accel;
            // 0.5 * (1 - cos(pi * ratio)) for a smooth S from 0.0 to 1.0
            float s_curve_factor = (1.0f - cosf(M_PI * ratio)) / 2.0f;
            current_speed = v_min + (v_max - v_min) * s_curve_factor;
        } 
        else if (steps_issued >= steps - steps_accel) {
            float dist_rem = (float)(steps - steps_issued);
            float ratio = dist_rem / steps_accel;
            float s_curve_factor = (1.0f - cosf(M_PI * ratio)) / 2.0f;
            current_speed = v_min + (v_max - v_min) * s_curve_factor;
        } 
        else {
            current_speed = v_max;
        }

        if (current_speed > v_max) current_speed = v_max;
        if (current_speed < v_min) current_speed = v_min;

        // Batch Size 
        uint32_t batch_size = 5;
        if (current_speed >= v_max * 0.95f) {
            batch_size = 20;
        }
        if (steps_issued + batch_size > steps) {
            batch_size = steps - steps_issued;
        }

        // Time
        uint32_t delay = get_delay_for_speed(current_speed);
        total_ticks += (uint64_t)batch_size * delay;

        steps_issued += batch_size;
    }

    return (float)total_ticks / 20833333.0f;
}

float calculate_sequence_time(float speed_scale, float steps_per_mm, float steps_per_deg) {
    float total_time_s = 0.0f;
    
    float v_min = MIN_SPEED_MM_PER_SEC * steps_per_mm;
    float v_max = MAX_SPEED_MM_PER_SEC * steps_per_mm * speed_scale;
    float accel = ACCEL_MM_PER_SEC2 * steps_per_mm; 
    
    if (v_max < v_min) v_max = v_min;

    for (const auto& cmd : command_sequence) {
        if (cmd.type == DELAY) {
            total_time_s += cmd.value / 1000.0f;
            total_time_s += 0.001f; 
        } else {
            uint32_t steps = 0;
            if (cmd.type == MOVE_FWD || cmd.type == MOVE_BWD) {
                steps = (uint32_t)(cmd.value * steps_per_mm);
            } else if (cmd.type == TURN_LEFT || cmd.type == TURN_RIGHT) {
                steps = (uint32_t)(cmd.value * steps_per_deg);
            }
            
            if (steps > 0) {
                total_time_s += simulate_move_time(steps, v_max, v_min, accel);
                total_time_s += 0.006f;
            }
        }
    }
    
    return total_time_s * 1.5f;
}

int main() {
    stdio_init_all();

    // initialize teh TMC2209s
    TMC2209 tmc_left(uart0, PIN_UART_LEFT_TX, 115200, 0);
    tmc_left.enable();
    tmc_left.setup(MICROSTEPS, CURRENT_MA);

    TMC2209 tmc_right(uart1, PIN_UART_RIGHT_TX, 115200, 0);
    tmc_right.enable();
    tmc_right.setup(MICROSTEPS, CURRENT_MA);

    sleep_ms(100); 

    init_pio();

    put_pixel(urgb_u32(0, 0, 0));

    motor_left.on_complete_callback(on_complete_left);
    motor_right.on_complete_callback(on_complete_right);

    set_speed_in_band(MIN_SPEED_MM_PER_SEC * (STEPS_PER_REV / MM_PER_REV));

    gpio_init(PIN_BUTTON);
    gpio_set_dir(PIN_BUTTON, GPIO_IN);
    gpio_pull_up(PIN_BUTTON);

    enum State { IDLE, MOVING };
    State state = IDLE;
    
    size_t current_cmd_idx = 0;
    bool waiting_for_delay = false;
    absolute_time_t delay_end_time;
    
    // motion profile
    bool motors_active = false;
    uint32_t total_steps_needed = 0;
    uint32_t steps_issued = 0;
    
    float steps_accel = 0;
    float cruise_speed_steps = 0;
    float min_speed_steps = 0;
    float accel_steps_per_sec2 = 0;
    
    bool left_fwd_latch = true;
    bool right_fwd_latch = true;

    // prec-calculated constants
    const float STEPS_PER_MM = STEPS_PER_REV / MM_PER_REV;
    const float STEPS_PER_DEG = STEPS_PER_90 / 90.0f;
    
    float speed_scale = 1.0f;

    while (true) {
        if (state == IDLE) {
            put_pixel(urgb_u32(0, 0, 0)); // Off

            if (!gpio_get(PIN_BUTTON)) {
                sleep_ms(20); 
                if (!gpio_get(PIN_BUTTON)) {
                    put_pixel(urgb_u32(0, 0, 255)); // Blue :)
                    while (!gpio_get(PIN_BUTTON)) {
                        sleep_ms(10);
                    }
                    
                    // Binary Search for Speed Scale (oh yeah this is the meta)
                    float predicted_min_time = calculate_sequence_time(1.0f, STEPS_PER_MM, STEPS_PER_DEG);
                    float target_time_s = TARGET_TIME_MS / 1000.0f;
                    
                    if (predicted_min_time > target_time_s) {
                        speed_scale = 1.0f;
                    } else {
                        float low = 0.01f; 
                        float high = 1.0f;
                        for (int i = 0; i < 20; i++) {
                            float mid = (low + high) / 2.0f;
                            float t = calculate_sequence_time(mid, STEPS_PER_MM, STEPS_PER_DEG);
                            if (t < target_time_s) {
                                high = mid;
                            } else {
                                low = mid;
                            }
                        }
                        speed_scale = (low + high) / 2.0f;
                    }
                    
                    state = MOVING;
                    current_cmd_idx = 0;
                    waiting_for_delay = false;
                    motors_active = false;
                }
            }
        } else if (state == MOVING) {
            put_pixel(urgb_u32(0, 255, 0)); // Green :D

            if (!gpio_get(PIN_BUTTON)) {
                sleep_ms(20); 
                if (!gpio_get(PIN_BUTTON)) {
                    put_pixel(urgb_u32(0, 0, 255)); 
                    pio_sm_set_enabled(pio_motors, sm_left, false);
                    pio_sm_set_enabled(pio_motors, sm_right, false);
                    
                    pio_sm_clear_fifos(pio_motors, sm_left);
                    pio_sm_clear_fifos(pio_motors, sm_right);
                    
                    state = IDLE;
                    while (!gpio_get(PIN_BUTTON)) {
                        sleep_ms(10);
                    }
                    continue;
                }
            }

            if (waiting_for_delay) {
                if (absolute_time_diff_us(get_absolute_time(), delay_end_time) < 0) {
                    waiting_for_delay = false;
                }
            } 
            else if (motors_active) {
                if (steps_issued >= total_steps_needed) {
                    if (pio_sm_is_tx_fifo_empty(pio_motors, sm_left) && 
                        pio_sm_is_tx_fifo_empty(pio_motors, sm_right)) {
                         sleep_ms(5); 
                         motors_active = false;
                    }
                } else {
                    if (!pio_sm_is_tx_fifo_full(pio_motors, sm_left) && 
                        !pio_sm_is_tx_fifo_full(pio_motors, sm_right)) {
                        
                        float current_speed = min_speed_steps;
                        
                        // Uses Sine-Squared ramp for smooth Jerk
                        
                        // Acceleration Phase
                        if (steps_issued < steps_accel) {
                            float ratio = (float)steps_issued / steps_accel;
                            float s_curve_factor = (1.0f - cosf(M_PI * ratio)) / 2.0f;
                            current_speed = min_speed_steps + (cruise_speed_steps - min_speed_steps) * s_curve_factor;
                        }
                        // Deceleration Phase
                        else if (steps_issued >= total_steps_needed - steps_accel) {
                            float dist_rem = (float)(total_steps_needed - steps_issued);
                            float ratio = dist_rem / steps_accel;
                            float s_curve_factor = (1.0f - cosf(M_PI * ratio)) / 2.0f;
                            current_speed = min_speed_steps + (cruise_speed_steps - min_speed_steps) * s_curve_factor;
                        }
                        // Cruise Phase
                        else {
                            current_speed = cruise_speed_steps;
                        }

                        if (current_speed > cruise_speed_steps) current_speed = cruise_speed_steps;
                        if (current_speed < min_speed_steps) current_speed = min_speed_steps;

                        set_speed_in_band(current_speed); 

                        uint32_t batch_size = 5; 
                        if (current_speed >= cruise_speed_steps * 0.95f) {
                            batch_size = 20;
                        }
                        if (steps_issued + batch_size > total_steps_needed) {
                            batch_size = total_steps_needed - steps_issued;
                        }
                        
                        stepper::command cmd_left(batch_size, left_fwd_latch);
                        stepper::command cmd_right(batch_size, right_fwd_latch);
                        
                        motor_left.take_steps(cmd_left);
                        motor_right.take_steps(cmd_right);
                        
                        steps_issued += batch_size;
                    }
                }
            }
            else {
                if (current_cmd_idx >= command_sequence.size()) {
                    for (int i = 0; i < 6; i++) {
                        put_pixel(urgb_u32(255, 0, 0)); 
                        sleep_ms(250);
                        put_pixel(urgb_u32(0, 0, 0));   
                        sleep_ms(250);
                    }
                    state = IDLE;
                } else {
                    const UserCommand& cmd = command_sequence[current_cmd_idx];
                    current_cmd_idx++;

                    if (cmd.type == DELAY) {
                        waiting_for_delay = true;
                        delay_end_time = make_timeout_time_ms((uint32_t)cmd.value);
                    } else {
                        total_steps_needed = 0;
                        bool left_fwd = true;
                        bool right_fwd = true;

                        if (cmd.type == MOVE_FWD || cmd.type == MOVE_BWD) {
                            total_steps_needed = (uint32_t)(cmd.value * STEPS_PER_MM);
                            if (cmd.type == MOVE_BWD) {
                                left_fwd = false;
                                right_fwd = false;
                            }
                        } else if (cmd.type == TURN_LEFT || cmd.type == TURN_RIGHT) {
                            total_steps_needed = (uint32_t)(cmd.value * STEPS_PER_DEG);
                            if (cmd.type == TURN_LEFT) {
                                left_fwd = false; 
                                right_fwd = true;
                            } else {
                                left_fwd = true;  
                                right_fwd = false;
                            }
                        }

                        if (REVERSE_LEFT) left_fwd = !left_fwd;
                        if (REVERSE_RIGHT) right_fwd = !right_fwd;
                        
                        left_fwd_latch = left_fwd;
                        right_fwd_latch = right_fwd;

                        min_speed_steps = MIN_SPEED_MM_PER_SEC * STEPS_PER_MM;
                        cruise_speed_steps = MAX_SPEED_MM_PER_SEC * STEPS_PER_MM * speed_scale;
                        if (cruise_speed_steps < min_speed_steps) cruise_speed_steps = min_speed_steps;
                        accel_steps_per_sec2 = ACCEL_MM_PER_SEC2 * STEPS_PER_MM;
                        
                        // S-Curve Accel Calculation
                        steps_accel = (cruise_speed_steps*cruise_speed_steps - min_speed_steps*min_speed_steps) / (accel_steps_per_sec2);
                        
                        if (steps_accel * 2.0f > (float)total_steps_needed) {
                            steps_accel = (float)total_steps_needed / 2.0f;
                        }

                        steps_issued = 0;
                        motor_left.reset_commands();
                        motor_right.reset_commands();
                        
                        pio_enable_sm_mask_in_sync(pio_motors, (1u << sm_left) | (1u << sm_right));
                        motors_active = true;
                    }
                }
            }
        }
        sleep_us(500);
    }
}