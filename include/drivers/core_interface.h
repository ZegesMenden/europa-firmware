#include "pico/multicore.h"
#include "core.h"

#pragma once
namespace core1_interface {

    enum core0_code {
        CORE0_NEW_ASCENT_SIM_INPUT = 0xff000001,
        CORE0_NEW_LANDING_SIM_INPUT = 0xff000002,
        CORE0_NEW_DIVERT_SIM_INPUT = 0xff000003
    } core0_code;

    enum core1_code {
        CORE1_INIT_SUCESS = 0xff000000,
        CORE1_NEW_ASCENT_SIM_RESULT = 0xff000001,
        CORE1_NEW_LANDING_SIM_RESULT = 0xff000002,
        CORE1_NEW_DIVERT_SIM_RESULT = 0xff000003
    } core1_code;

    typedef void(*callback_t)();

    callback_t core1_ascent_sim_func = (callback_t)NULL;
    callback_t core1_landing_sim_func = (callback_t)NULL;
    callback_t core1_divert_sim_func = (callback_t)NULL;
    
    void core1_entry() {

        bool in_operation = false;

        multicore_fifo_push_blocking(CORE1_INIT_SUCESS);

        while(1) {
            if ( multicore_fifo_rvalid() ) {

                uint32_t fifo_rx = multicore_fifo_pop_blocking();

                if ( !in_operation ) {
                    if ( fifo_rx == CORE0_NEW_ASCENT_SIM_INPUT ) { 
                        if ( core1_ascent_sim_func != NULL ) { 
                            core1_ascent_sim_func();
                            multicore_fifo_push_blocking(CORE1_NEW_ASCENT_SIM_RESULT);
                        }
                    }
                    if ( fifo_rx == CORE0_NEW_LANDING_SIM_INPUT ) { 
                        if ( core1_landing_sim_func != NULL ) { 
                            core1_landing_sim_func();
                            multicore_fifo_push_blocking(CORE1_NEW_LANDING_SIM_RESULT);
                        }
                    }
                    if ( fifo_rx == CORE0_NEW_DIVERT_SIM_INPUT ) {
                        if ( core1_divert_sim_func != NULL ) { 
                            core1_divert_sim_func();
                            multicore_fifo_push_blocking(CORE1_NEW_DIVERT_SIM_RESULT);
                        }
                    }
                }

            }
        }

    }

    void core0_fifo_irq() {

        uint32_t fifo_rx;

        if ( multicore_fifo_pop_timeout_us(10, &fifo_rx) ) {

            if ( fifo_rx == CORE1_NEW_ASCENT_SIM_RESULT ) { flags::control::new_ascent_sim_result = true; }
            if ( fifo_rx == CORE1_NEW_LANDING_SIM_RESULT ) { flags::control::new_landing_sim_result = true; }
            if ( fifo_rx == CORE1_NEW_DIVERT_SIM_RESULT ) { flags::control::new_divert_sim_result = true; }

        }

    }

    bool init() {
        
        multicore_launch_core1(core1_interface::core1_entry);
        uint32_t fifo_rx = multicore_fifo_pop_blocking();
        if ( fifo_rx != core1_interface::CORE1_INIT_SUCESS ) { printf("core1 failed to initialize!\n"); return 0;}

        irq_set_exclusive_handler(SIO_IRQ_PROC0 + get_core_num(), core0_fifo_irq);
        irq_set_enabled(SIO_IRQ_PROC0 + get_core_num(), true);

        return 1;
    }

}