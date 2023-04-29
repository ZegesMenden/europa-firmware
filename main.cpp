#include <stdio.h>
#include "pico/stdlib.h"
#include "core.h"
#include "pico/sem.h"
#include "drivers/perif_ctrl.h"
#include "navigation.h"
#include "datalogging.h"
#include "control.h"
#include "hardware/gpio.h"
#include "state_ctrl.h"
#include "task.h"
#include "drivers/core_interface.h"
#include "simulation.h"
#include "telemetry.h"
#include "drivers/radio.h"
#include "gps.h"

task_t task_nav = { (callback_t)nav::update, 10000, 0, 0, 0 };
task_t task_perif = { (callback_t)perif::update, 10000, 0, 0, 0 };
task_t task_state = { (callback_t)state::update, 10000, 0, 0, 0 };
task_t task_datalog = { (callback_t)datalog::update, 10000, 0, 0, 0 };
task_t task_control = { (callback_t)control::update, 10000, 0, 0, 0 };
task_t task_telemetry = { (callback_t)telemetry::update, 10000, 0, 0, 0 };
task_t task_radio = { (callback_t)radio::update, 10000, 0, 0, 0 };
task_t task_gps = { (callback_t)gps::update, 10000, 0, 0, 0 };

int main(void)
{ 
  stdio_init_all();

  datalog::ptrs.points = {

    .state = &vehicle_state,
    .time = &timing::RUNTIME,
    .flag_gpio = &flags::perif_flags::gpio_sts,
    .flag_state = &flags::state_flags::sts_bitmap,
    .flag_control = &flags::control_flags::control_bitmap,
    .flag_nav = &flags::nav_flags::nav_bitmap,
    .active_state_timer = &state::current_state_timer,
    .voltage_batt = &perif::voltage_batt_raw,
    .voltage_pyro = &perif::voltage_pyro_raw,
        
    .position = &nav::position,
    .velocity = &nav::velocity,
    .rotation = &nav::rotation,
    .accel_bias = &nav::acceleration_b,
    
    .acceleration = &nav::raw::accel,
    .ori_rate = &nav::raw::gyro,

    .baro_alt = &nav::altitude,
    .baro_pressure = &nav::pressure,
    .baro_temp = &nav::temperature,

    .mag = &nav::raw::mag,
    
    .burn_alt                = &control::gfield::burn_alt,
    .comp                    = &control::gfield::comp,
    .simulation_energy_est   = &control::simulation_energy_est,
    .simulation_work_est     = &control::simulation_work_est,
    .simulation_position_est = &control::simulation_position_est,
    .simulation_velocity_est = &control::simulation_velocity_est,
    .simulation_time_est     = &control::simulation_time_est

  };
  
  sleep_ms(2000);

  print_compile_config();
  perif::init();
  nav::init();
  datalog::init();
  telemetry::init();
  #ifdef USE_GPS
    gps::init();
  #endif
  radio::init();
  
  core1_interface::core1_landing_sim_func = (core1_interface::callback_t)simulation::run_landing_sim;
  core1_interface::core1_ascent_sim_func = (core1_interface::callback_t)simulation::run_ascent_sim;
  core1_interface::core1_divert_sim_func = (core1_interface::callback_t)simulation::run_divert_sim;
  core1_interface::init();

  vehicle_state = state_idle;

  while(1) {

    uint64_t t_start = time_us_64();

    timing::update();
    update_task(task_nav);
    update_task(task_state);
    update_task(task_control);
    update_task(task_perif);
    update_task(task_datalog);
    update_task(task_telemetry);
    update_task(task_radio);

    #ifdef USE_GPS
      update_task(task_gps);
    #endif

    // if ( time_us_64() > 10000 ) { set_vehicle_state(state_landed); }

    timing::average_runtime = (time_us_64()-t_start);

    // wait for next cycle
    int t_sleep = 10000-(timing::average_runtime);
    if ( t_sleep > 0 ) { sleep_us(t_sleep); }

  }
}
