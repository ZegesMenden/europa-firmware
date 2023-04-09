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
// #include "drivers/softi2c.h"

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
    .flag_gpio = &flags::perif::gpio_sts,
    .flag_state = &flags::state::sts_bitmap,
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
  gps::init();
  radio::init();
  
  neopix_write(5, 5, 5);

  // uart_init(uart1, 115200);
  // while(1) {
  //   if ( uart_is_readable(uart1) ) {
  //     uint8_t c;
  //     uart_read_blocking(uart1, &c, 1);
  //     // printf("%c", c);
  //     uart_write_blocking(uart0, &c, 1);
  //   }
  //   // int c;
  //   // c = getchar_timeout_us(1);
  //   // uint8_t _c = c;
  //   // if ( c != PICO_ERROR_TIMEOUT ) { uart_write_blocking(uart1, &_c, 1); }
  // }

  core1_interface::core1_landing_sim_func = (core1_interface::callback_t)simulation::run_landing_sim;
  core1_interface::core1_ascent_sim_func = (core1_interface::callback_t)simulation::run_ascent_sim;
  core1_interface::core1_divert_sim_func = (core1_interface::callback_t)simulation::run_divert_sim;
  core1_interface::init();

  vehicle_state = state_nav_init;
  timing::update();
  update_task(task_nav);
  update_task(task_state);
  update_task(task_control);
  update_task(task_perif);
  update_task(task_datalog);
  update_task(task_telemetry);
  timing::average_runtime = task_nav.average_runtime+task_perif.average_runtime+task_datalog.average_runtime+task_control.average_runtime;

  bool data_has_been_logged = false;

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
    update_task(task_gps);

    // if ( ((datalog::page - datalog::start_page) > 500) && !data_has_been_logged ) {
    //   datalog::export_flash_data(datalog::start_page);
    //   data_has_been_logged = true;
    // }
    
    timing::average_runtime = time_us_64()-t_start;//((timing::average_runtime*80)/100) + (((time_us_64()-t_start)*20)/100);

    // printf("%i, %f, %f, %f, %f, %f, %f, %i\n", get_vehicle_state(), nav::position.x, nav::altitude, nav::velocity.x, nav::acceleration_i.x, nav::acceleration_b.x, nav::acceleration_i.x - nav::acceleration_b.x, average_runtime);
    
    // printf("%f,%f,%f,%f,%i,%i\n", control::angle_error.y*180.f/3.1415f, control::angle_error.z*180.f/3.1415f, -control::pid_ori_y.output, -control::pid_ori_z.output, perif::servo_1_position, perif::servo_2_position);

    int t_sleep = 10000-(time_us_64()-t_start);
    if ( t_sleep > 0 ) { sleep_us(t_sleep); }
  }
}
