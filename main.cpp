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
// #include "drivers/softi2c.h"

task_t task_nav = { (callback_t)nav::update, 10000, 0, 0, 0 };
task_t task_perif = { (callback_t)perif::update, 10000, 0, 0, 0 };
task_t task_state = { (callback_t)update_sys_state, 10000, 0, 0, 0 };
task_t task_datalog = { (callback_t)datalog::update, 10000, 0, 0, 0 };
task_t task_control = { (callback_t)control::update, 10000, 0, 0, 0 };


int main(void)
{ 
  stdio_init_all();

  datalog::ptrs.points = {
    .state = &vehicle_state,
    .time = NULL,
    .flag_gpio = &flags::perif::gpio_sts,
    .flag_state = &flags::state::sts_bitmap,
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
    
    .gps_latitude = NULL,
    .gps_longitude = NULL,
    .gps_accuracy_h = NULL,
    .gps_accuracy_v = NULL,
    .gps_pdop = NULL,
    .gps_n_sats = NULL,
  };
  
  perif::init();

  print_compile_config();
  nav::init();
  datalog::init();
  neopix_write(5, 5, 5);

  vehicle_state = state_nav_init;
  update_task(task_nav);
  update_task(task_perif);
  update_task(task_datalog);
  update_task(task_state);
  uint32_t n_avaerages = 1;
  uint32_t average_runtime = task_nav.average_runtime+task_perif.average_runtime+task_datalog.average_runtime+task_control.average_runtime;
  // gpio_init(25);
  // gpio_set_dir(25, 1);

  while(1) {
    uint32_t t_start = time_us_32();
    // gpio_put(25, 1);
    update_task(task_nav);
    update_task(task_perif);
    update_task(task_datalog);
    update_task(task_state);
    update_task(task_control);
    // gpio_put(25, 0);
    average_runtime = ((average_runtime*95)/100) + (((time_us_32()-t_start)*5)/100);
    printf("%f, %f, %f, %f, %f, %f, %i\n", nav::position.x, nav::altitude, nav::velocity.x, nav::acceleration_i.x, nav::acceleration_b.x, nav::acceleration_i.x - nav::acceleration_b.x, average_runtime);
    sleep_us(10000-(time_us_32()-t_start));
  }
}
