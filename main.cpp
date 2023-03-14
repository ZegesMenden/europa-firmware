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
// #include "drivers/softi2c.h"

task_t task_nav = { (callback_t)nav::update, 10000, 0, 0, 0 };
task_t task_perif = { (callback_t)perif::update, 10000, 0, 0, 0 };
task_t task_state = { (callback_t)state::update, 10000, 0, 0, 0 };
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

  sleep_ms(2000);
  
  perif::init();

  print_compile_config();
  nav::init();
  datalog::init();
  
  neopix_write(5, 5, 5);

  core1_interface::core1_landing_sim_func = (core1_interface::callback_t)simulation::run_landing_sim;

  multicore_launch_core1(core1_interface::core1_entry);
  uint32_t fifo_rx = multicore_fifo_pop_blocking();
  if ( fifo_rx != core1_interface::CORE1_INIT_SUCESS ) { printf("core1 failed to initialize!\n"); while(1) {;} }

  printf("testing core1 landing sim...\n");

  simulation::landing_sim_input.position = 14.25;
  simulation::landing_sim_input.burn_alt = 9.24;
  simulation::landing_sim_input.velocity = -4.25;
  simulation::landing_sim_input.acceleration = 0.00276816608;
  simulation::landing_sim_input.mass = 0.31;  
  

  uint32_t t_execution = time_us_32();
  multicore_fifo_push_blocking(core1_interface::CORE0_NEW_LANDING_SIM_INPUT);

  fifo_rx = multicore_fifo_pop_blocking();
  t_execution = time_us_32() - t_execution;

  printf("done!\nexecution took %iuS\n", t_execution);
  printf("%f\n%f\n%f\n%f\n", simulation::landing_sim_output.position, simulation::landing_sim_output.velocity, simulation::landing_sim_output.time, simulation::landing_sim_output.work_done);

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
    // printf("%f, %f, %f, %f, %f, %f, %i\n", nav::position.x, nav::altitude, nav::velocity.x, nav::acceleration_i.x, nav::acceleration_b.x, nav::acceleration_i.x - nav::acceleration_b.x, average_runtime);
    printf("%f, %f, %f, %f\n", control::angle_error.y*180.f/PI, control::angle_error.z*180.f/PI, control::pid_ori_y.output, control::pid_ori_z.output);
    // printf("%i, %i\n", perif::voltage_switch_raw, get_vehicle_state());
    sleep_us(10000-(time_us_32()-t_start));
    //nav::rotation.euler_angles_x()*180.f/PI, nav::rotation.euler_angles_y()*180.f/PI, nav::rotation.euler_angles_z()*180.f/PI
    //control::angle_error.y*180.f/PI, control::angle_error.z*180.f/PI
  }
}
