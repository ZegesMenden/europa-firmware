#include <stdio.h>
#include "pico/stdlib.h"
#include "core.h"
#include "pico/sem.h"
#include "drivers/perif_ctrl.h"
#include "navigation.h"
#include "datalogging.h"
#include "navigation.h"
#include "hardware/gpio.h"
#include "task.h"

task_t task_nav_update = { (callback_t)nav::update_nav, 10000, 0 };
task_t task_gpio_update = { (callback_t)NULL, 10000, 0 };
task_t task_datalog = { (callback_t)NULL, 10000, 0 };

scheduler<3> scheduler_core0;

int main(void)
{  
  stdio_init_all();
  perif::init();

  while(!stdio_usb_connected()) {};
  sleep_ms(1000);
  print_compile_config();
  nav::init_nav();
  neopix_write(5, 5, 5);
  vehicle_state = state_nav_init;

  while(1) {
    nav::update_nav();
    sleep_ms(10);

    printf("%f, %f, %f, %f, %f\n", nav::acceleration_l.x, nav::acceleration_l.y, nav::acceleration_l.z, (float)nav::raw::read_time, (float)nav::raw::process_time);
  }
}
