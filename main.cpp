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

task_t task_nav_update = { (callback_t)nav::update, 10000, 0 };
task_t task_gpio_update = { (callback_t)perif::update, 10000, 0 };
task_t task_datalog = { (callback_t)datalog::update, 10000, 0 };

scheduler<3> scheduler_core0;

int main(void)
{  
  stdio_init_all();
  perif::init();

  while(!stdio_usb_connected()) {};
  sleep_ms(1000);
  print_compile_config();
  nav::init();
  datalog::init();
  neopix_write(5, 5, 5);
  vehicle_state = state_nav_init;

  scheduler_core0.add_task(task_nav_update);
  scheduler_core0.add_task(task_gpio_update);
  scheduler_core0.add_task(task_datalog);

  while(1) {
    scheduler_core0.update();
    sleep_ms(10);

    printf("%f, %f, %f, %f, %f\n", nav::acceleration_l.x, nav::acceleration_l.y, nav::acceleration_l.z, (float)nav::raw::read_time, (float)nav::raw::process_time);
  }
}
