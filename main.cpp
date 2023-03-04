#include <stdio.h>
#include "pico/stdlib.h"
#include "core.h"
#include "pico/sem.h"
#include "drivers/perif_ctrl.h"
#include "navigation.h"
#include "datalogging.h"
#include "navigation.h"
#include "hardware/gpio.h"
#include "state_ctrl.h"
#include "task.h"

task_t task_nav_update = { (callback_t)nav::update, 10000, 0, 0, 0 };
task_t task_gpio_update = { (callback_t)perif::update, 10000, 0, 0, 0 };
task_t task_datalog = { (callback_t)datalog::update, 10000, 0, 0, 0 };
task_t task_state = { (callback_t)update_sys_state, 10000, 0, 0, 0 };

int main(void)
{  
  stdio_init_all();

  // while(1) {
  //   printf("Hello, World!\n");
  //   sleep_ms(1000);
  // }

  perif::init();

  // while(!stdio_usb_connected()) {};
  // char c = getchar();

  sleep_ms(3000);
  print_compile_config();
  nav::init();
  datalog::init();
  neopix_write(5, 5, 5);

  sleep_ms(1000);
  vehicle_state = state_nav_init;

  while(1) {
    update_task(task_nav_update);
    update_task(task_gpio_update);
    update_task(task_datalog);
    update_task(task_state);
        
    sleep_ms(10);
    // printf("%i, %.10f, %.10f\n", nav::raw::pressure, nav::pressure, ( 1.f - pow(nav::pressure/101325.f, 1.f/5.255f) ) * 44330.f);
    printf("%f, %f, %f, %f, %f, %f, %i\n", nav::position.x, nav::velocity.x, nav::altitude, nav::acceleration_i.x, nav::acceleration_b.x, nav::acceleration_i.x - nav::acceleration_b.x, task_nav_update.average_runtime+task_gpio_update.average_runtime+task_datalog.average_runtime);
  }
}
