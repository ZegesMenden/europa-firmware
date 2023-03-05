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

  print_compile_config();
  nav::init();
  datalog::init();
  neopix_write(5, 5, 5);

  vehicle_state = state_nav_init;
  update_task(task_nav_update);
  update_task(task_gpio_update);
  update_task(task_datalog);
  update_task(task_state);
  uint32_t n_avaerages = 1;
  uint32_t average_runtime = task_nav_update.average_runtime+task_gpio_update.average_runtime+task_datalog.average_runtime;
  gpio_init(25);
  gpio_set_dir(25, 1);
  while(1) {
    uint32_t t_start = time_us_32();
    gpio_put(25, 1);
    update_task(task_nav_update);
    update_task(task_gpio_update);
    update_task(task_datalog);
    update_task(task_state);
    gpio_put(25, 0);
    average_runtime = ((average_runtime*95)/100) + (((time_us_32()-t_start)*5)/100);
    printf("%f, %f, %f, %f, %f, %f, %i\n", nav::position.x, nav::altitude, nav::velocity.x, nav::acceleration_i.x, nav::acceleration_b.x, nav::acceleration_i.x - nav::acceleration_b.x, average_runtime);
    sleep_us(10000-(time_us_32()-t_start));
  }
}
