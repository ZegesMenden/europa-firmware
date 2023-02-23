#include <stdio.h>
#include "pico/stdlib.h"
#include "core.h"
#include "pico/sem.h"
#include "drivers/perif_ctrl.h"
#include "navigation.h"
#include "datalogging.h"
#include "navigation.h"
#include "hardware/gpio.h"

int main(void)
{  
  stdio_init_all();
  perif_init();

  while(!stdio_usb_connected()) {};
  sleep_ms(1000);
  // for (int i = 0; i < 50; i++ ) { printf("%i\n", i); }
  print_compile_config();
  // for (int i = 0; i < 50; i++ ) { printf("%i\n", i); }
  nav::init_nav();
  // for (int i = 0; i < 50; i++ ) { printf("%i\n", i); }
  neopix_write(5, 5, 5);
  vehicle_state = state_nav_init;

  while(1) {
    nav::update_nav();
    sleep_ms(10);

    printf("%f, %f, %f, %f, %f\n", nav::acceleration_l.x, nav::acceleration_l.y, nav::acceleration_l.z, (float)nav::raw::read_time, (float)nav::raw::process_time);
  }
}
