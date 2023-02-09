#include <stdio.h>
#include "pico/stdlib.h"
#include "core.h"
#include "perif_ctrl.h"

int main(void)
{  
  stdio_init_all();
  while(!stdio_usb_connected()) {};
  print_compile_config();
  perif_init();
}
