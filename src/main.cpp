#include <stdio.h>
#include "pico/stdlib.h"
#include "core.h"
#include "drivers/perif_ctrl.h"
#include "navigation.h"
#include "datalogging.h"
#include "pico/mem_ops.h"
int main(void)
{  
  stdio_init_all();
  while(!stdio_usb_connected()) {};
  print_compile_config();
  perif_init();
}
