#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "drivers/ws2812.pio.h"
#include "physics.h"

int main(void)
{  
    stdio_init_all();

    neopix_init(16);
    neopix_write(5, 5, 5);

}
