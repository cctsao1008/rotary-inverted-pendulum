#include <cstdio>

#include "pico/stdlib.h"
#include "rip/board.hpp"

#ifndef RIP_ENABLE_CDC_LOG
#define RIP_ENABLE_CDC_LOG 1
#endif

int main() {
    // Safety boundary: motor command pins are driven inactive before any
    // potentially blocking or higher-level initialization.
    rip::board::init_safe_idle();

#if RIP_ENABLE_CDC_LOG
    stdio_init_all();
    sleep_ms(500);
    std::printf("boot,target=rp2350a,board=uno_rp2350,safe_idle=1,motor_authority=0\n");
#endif

    while (true) {
#if RIP_ENABLE_CDC_LOG
        std::printf("status,safe_idle=1,motor_authority=0\n");
#endif
        sleep_ms(1000);
    }
}
