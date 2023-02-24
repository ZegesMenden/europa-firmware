#include "core.h"
#include <hardware/timer.h>
#pragma once

typedef void(*callback_t)();

struct task_t {
    callback_t function;
    uint64_t run_interval;
    uint64_t last_run;
};

template<int n = 1>
class scheduler {

private:

    task_t tasks[n];
    unsigned int n_tasks = 0;

public:

    scheduler() {};

    bool add_task(task_t task) {
        if ( n_tasks+1 < n ) {
            tasks[n_tasks++] = task;
            return 1;
        }
        return 0;
    }

    void update() {
        for ( int i = 0; i < n_tasks; i++ ) {

            if ( tasks[i].last_run + tasks[i].run_interval < time_us_64() ) {
                tasks[i].function();
                tasks[i].last_run = time_us_64();
            }

        }
    }

};