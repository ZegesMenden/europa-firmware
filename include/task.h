#include "core.h"
#include <hardware/timer.h>
#pragma once

typedef void(*callback_t)();

struct task_t {
	callback_t function;
	uint64_t run_interval;
	uint64_t last_run;
	uint64_t average_runtime;
	uint64_t n_runs;
};

void update_task(task_t &task) {

	// if ( task.last_run+task.run_interval < time_us_64() ) {

		uint64_t t_start = time_us_64();
		task.function();
		task.last_run = time_us_64();
		task.n_runs++;
		task.average_runtime = (task.last_run-t_start);
		
	// }

}