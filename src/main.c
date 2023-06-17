/*
 * Copyright (C) 2022 Jan Hamal Dvořák <mordae@anilinux.org>
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <pico/stdlib.h>
#include <stdlib.h>
#include <tusb.h>

#include <switch.h>
#include <task.h>

#include "ircodes.h"
#include "ws.h"
#include "tx.h"


/* We need these to keep track of registered tasks. */
static task_t switch_task_id;
static task_t stats_task_id;
static task_t tx_task_id;


/*
 * This task receives information about button events (presses and releases)
 * and allows us to react to them.
 */
static void switch_task(void)
{
	/* Initialize the library that takes care of switches for us. */
	switch_init();

	/* Configure our only switch to use the correct GPIO pin. */
	switch_config(0, SW1_PIN);

	while (true) {
		/* Wait until next event arrives. */
		struct switch_event event;
		switch_read_blocking(&event);

		/* Print what we have received. */
		printf("sw: num=%i, sw=%i\n", event.num, event.sw);

		if (event.sw) {
			/* Change LED color. */
			ws_set_rgb(0, 7, 0);

			/* Enable IR blasting. */
			tx_enable(true);
		} else {
			/* Reset LED color. */
			ws_set_rgb(3, 0, 0);

			/* Disable IR blasting. */
			tx_enable(false);
		}
	}
}


/*
 * This tasks prints information about running tasks every 10s.
 */
static void stats_task(void)
{
	while (true) {
		task_sleep_ms(10 * 1000);
		for (unsigned i = 0; i < NUM_CORES; i++)
			task_stats_report_reset(i);
	}
}


/*
 * Prepare to communicate with out WS2812B RGB LED.
 */
static void ws_init(void)
{
	/* We are using SIO to control this pin. */
	gpio_init(LED_PIN);

	/* We are going to control the voltage. */
	gpio_disable_pulls(LED_PIN);
	gpio_set_dir(LED_PIN, GPIO_OUT);

	/* It is going to be zero volts for now. */
	gpio_put(LED_PIN, 0);
}


/*
 * This is where the program starts.
 */
int main()
{
	/* Bring up the LED and indicate we are booting. */
	ws_init();
	ws_set_rgb(3, 0, 0);

	/* Initialize text input/output over USB. */
	stdio_init_all();

	/* Wait up to 2s to see if a console is attached. */
	for (int i = 0; i < 20; i++) {
		/* If we are connected, break from the loop and continue
		 * with the bootup process. */
		if (stdio_usb_connected())
			break;

		/* Blink the LED as we wait so that the user has
		 * some feedback and knows we are alive. */
		ws_set_rgb(3 * (i & 1), 0, 0);

		/* Do nothing for 1/10 of second. */
		sleep_ms(100);
	}

	/* Greet any potential user reading us over USB. */
	printf("Welcome! Have a safe and productive day!\n");

	/* Stop blinking but stay lit. */
	ws_set_rgb(3, 0, 0);

	/* Initialize the task scheduler. */
	task_init();

	/* Prepare the "switch" task. */
	switch_task_id = task_create(switch_task, 1536);
	task_set_name(switch_task_id, "switch");
	task_set_priority(switch_task_id, 3);
	task_set_ready(switch_task_id);

	/* Prepare the "stats" task. */
	stats_task_id = task_create(stats_task, 1536);
	task_set_name(stats_task_id, "stats");
	task_set_priority(stats_task_id, 1);
	task_set_ready(stats_task_id);

	/* Prepare the "tx" task. */
	tx_task_id = task_create(tx_task, 1536);
	task_set_name(tx_task_id, "tx");
	task_set_priority(tx_task_id, 1);
	task_set_ready(tx_task_id);

	/* Run tasks on this core indefinitely. */
	task_run_loop();
}
