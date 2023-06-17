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
#include <hardware/pwm.h>
#include <stdio.h>

#include <task.h>

#include "ircodes.h"


/* 0 - 24 */
#define TX_DUTY_OFF 0
#define TX_DUTY_ON 12


/* Transmit when true, be silent when false. */
static bool tx_enabled = false;


/* Used to indicate whether to transmit or not. */
void tx_enable(bool en)
{
	tx_enabled = en;
}


/* Play single IR script to turn off one TV model. */
static void tx_play_script(struct ir_script *script)
{
	/* Prepare the GPIO pin. */
	gpio_init(IR_EN_PIN);
	gpio_disable_pulls(IR_TX_PIN);

	/* TODO: We need to write this! */

	/* Reset & pull down the pin to keep the LED off. */
	gpio_init(IR_EN_PIN);
	gpio_set_pulls(IR_TX_PIN, false, true);
}


void tx_task(void)
{
	/* Reset & pull down the pin to keep the LED off. */
	gpio_init(IR_EN_PIN);
	gpio_set_pulls(IR_TX_PIN, false, true);

	while (true) {
		if (!tx_enabled) {
			/* When TX is not enabled, we do nothing. */
			task_sleep_ms(50);
			continue;
		}

		/* Walk over the IR scripts and start blasting! */
		for (int i = 0; i < NUM_IR_SCRIPTS; i++) {
			struct ir_script *script = ir_scripts[i];

			/* We only want the European codes. */
			if (IR_EUROPE != script->region)
				continue;

			/* Indicate how far are we. */
			printf("tx: ir_script_%03i (freq=%u)\n", i, script->freq);

			/* Play the script. */
			tx_play_script(script);

			/* Pause for a bit to have some space between the
			 * transmissions. And also to keep the LEDs cool. */
			task_sleep_ms(25);
		}
	}
}
