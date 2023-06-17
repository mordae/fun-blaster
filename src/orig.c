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
#include <pico/multicore.h>
#include <hardware/adc.h>
#include <hardware/dma.h>
#include <hardware/pwm.h>
#include <tusb.h>

#include <hardware/structs/adc.h>
#include <hardware/structs/dma.h>

#include <switch.h>
#include <task.h>
#include <math.h>

#include <stdlib.h>

#include "ircodes.h"
#include "filter.h"
#include "ws.h"


static task_t switch_task_id;
static task_t stats_task_id;
static task_t rx_task_id;
static task_t tx_task_id;


#define SYS_CLOCK_HZ (125 * 1000 * 1000)


/* 0 - 100 */
#define TX_DUTY_OFF 0
#define TX_DUTY_ON 12

/* 0 - 1000 */
#define BZ_DUTY_OFF 0
#define BZ_DUTY_ON 50


static bool tx_enable = false;


static void switch_task(void)
{
	switch_init();
	switch_config(0, SW1_PIN);

	while (true) {
		struct switch_event event;
		switch_read_blocking(&event);
		printf("sw: num=%i, sw=%i\n", event.num, event.sw);

		tx_enable = event.sw;

		//pwm_set_gpio_level(IR_TX_PIN, TX_DUTY_ON * event.sw);
		//pwm_set_gpio_level(BZ_PIN, BZ_DUTY_ON * event.sw);
	}
}


static void stats_task(void)
{
	while (true) {
		task_sleep_ms(10 * 1000);
		for (unsigned i = 0; i < NUM_CORES; i++)
			task_stats_report_reset(i);
	}
}


static void rx_init(void)
{
	adc_init();

	/*
	 * 48_000_000 / 253 = 189_723 Hz
	 *
	 * Why this weird number?
	 * Well because 5-sample moving average filter happens to hit zero
	 * at 38 kHz, meaning that if we instead subtract it, we obtain a
	 * pretty good high-pass filter at our band of interest.
	 */
	adc_set_clkdiv(253 - 1);
	adc_gpio_init(IR_RX_PIN);
	gpio_disable_pulls(IR_RX_PIN);

	gpio_init(IR_EN_PIN);
	gpio_disable_pulls(IR_EN_PIN);
	gpio_set_dir(IR_EN_PIN, GPIO_OUT);
	gpio_put(IR_EN_PIN, 1);

	/* We need to drive the pump oscillator. */
	gpio_disable_pulls(IR_OSC_PIN);
	gpio_set_function(IR_OSC_PIN, GPIO_FUNC_PWM);
	pwm_config pwm_conf = pwm_get_default_config();
	pwm_init(IR_OSC_SLICE, &pwm_conf, false);

	/* 125_000_000 Hz / 25 / 100 = 50_000 Hz */
	pwm_set_clkdiv_int_frac(IR_OSC_SLICE, 25, 0);
	pwm_set_wrap(IR_OSC_SLICE, 99);
	pwm_set_gpio_level(IR_OSC_PIN, 50);
	pwm_set_enabled(IR_OSC_SLICE, true);
}


static void rx_block(int16_t *block)
{
	static int32_t h4 = 0, h3 = 0, h2 = 0, h1 = 0, h0 = 0;
	static int32_t l3 = 0, l2 = 0, l1 = 0, l0 = 0;
	static int32_t dc1 = 0, dc2 = 0;

	for (int i = 0; i < 1024; i++) {
		int32_t tmp = block[i];

		/* Update DC offset, but veeeery slowly. */
		dc1 = (dc1 * 1023 + (tmp << 8)) >> 10;

		/* Subtract from DC. We use common emitter topology. */
		tmp = (dc1 >> 8) - tmp;

		/* High-pass filter. 51 / 256 approximates 5. */
		h4 = h3; h3 = h2; h1 = h2; h1 = h0; h0 = tmp;
		tmp = tmp - ((h4 + h3 + h2 + h1 + h0) * 51) / 256;

		/* Absolute value. Basically AM detector. */
		if (tmp < 0)
			tmp = -tmp;

		/* Low pass filter before decimation. */
		l3 = l2; l2 = l1; l1 = l0; l0 = tmp;
		tmp = (l3 + l2 + l1 + l0) / 4;

		block[i] = tmp;
	}

	static int32_t ll3 = 0, ll2 = 0, ll1 = 0, ll0 = 0;

	for (int i = 0, ii = 0; i < 512; i++, ii += 2) {
		/* Decimate by 2. */
		int32_t tmp = block[ii] + block[ii + 1];

		/* Another filtering pass before next decimation. */
		ll3 = ll2; ll2 = ll1; ll1 = ll0; ll0 = tmp;
		tmp = (ll3 + ll2 + ll1 + ll0) / 4;

		block[i] = tmp;
	}

	/* Low-pass filter with about 6.6 kHz cutoff. */
	static struct fir11 lp = {
		.params = { -767, -1952, -145, 7333, 17294, 22012, 17294, 7333, -145, -1952, -767 },
	};

	for (int i = 0, ii = 0; i < 256; i++, ii += 2) {
		/* Decimate by 2. */
		int32_t tmp = block[ii] + block[ii + 1];

		/* Apply FIR low-pass filter. */
		tmp = fir11_add(&lp, tmp);

		block[i] = tmp;
	}

	for (int i = 0, ii = 0; i < 64; i++, ii += 4) {
		/* Final decimation by 4. */
		int32_t tmp = block[ii] + block[ii + 1] + block[ii + 2] + block[ii + 3];

		/* Update DC offset, but veeeery slowly. */
		dc2 = (dc2 * 127 + (tmp << 8)) >> 7;
		tmp -= dc2 >> 8;

		block[i] = tmp;
	}

#if 0
	/* Activate the buzzer when where was a signal present. */
	pwm_set_gpio_level(BZ_PIN, BZ_DUTY_OFF);

	for (int i = 0; i < 64; i++)
		if (block[i] > 15)
			pwm_set_gpio_level(BZ_PIN, BZ_DUTY_ON);
#endif

#if 1
	/* Debug print of the line just to get some feedback. */
	char line[80] = {0};

	for (int i = 0; i < 64; i++) {
		int32_t tmp = block[i];

		if (tmp > 30)
			line[i] = '#';
		else if (tmp > 15)
			line[i] = '-';
		else
			line[i] = ' ';
	}

	puts(line);
#endif
}


static void rx_task(void)
{
	size_t current = 0;
	static int16_t buffer[16384] __attribute__((__aligned__((32768))));

	gpio_put(IR_EN_PIN, 0);

	int dma_chan = dma_claim_unused_channel(true);
	dma_channel_config dma = dma_channel_get_default_config(dma_chan);
	channel_config_set_transfer_data_size(&dma, DMA_SIZE_16);
	channel_config_set_read_increment(&dma, false);
	channel_config_set_write_increment(&dma, true);
	channel_config_set_ring(&dma, true, 15);
	channel_config_set_irq_quiet(&dma, true);
	channel_config_set_dreq(&dma, DREQ_ADC);

	adc_fifo_drain();
	dma_channel_configure(dma_chan, &dma, buffer, &adc_hw->fifo, UINT32_MAX, true);
	adc_fifo_setup(true, true, 1, false, false);
	adc_run(true);

	while (true) {
		if (!dma_channel_is_busy(dma_chan)) {
			dma_channel_start(dma_chan);
			continue;
		}

		size_t offset = ((dma_hw->ch[dma_chan].write_addr & 32767) >> 10) & 15;
		static int16_t block[1024] = {0};

		while (current != offset) {
			memcpy(block, buffer + (current << 10), 2048);
			current = (current + 1) & 15;
			rx_block(block);
		}


		task_sleep_ms(10);
	}
}


static void tx_play_script(struct ir_script *script)
{
	gpio_init(IR_EN_PIN);
	gpio_disable_pulls(IR_TX_PIN);

	if (script->freq) {
		gpio_set_function(IR_TX_PIN, GPIO_FUNC_PWM);

		pwm_config pwm_conf = pwm_get_default_config();
		pwm_init(IR_TX_SLICE, &pwm_conf, false);

		float div = (SYS_CLOCK_HZ / 24.0f) / (float)script->freq;
		pwm_set_clkdiv(IR_TX_SLICE, div);

		pwm_set_wrap(IR_TX_SLICE, 23);
		pwm_set_gpio_level(IR_TX_PIN, TX_DUTY_OFF);
		pwm_set_enabled(IR_TX_SLICE, true);

		for (unsigned i = 0; i < script->num_codes; i++) {
			struct ir_code *code = &script->codes[i];
			pwm_set_gpio_level(IR_TX_PIN, TX_DUTY_ON * code->en);
			task_sleep_us(10 * code->us);
		}

		pwm_set_enabled(IR_TX_SLICE, false);
	} else {
		gpio_set_dir(IR_TX_PIN, GPIO_OUT);

		for (unsigned i = 0; i < script->num_codes; i++) {
			struct ir_code *code = &script->codes[i];
			gpio_put(IR_TX_PIN, code->en);
			task_sleep_us(10 * code->us);
		}

		gpio_put(IR_TX_PIN, 0);
	}

	gpio_init(IR_EN_PIN);
	gpio_set_pulls(IR_TX_PIN, false, true);
}


static void tx_task(void)
{
	while (true) {
		if (!tx_enable) {
			ws_set_rgb(0, 3, 0);
			task_sleep_ms(50);
			continue;
		}

		for (int i = 0; i < NUM_IR_SCRIPTS; i++) {
			struct ir_script *script = ir_scripts[i];

			if (IR_EUROPE != script->region)
				continue;

			printf("tx: ir_script_%03i (freq=%u)\n", i, script->freq);
			ws_set_rgb(0, 0, 7);

			tx_play_script(script);

			ws_set_rgb(0, 3, 0);
			task_sleep_ms(25);
		}
	}
}


/*
 * Our buzzer is "QMB-09B-05".
 *
 * According to the data sheet, it peaks at 2_700 Hz, 5_500 Hz,
 * 1_800 Hz and 1_000 Hz, in that order.
 *
 * It is unusable above 7_000 Hz.
 */
static void bz_init(void)
{
	gpio_disable_pulls(BZ_PIN);
	gpio_set_function(BZ_PIN, GPIO_FUNC_PWM);
	pwm_config pwm_conf = pwm_get_default_config();
	pwm_init(BZ_SLICE, &pwm_conf, false);

	/* 125_000_000 Hz / 45 / 1000 =~ 2_777 Hz */
	pwm_set_clkdiv_int_frac(BZ_SLICE, 45, 0);
	pwm_set_wrap(BZ_SLICE, 999);
	pwm_set_gpio_level(BZ_PIN, BZ_DUTY_OFF);
	pwm_set_enabled(BZ_SLICE, true);
}


/*
 * We have a single WS2812B LED.
 */
static void ws_init(void)
{
	gpio_init(LED_PIN);
	gpio_disable_pulls(LED_PIN);
	gpio_set_dir(LED_PIN, GPIO_OUT);
	gpio_put(LED_PIN, 0);
}


int main()
{

	/* Bring up the LED and indicate we are booting. */
	ws_init();
	ws_set_rgb(3, 0, 0);

	/* Initialize stdio over USB. */
	stdio_init_all();

	/* Initialize task scheduler. */
	task_init();

	/* Wait up to 2s to see if a console is attached. */
	for (int i = 0; i < 20; i++) {
		if (stdio_usb_connected())
			break;

		ws_set_rgb(3 * (i & 1), 0, 0);
		sleep_ms(100);
	}

	ws_set_rgb(3, 0, 0);
	printf("Welcome! Have a safe and productive day!\n");

	/* Initialize peripherals. */
	rx_init();
	bz_init();

	/* Interprets switch events. */
	switch_task_id = task_create_on_core(0, switch_task, 1536);
	task_set_name(switch_task_id, "switch");
	task_set_priority(switch_task_id, 3);
	task_set_ready(switch_task_id);

	/* Prints task statistics. */
	stats_task_id = task_create_on_core(0, stats_task, 1536);
	task_set_name(stats_task_id, "stats");
	task_set_priority(stats_task_id, 1);
	task_set_ready(stats_task_id);

	/* Modulate transmitted IR signal. */
	tx_task_id = task_create_on_core(0, tx_task, 1536);
	task_set_name(tx_task_id, "tx");
	task_set_priority(tx_task_id, 0);
	task_set_ready(tx_task_id);

	/* Receive IR signal. */
	rx_task_id = task_create_on_core(1, rx_task, 1536);
	task_set_name(rx_task_id, "rx");
	task_set_priority(rx_task_id, 0);
	//task_set_ready(rx_task_id);

	/* Run tasks on the second core. */
	multicore_reset_core1();
	multicore_launch_core1(task_run_loop);

	/* Run tasks on this core. */
	task_run_loop();
}
