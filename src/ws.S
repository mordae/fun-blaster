#include "pico/asm_helper.S"

pico_default_asm_setup

.section .time_critical.ws_led
.global ws_send_raw
.type ws_send_raw,%function
.thumb_func

// r0 holds 24 bits to send followed by 8 padding bits
ws_send_raw:
	push {r4}

	movs r2, #208		// r2 = 0xd0
	lsls r2, r2, #24	// r2 = 0xd0000000 (SIO_BASE)

	movs r3, #1
	lsls r3, r3, #LED_PIN	// r3 has pin mask

	movs r1, #25		// r1 counts the bits to send

	cpsid i			// disable interrupts for precise timing

send_bit:
	subs r1, r1, #1		// [1] decrease count of bits to send
	beq end			// [1] quit if done

	lsls r0, #1		// [1] shift top bit to carry
	bcs send_bit_1		// [1] branch on the carry

send_bit_0:
	nop			// [1] to compensate for branch difference
	str r3, [r2, #20]	// [1] r3 => 0xd0000014 (SIO_GPIO_OUT_SET)

	movs r4, #16		// [1] r4 has delay counter
L0:	subs r4, r4, #1		// [1,2] decrement by 1
	bne L0			// [1] loop while != 0
	nop			// [1] to get to 46 cycles delay

	str r3, [r2, #24]	// [1] r3 => 0xd0000018 (SIO_GPIO_OUT_CLR)

	movs r4, #30		// [1] r4 has delay counter
L1:	subs r4, r4, #1		// [1,2] decrement by 1
	bne L1			// [1] loop while != 0
	nop; nop		// [2] to get to 92 cycles delay

	b send_bit		// [2] loop to next bit

send_bit_1:
				// [1] from branch
	str r3, [r2, #20]	// [1] r3 => 0xd0000014 (SIO_GPIO_OUT_SET)

	movs r4, #33		// [1] r4 has delay counter
L2:	subs r4, r4, #1		// [1,2] decrement by 1
	bne L2			// [1] loop while != 0

	str r3, [r2, #24]	// [1] r3 => 0xd0000018 (SIO_GPIO_OUT_CLR)

	movs r4, #14		// [1] r4 has delay counter
L3:	subs r4, r4, #1		// [1,2] decrement by 1
	bne L3			// [1] loop while != 0

	b send_bit		// [2] loop to next bit

end:
	cpsie i			// re-enable interrupts
	pop {r4}
	bx lr

.global literals
literals:
.ltorg

.end
