/*
 * Copyright (c) 2018 - 2019 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT litex_timer0

#include <kernel.h>
#include <arch/cpu.h>
#include <device.h>
#include <irq.h>
#include <spinlock.h>
#include <drivers/timer/system_timer.h>

#define CYC_PER_TICK ((uint32_t)((uint64_t)sys_clock_hw_cycles_per_sec()        \
                              / (uint64_t)CONFIG_SYS_CLOCK_TICKS_PER_SEC))
#define TICKLESS IS_ENABLED(CONFIG_TICKLESS_KERNEL)

#define TIMER_BASE	        DT_INST_REG_ADDR(0)
#define TIMER_LOAD_ADDR		((TIMER_BASE) + 0x00)
#define TIMER_RELOAD_ADDR	((TIMER_BASE) + 0x04)
#define TIMER_EN_ADDR		((TIMER_BASE) + 0x08)
#define TIMER_UPDATE_VALUE_ADDR	((TIMER_BASE) + 0x0c)
#define TIMER_VALUE_ADDR	((TIMER_BASE) + 0x10)
#define TIMER_EV_STATUS_ADDR	((TIMER_BASE) + 0x14)
#define TIMER_EV_PENDING_ADDR	((TIMER_BASE) + 0x18)
#define TIMER_EV_ENABLE_ADDR	((TIMER_BASE) + 0x1c)

// not defined anymore:
#define TIMER_TOTAL_UPDATE	((TIMER_BASE) + 0x20)
#define TIMER_TOTAL		((TIMER_BASE) + 0x24)

#define TIMER_EV	0x1
#define TIMER_IRQ	DT_INST_IRQN(0)
#define TIMER_DISABLE	0x0
#define TIMER_ENABLE	0x1
#define UPDATE_TOTAL	0x1

#define MAX_CYC 0xffffffffu
#define MAX_TICKS ((MAX_CYC - CYC_PER_TICK) / CYC_PER_TICK)
#define MIN_DELAY 1000

static struct k_spinlock lock;
static uint64_t last_count;

static void set_mtimecmp(uint64_t time)
{
#ifdef CONFIG_64BIT
	*(volatile uint64_t *)RISCV_MTIMECMP_BASE = time;
#ifdef RISCV_MTIME_LATCH
	volatile uint32_t *latch = (uint32_t *)RISCV_MTIME_LATCH;
	*latch = 1;
#endif
#else
	volatile uint32_t *r = (uint32_t *)RISCV_MTIMECMP_BASE;

	/* Per spec, the RISC-V MTIME/MTIMECMP registers are 64 bit,
	 * but are NOT internally latched for multiword transfers.  So
	 * we have to be careful about sequencing to avoid triggering
	 * spurious interrupts: always set the high word to a max
	 * value first.
	 */
#ifdef RISCV_MTIME_LATCH
	volatile uint32_t *latch = (uint32_t *)RISCV_MTIME_LATCH;
	*latch = 0;
	r[0] = (uint32_t)time;
	r[1] = (uint32_t)(time >> 32);
	*latch = 1;
#else
	r[1] = 0xffffffff;
	r[0] = (uint32_t)time;
	r[1] = (uint32_t)(time >> 32);
#endif
#endif
}

static uint64_t mtime(void)
{
#ifdef CONFIG_64BIT
	return *(volatile uint64_t *)RISCV_MTIME_BASE;
#else
	volatile uint32_t *r = (uint32_t *)RISCV_MTIME_BASE;
	uint32_t lo, hi;

#ifdef RISCV_MTIME_LATCH
	volatile uint32_t *latch = (uint32_t *)RISCV_MTIME_LATCH;
	*latch = 1;
#endif
	/* Likewise, must guard against rollover when reading */
	do {
		hi = r[1];
		lo = r[0];
	} while (r[1] != hi);
#ifdef RISCV_MTIME_LATCH
	*latch = 0;
#endif

	return (((uint64_t)hi) << 32) | lo;
#endif
}

static void litex_timer_irq_handler(const void *device)
{
	ARG_UNUSED(device);
	int key = irq_lock();

	sys_write8(TIMER_EV, TIMER_EV_PENDING_ADDR);

	uint64_t now = mtime();
	uint32_t dticks = (uint32_t)((now - last_count) / CYC_PER_TICK);

	last_count += dticks * CYC_PER_TICK;

	if (!TICKLESS) {
		uint64_t next = last_count + CYC_PER_TICK;

		if ((int64_t)(next - now) < MIN_DELAY) {
			next += CYC_PER_TICK;
		}
		set_mtimecmp(next);
	}

	irq_unlock(key);
	z_clock_announce(IS_ENABLED(CONFIG_TICKLESS_KERNEL) ? dticks : 1);
}

uint32_t z_timer_cycle_get_32(void)
{
	static struct k_spinlock lock;
	uint32_t timer_total;
	k_spinlock_key_t key = k_spin_lock(&lock);

	litex_write8(UPDATE_TOTAL, TIMER_TOTAL_UPDATE);
	timer_total = (uint32_t)litex_read64(TIMER_TOTAL);

	k_spin_unlock(&lock, key);

	return timer_total;
}

int z_clock_driver_init(const struct device *device)
{
	ARG_UNUSED(device);
	IRQ_CONNECT(TIMER_IRQ, DT_INST_IRQ(0, priority),
			litex_timer_irq_handler, NULL, 0);
	last_count = mtime();
	set_mtimecmp(last_count + CYC_PER_TICK);
	irq_enable(TIMER_IRQ);

	sys_write8(TIMER_DISABLE, TIMER_EN_ADDR);

	for (int i = 0; i < 4; i++) {
		sys_write8(k_ticks_to_cyc_floor32(1) >> (24 - i * 8),
				TIMER_RELOAD_ADDR + i * 0x4);
		sys_write8(k_ticks_to_cyc_floor32(1) >> (24 - i * 8),
				TIMER_LOAD_ADDR + i * 0x4);
	}

	sys_write8(TIMER_ENABLE, TIMER_EN_ADDR);
	sys_write8(sys_read8(TIMER_EV_PENDING_ADDR), TIMER_EV_PENDING_ADDR);
	sys_write8(TIMER_EV, TIMER_EV_ENABLE_ADDR);

	return 0;
}

void z_clock_set_timeout(int32_t ticks, bool idle)
{
	ARG_UNUSED(idle);

#if defined(CONFIG_TICKLESS_KERNEL)
	/* RISCV has no idle handler yet, so if we try to spin on the
	 * logic below to reset the comparator, we'll always bump it
	 * forward to the "next tick" due to MIN_DELAY handling and
	 * the interrupt will never fire!  Just rely on the fact that
	 * the OS gave us the proper timeout already.
	 */
	if (idle) {
		return;
	}

	ticks = ticks == K_TICKS_FOREVER ? MAX_TICKS : ticks;
	ticks = CLAMP(ticks - 1, 0, (int32_t)MAX_TICKS);

	k_spinlock_key_t key = k_spin_lock(&lock);
	uint64_t now = mtime();
	uint32_t adj, cyc = ticks * CYC_PER_TICK;

	/* Round up to next tick boundary. */
	adj = (uint32_t)(now - last_count) + (CYC_PER_TICK - 1);
	if (cyc <= MAX_CYC - adj) {
		cyc += adj;
	} else {
		cyc = MAX_CYC;
	}
	cyc = (cyc / CYC_PER_TICK) * CYC_PER_TICK;

	if ((int32_t)(cyc + last_count - now) < MIN_DELAY) {
		cyc += CYC_PER_TICK;
	}

	set_mtimecmp(cyc + last_count);
	k_spin_unlock(&lock, key);
#endif
}

uint32_t z_clock_elapsed(void)
{
	if (!TICKLESS) {
		return 0;
	}

	k_spinlock_key_t key = k_spin_lock(&lock);
	uint32_t ret = ((uint32_t)mtime() - (uint32_t)last_count) / CYC_PER_TICK;

	k_spin_unlock(&lock, key);
	return ret;
}

