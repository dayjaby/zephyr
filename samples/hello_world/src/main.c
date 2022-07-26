/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/uart.h>

#define MY_SERIAL DT_NODELABEL(usart1)

void main(void)
{
	struct device *dev = device_get_binding(DT_LABEL(MY_SERIAL));
	printk("Hello World! %s with console %s and dev %i\n", CONFIG_BOARD, DT_LABEL(MY_SERIAL), (int)dev);
	uart_poll_out(dev, 'A');
}
