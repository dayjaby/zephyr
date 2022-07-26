.. _nucleo_f303rc_board:

ST Nucleo F303RC
################

Overview
********

The Olimexino STM32F303RC board features an ARM Cortex-M4 based STM32F303RC
mixed-signal MCU with FPU and DSP instructions capable of running at 72 MHz.
Here are some highlights of the Olimexino STM32F303RC board:

- STM32 microcontroller in LQFP64 package
- LSE crystal: 32.768 kHz crystal oscillator
- Two types of extension resources:

  - Arduino* Uno V3 connectors
  - ST morpho extension pin headers for full access to all STM32 I/Os

- On-board ST-LINK/V2-1 debugger/programmer with SWD connector
- Flexible board power supply:

  - 5 V from ST-LINK/V2-1 USB VBUS
  - External power sources: 3.3 V and 7 - 12 V on ST Zio or ST morpho
    connectors, 5 V on ST morpho connector

- One user LED
- Two push-buttons: USER and RESET

More information about the board can be found at the `Olimexino STM32F303RC website`_.
Hardware
********

The Nucleo F303RC provides the following hardware components:

- STM32F303RCT6 in QFP64 package
- ARM |reg| 32-bit Cortex |reg| -M4 CPU with FPU
- 72 MHz max CPU frequency
- VDD from 2.0 V to 3.6 V
- 256 KB Flash
- 32 + 8 KB SRAM
- RTC
- Advanced-control Timer
- General Purpose Timers (4)
- Basic Timer
- Watchdog Timers (2)
- PWM channels (18)
- SPI/I2S (2)
- I2C (3)
- USART/UART (3/3)
- USB 2.0 FS with on-chip PHY
- CAN (2)
- GPIO with external interrupt capability
- DMA channels (12)
- Capacitive sensing channels (18)
- 12-bit ADC with 40 channels (4)
- 12-bit D/A converter with two channels
- Analog comparator (7)
- Op amp (4)
- Capacitive sensing 24 channels


More information about the STM32F303RC can be found here:

- `STM32F303RC on www.st.com`_
- `STM32F303RC reference manual`_
- `STM32F303RC datasheet`_

Supported Features
==================

The default configuration can be found in the defconfig file:
``boards/arm/nucleo_f303rc/nucleo_f303rc_defconfig``

Connections and IOs
===================

The Olimexino STM32F303RC Board has 5 GPIO controllers. These controllers are
responsible for pin muxing, input/output, pull-up, etc.

Default Zephyr Peripheral Mapping:
----------------------------------

The Olimexino STM32F303RC board features an Arduino Uno V3 connector and a ST
morpho connector. Board is configured as follows:

- UART_2 TX/RX : PA2/PA3 (ST-Link Virtual Port Com)
- USER_PB   : PC13
- LD2       : PA5

System Clock
------------

The STM32F303RC System Clock can be driven by an internal or
external oscillator, as well as by the main PLL clock. By default the
System Clock is driven by the PLL clock at 72 MHz. The input to the
PLL is an 8 MHz external clock supplied by the processor of the
on-board ST-LINK/V2-1 debugger/programmer.

Serial Port
-----------

The Olimexino STM32F303RE board has 2 UARTs. The Zephyr console output is assigned
to UART2.  Default settings are 115200 8N1.

.. _Olimexino STM32F303RC website:
   https://www.olimex.com/Products/Duino/STM32/OLIMEXINO-STM32F3/open-source-hardware

.. _STM32F303RC on www.st.com:
   http://www.st.com/en/microcontrollers/stm32f303rc.html

.. _STM32F303RC reference manual:
   https://www.st.com/resource/en/reference_manual/dm00043574.pdf

.. _STM32F303RC datasheet:
   http://www.st.com/resource/en/datasheet/stm32f303rc.pdf
