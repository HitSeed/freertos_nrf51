# FreeRTOS (V8.2.1) port for the nRF51 MCUs using the GCC ARM Toolchain

This is a new port of the FreeRTOS (V8.2.1) for the Nordic Semiconductor nRF51x22
(Bluetooth LE and ANT+ chips with a ARM Cortex-M0 MCU). This works with the S110/S120/S130 softdevices in the nRF51_SDK_8.

## Installation

Install both http://sourceforge.net/projects/freertos/files/FreeRTOS/V8.2.1/ and
https://developer.nordicsemi.com/nRF51_SDK/nRF51_SDK_v8.x.x/
under a same directory. In that same directory untar this package (tar xzvf freertos_nrf51.tgz).
Untarring does not replace anything, it just adds two new directories under ./FreeRTOSV8.2.1/: the actual port and a demo app.

You need to have the GCC ARM Toolchain (arm-eabi-none-gcc, et. al) installed.
Configure the path of the GCC ARM installation in the file nRF51_SDK_8/components/toolchain/gcc/Makefile.posix or nRF51_SDK_8/components/toolchain/gcc/Makefile.windows.

Run 'make' in the demo directory FreeRTOSV8.2.1/FreeRTOS/Demo/CORTEX_M0_nRF51_GCC
Install the softdevice and this application with whatever tools you are using with your nRF51 board.

The things you may need to configure should be within the first ten lines of the Makefile.
The Makefile reuses the current nRF_SDK_8 setup as much as possible to minimize the use of duplicate files.

## Description

This port.c is based on the FreeRTOS ARM_CM0 code.
The main modification to that is the use of a nRF51 softdevice timer for the FreeRTOS SysTick.
This package contains a simple blinky demo based on an earlier FreeRTOS port from Shawn Nock <nock@nocko.se> (thanks!).

The options in the FreeRTOSConfig.h have been set for a quite full FreeRTOS setup
- useful for experimenting with how FreeRTOS works on the nRF51.
The setup is optimized for the newer nFR51 chips with 32KB of RAM.

Depending on the configuration, using FreeRTOS is likely to use 20 - 40 kB of flash memory
and a few kilobytes of RAM. It can run in less than that by changing the FreeRTOSConfig.h
and exclusion of optional components of FreeRTOS in the Makefile.

The vApplicationIdleHook() is implemented to call sd_app_evt_wait(). So the MCU
ought to go to sleep when there is no code that needs to be running.

## Including FreeRTOS in your own rojects

This FreeRTOS port can be included in any application that uses a softdevice from the nRF51_SDK_8.
- Take a look at the Makefile in the demo directory FreeRTOSV8.2.1/FreeRTOS/Demo/CORTEX_M0_nRF51_GCC.
  Merge the lines that contain "freertos" to your own Makefile.
- Copy the code from the demo nrf51_freertos.c file functions timers_init() and application_timers_start().
  Merge those lines to your application's timers_init() and application_timers_start() functions in the file where the softdevice is initialized.
- Add  #include:s form the demo *.c files if needed to compile.
- Copy the FreeRTOSConfig.h from the demo to your application. Configure and use FreeRTOS as needed in your application.
- If you like to have your own hooks for handling FreeRTOS error conditions, copy and modify the functions from the end of the nrf51_freertos.c to your own project.

## Testing status and feedback

This successfully compiled and linked with S110, S120 and S130 softdevices (just change the definition in the beginning of hte Makefile) but so far it has been tested only with S110.
Stack overflow checking in FreeRTOS did not work; I will look at that later (by default that feature is disabled in the FreeRTOSConfig.h).

I have a test running with a couple of FreeRTOS tasks and queues, multiple nFR timers, I2C peripherals and all this seems to work properly with Bluetooth LE connections.
I just finished this port and test, many things have not been tested yet.

This has been developed in an OSX machine and it should run without modifications at least in Linux.
Please let me know if any changes are needed for compiling the demo in a Windows machine.

Please comment, ask questions, provide patches etc. mainly in the FreeRTOS forum as instructed in
 http://www.freertos.org/RTOS-contributed-ports.html


