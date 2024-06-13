# NoiseCard

NoiseCard is an attempt to build a dead easy tool for monitoring ambient noise levels. The ultra-low-power microcontroller takes periodic decibel measurements while powered on, using blinking LEDs for indication. The circuit requires only a few milliwatts to run, enabling it to run off of direct solar power without a battery (though some large capacitors provide a small reserve).

Ambient noise levels in populated areas, especially North America, can often exceed what is considered safe for our ears, with prolonged exposure leading to lasting impacts on our health. Through a tool like the NoiseCard, people can become more aware of the noisy environments they're living in (and potentially take action to reduce excess noise exposure).

## Building the source

You need:

* [ChibiOS sources](https://www.chibios.org/dokuwiki/doku.php)
* The arm-none-eabi GCC toolchain (through your distro or from [ARM developer](https://developer.arm.com/downloads/-/gnu-rm))
* [GNU Make](https://www.gnu.org/software/make/)
* [OpenOCD](https://openocd.org/) or another tool to program the microcontroller
* [KiCAD](https://www.kicad.org/) if you wish to work with the hardware files

Extract ChibiOS to a folder, edit the `Makefile` so CHIBIOS points to that folder, then run `make`.

### Flashing the card

You'll need a 6-pin Tag-Connect cable (e.g. [TC2030-CTX-NL](https://www.tag-connect.com/product/tc2030-ctx-nl-6-pin-no-legs-cable-with-10-pin-micro-connector-for-cortex-processors)), compatible programmer, and OpenOCD. Power up the card and run the following command (using the appropriate interface scripts for your programmer):

```
openocd -f interface/ftdi/olimex-arm-usb-ocd-h.cfg -f interface/ftdi/olimex-arm-jtag-swd.cfg -f target/stm32g0x.cfg -c "program build/ch.hex verify reset exit"
```

## Credits

* [ESP32-I2S-SLM](https://hackaday.io/project/166867-esp32-i2s-slm) for a starting point with accurate decibel-measuring code.
* [Qfplib](https://www.quinapalus.com/qfplib.html) for providing optimized floating-point code for the Cortex-M0+ microcontroller.
* ChibiOS for providing an awesome RTOS and HAL that made firmware porting and device configuration a breeze.

