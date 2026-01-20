## ATTiny85 Demo Board

This repository is just me tinkering arround with a ATTiny85.

In order to get started with the μC, I put together a small board with a few
LEDs, two buttons and a rotary encoder.
The sketch provides a small demo of digitial I/O, pulse width modulation (PWM)
and analog input.
In order to gain some low-level experience with the board, the sketch uses only
[AVR-LibC][avr-libc] and is intended to be flashed with [AVRDUDE][avrdude].

### The setup

[![The demo board soldered together and attached to a breadboard][breadboard]][breadboard]

<picture>
    <img
        src="/images/schematics.svg"
        alt="Schematics of the demo board including microcontroller and power supply"
        width="100%"
    ></img>
</picture>

### Project Structure

* [`build`](build) → Output of the build process
* [`images`](images) → Assets for documentation
* [`src`](src) → Source code

### How to build?

First, ensure that `avrdude`, `avr-binutils` and `avr-gcc` are installed.
Then, run the command `make build` to generate the binary `build/demo.hex`.
Finally, the binary is written to the μC with the command `make flash`.

**Note:** that `make flash` assumes, that a USBTiny ISP is used for flashing.
If a different programmer is to be used, the [`Makefile`](Makefile) must be
updated.

  [avrdude]: https://github.com/avrdudes/avrdude
  [avr-libc]: https://avrdudes.github.io/avr-libc/
  [breadboard]: images/breadboard.avif
  [schematics]: images/schematics.svg
