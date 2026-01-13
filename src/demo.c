/*
 * This file is part of attiny85-demo-board.
 * Copyright (c) 2026 Karsten Lehmann <mail@kalehmann.de>.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *  Pinout of the Attiny85
 *
 *               ┌──┐┌──┐
 * PB5 (Reset) = │  ╰╯  │ = VCC
 * PB3         = │      │ = PB2
 * PB4         = │      │ = PB1
 * GND         = │      │ = PB0
 *               └──────┘
 */

#include <avr/io.h>
#include <util/delay.h>

void init_input(uint8_t pin) {
        /*
         * Data Direction Register port B (DDRB)
         *
         * ┌───────┬───────┬───────┬───────┬───────┬───────┬───────┬───────┐
         * │ 7     │ 6     │ 5     │ 4     │ 3     │ 2     │ 1     │ 0     │
         * ├───────┼───────┼───────┼───────┼───────┼───────┼───────┼───────┤
         * │       │       │ DDB5  │ DDB4  │ DDB3  │ DDB2  │ DDB1  │ DDB0  │
         * └───────┴───────┴───────┴───────┴───────┴───────┴───────┴───────┘
         * DDBn: Direction of Pin n. One means output and zero input.
         */
        if (pin > 5) {
                return;
        }

        DDRB &= ~(1 << pin);
}

void init_output(uint8_t pin) {
        // See init_input
        if (pin > 5) {
                return;
        }

        DDRB |= (1 << pin);
}

bool read_pin(uint8_t pin) {
        return (PINB & (1 << pin)) != 0;
}

void write_pin(uint8_t pin, bool high) {
        if (high) {
                PORTB |= (1 << pin);
        } else {
                PORTB &= ~(1 << pin);
        }
}

int main(void) {
        bool led_state = false;

        init_output(0);
        init_output(1);
        init_output(3);
        init_input(4);

        while (1) {
                if (!read_pin(4)) {
                        led_state = !led_state;
                        write_pin(0, led_state);
                        write_pin(1, led_state);
                        write_pin(3, led_state);
                }
                _delay_ms(50);
        }

        return 0;
}
