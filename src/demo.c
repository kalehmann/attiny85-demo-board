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

void init_adc(uint8_t pin) {
        /*
         * ADC Control and Status Register A (ADCSRA)
         *
         * ┌───────┬───────┬───────┬───────┬───────┬───────┬───────┬───────┐
         * │ 7     │ 6     │ 5     │ 4     │ 3     │ 2     │ 1     │ 0     │
         * ├───────┼───────┼───────┼───────┼───────┼───────┼───────┼───────┤
         * │ ADEN  │ ADSC  │ ADATE │ ADIF  │ ADIE  │ ADPS2 │ ADPS1 │ ADPS0 │
         * └───────┴───────┴───────┴───────┴───────┴───────┴───────┴───────┘
         *
         * ADEN: Enable ADC
         * ADSC: Start conversion (reset to zero by hardware)
         * ADATE: Automatically trigger ADC conversion
         * ADIF: ADC interrupt flag
         * ADIE: ADC interrupt enable
         * ADPS0, ADPS1, ADPS2: Prescaler selection for ADC clock frequency
         */
        ADCSRA |= (1 << ADEN)
                | (1 << ADSC)
                | (1 << ADATE);

        /*
         * ADC Multiplexer Selection Register
         *
         * ┌───────┬───────┬───────┬───────┬───────┬───────┬───────┬───────┐
         * │ 7     │ 6     │ 5     │ 4     │ 3     │ 2     │ 1     │ 0     │
         * ├───────┼───────┼───────┼───────┼───────┼───────┼───────┼───────┤
         * │ REFS1 │ REFS0 │ ADLAR │ REFS2 │ MUX3  │ MUX2  │ MUX1  │ MUX0  │
         * └───────┴───────┴───────┴───────┴───────┴───────┴───────┴───────┘
         *
         * ADLAR: If set, data is stored left shifted in ADCH and ADCL, otherwise
         *        right shifted.
         * REFS0, REFS1, REFS2: reference voltage selection. Some important values:
         *                      ┌───────┬───────┬───────┬─────────────┐
         *                      │ REFS2 │ REFS1 │ REFS0 │             │
         *                      ├───────┼───────┼───────┼─────────────┤
         *                      │       │ 0     │ 0     │ VCC         │
         *                      ├───────┼───────┼───────┼─────────────┤
         *                      │       │ 0     │ 1     │ PB0 (AREF)  │
         *                      ├───────┼───────┼───────┼─────────────┤
         *                      │ 0     │ 1     │ 0     │ 1.1V        │
         *                      └───────┴───────┴───────┴─────────────┘
         * MUX0, MUX1, MUX2, MUX3: ADC Channel selection. Some important values:
         *                         ┌───────┬───────┬───────┬───────┬───────┐
         *                         │ MUX3  │ MUX2  │ MUX1  │ MUX0  │       │
         *                         ├───────┼───────┼───────┼───────┼───────┤
         *                         │ 0     │ 0     │ 0     │ 0     │ PB5   │
         *                         ├───────┼───────┼───────┼───────┼───────┤
         *                         │ 0     │ 0     │ 0     │ 1     │ PB2   │
         *                         ├───────┼───────┼───────┼───────┼───────┤
         *                         │ 0     │ 0     │ 1     │ 0     │ PB4   │
         *                         ├───────┼───────┼───────┼───────┼───────┤
         *                         │ 0     │ 0     │ 1     │ 1     │ PB3   │
         *                         └───────┴───────┴───────┴───────┴───────┘
         */
        switch (pin) {
        case 2:
                ADMUX = (1 << MUX0);
                break;
        case 3:
                ADMUX = (1 << MUX1) | (1 << MUX0);
                break;
        case 4:
                ADMUX = (1 << MUX1);
                break;
        case 5:
                ADMUX = 0;
                break;
        }

        /*
         * ADC Control and Status Register B (ADCSRB)
         *
         * ┌───────┬───────┬───────┬───────┬───────┬───────┬───────┬───────┐
         * │ 7     │ 6     │ 5     │ 4     │ 3     │ 2     │ 1     │ 0     │
         * ├───────┼───────┼───────┼───────┼───────┼───────┼───────┼───────┤
         * │       │ ACME  │       │       │       │ ADTS2 │ ADTS1 │ ADTS0 │
         * └───────┴───────┴───────┴───────┴───────┴───────┴───────┴───────┘
         *
         * ACME: Analog Comparator Multiplexer Enable (only relevant when ADEN is
         *       zero)
         * ADTS0, ADTS1, ADTS2: ADC Auto Trigger Source. Some important values:
         *                      ┌───────┬───────┬───────┬───────────────────┐
         *                      │ ADTS2 │ ADTS1 │ ADTS0 │                   │
         *                      ├───────┼───────┼───────┼───────────────────┤
         *                      │ 0     │ 0     │ 0     │ Free running mode │
         *                      ├───────┼───────┼───────┼───────────────────┤
         *                      │ 0     │ 0     │ 1     │ Analog comparator │
         *                      └───────┴───────┴───────┴───────────────────┘
         *                      Free running mode means next conversion is started
         *                      Automatically
         */
        ADCSRB = 0;
}

void init_fast_pwm(bool enableOC0A, bool enableOC0B) {
        /*
         * General Timer/Counter Control Register (GTCCR)
         *
         * ┌───────┬───────┬───────┬───────┬───────┬───────┬───────┬───────┐
         * │ 7     │ 6     │ 5     │ 4     │ 3     │ 2     │ 1     │ 0     │
         * ├───────┼───────┼───────┼───────┼───────┼───────┼───────┼───────┤
         * │ TSM   │       │       │       │       │       │       │ PSR0  │
         * └───────┴───────┴───────┴───────┴───────┴───────┴───────┴───────┘
         *
         * TSM: Timer/Counter Synchronization Mode. When set to one, the value
         *      in PSR0 is kept.
         * PSR0: Prescaler Reset Timer/Counter 0. When set to one, the timer or
         *       counter is reset. This is automatically cleared except when TSM
         *       is set.
         */


        // Disable timer during configuration
        GTCCR |= (1 << TSM) | (1 << PSR0);

        /*
         * Timer/Counter Control Register A (TCCR0A)
         *
         * ┌────────┬────────┬────────┬────────┬─────┬─────┬───────┬───────┐
         * │ 7      │ 6      │ 5      │ 4      │ 3   │ 2   │ 1     │ 0     │
         * ├────────┼────────┼────────┼────────┼─────┼─────┼───────┼───────┤
         * │ COM0A1 │ COM0A0 │ COM0B1 │ COM0B0 │     │     │ WGM01 │ WGM00 │
         * └────────┴────────┴────────┴────────┴─────┴─────┴───────┴───────┘
         *
         * Timer/Counter Control Register B (TCCR0B)
         *
         * ┌───────┬───────┬───────┬───────┬───────┬───────┬───────┬───────┐
         * │ 7     │ 6     │ 5     │ 4     │ 3     │ 2     │ 1     │ 0     │
         * ├───────┼───────┼───────┼───────┼───────┼───────┼───────┼───────┤
         * │ FOC0A │ FOC0B │       │       │ WGM02 │ CS02  │ CS01  │ CS00  │
         * └───────┴───────┴───────┴───────┴───────┴───────┴───────┴───────┘
         *
         * WGM00, WGM01, WGM02: Waveform Generation Mode. Some important values:
         *                      ┌───────┬───────┬───────┬───────────────────┐
         *                      │ WGM02 │ WGM01 │ WGM00 │                   │
         *                      ├───────┼───────┼───────┼───────────────────┤
         *                      │ 0     │ 0     │ 0     │ Normal mode       │
         *                      ├───────┼───────┼───────┼───────────────────┤
         *                      │ 0     │ 0     │ 1     │ Phase correct PWM │
         *                      ├───────┼───────┼───────┼───────────────────┤
         *                      │ 0     │ 1     │ 1     │ Fast PWM to 0xFF  │
         *                      └───────┴───────┴───────┴───────────────────┘
         *
         *                      In Fast PWM mode, the counter counts continuously
         *                      from 0 to 0xFF and then resets.
         *
         * COM0A1, COMA0: Compare Output Mode for OC0A
         * COM0B1, COMB0: Compare Output Mode for OC0B
         *                Configuration in Fast PWM:
         *                ┌────────┬────────┬───────────────────────┐
         *                │ COM0*1 │ COM0*0 │                       │
         *                ├────────┼────────┼───────────────────────┤
         *                │ 0      │ 0      │ Normal port operation │
         *                ├────────┼────────┼───────────────────────┤
         *                │ 1      │ 0      │ Non inverting mode    │
         *                ├────────┼────────┼───────────────────────┤
         *                │ 0      │ 1      │ Inverting mode        │
         *                └────────┴────────┴───────────────────────┘
         *
         *                Non inverting mode means, the port is set high when the
         *                counter is reset to zero and is set low when the
         *                counter matches the value in OCR0*. That means, the
         *                duty cycle is OCR0* / 0xFF.
         *
         *                In inverting mode, the port is set high when the
         *                counter matches the value in OCR0* and low when the
         *                counter resets.
         *
         * CS02, CS01, CS00: Clock select. Important values:
         *                   ┌──────┬──────┬──────┬──────────────────────────┐
         *                   │ CS02 │ CS01 │ CS00 │                          │
         *                   ├──────┼──────┼──────┼──────────────────────────┤
         *                   │ 0    │ 0    │ 0    │ No clock, timer stopped  │
         *                   ├──────┼──────┼──────┼──────────────────────────┤
         *                   │ 0    │ 0    │ 1    │ I/O clock, no prescaling │
         *                   └──────┴──────┴──────┴──────────────────────────┘
         */

        // Set Fast PWM mode
        TCCR0A = (1 << WGM01) | (1 << WGM00);

        if (enableOC0A) {
                // Set OC0A to non inverting mode
                TCCR0A |= (1 << COM0A1);
        }
        if (enableOC0B) {
                // Set OC0B to non inverting mode
                TCCR0A |= (1 << COM0B1);
        }

        // Use I/O clock without prescaling
        TCCR0B = (1 << CS00);

        // Re-enable the timer
        GTCCR &= ~(1 << TSM);
}

void init_input(uint8_t pin, bool toggle_pullup) {
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

        /*
         * MCU Control Register (MCUCR)
         *
         * ┌───────┬───────┬───────┬───────┬───────┬───────┬───────┬───────┐
         * │ 7     │ 6     │ 5     │ 4     │ 3     │ 2     │ 1     │ 0     │
         * ├───────┼───────┼───────┼───────┼───────┼───────┼───────┼───────┤
         * │ BODS  │ PUD   │ SE    │ SM1   │ SM0   │ BODSE │ ISC01 │ ISC00 │
         * └───────┴───────┴───────┴───────┴───────┴───────┴───────┴───────┘
         * PUD: Pull-up disable, ignores pull-up-resistor configuration if set to one.
         */
        MCUCR &= ~(1 << PUD);

        /*
         * Port B Data Register (PORTB)
         * Bits 0 to 5 correspond to pins 0 to 5 and enable the internal pull-up
         * resistor if the pin is configured as output
         */

        if (toggle_pullup) {
                PORTB |= (1 << pin);
        } else {
                PORTB &= ~(1 << pin);
        }
}

void init_output(uint8_t pin) {
        if (pin > 5) {
                return;
        }

        // See init_input for DDRB description
        DDRB |= (1 << pin);
}

uint16_t read_adc(void) {
        unsigned int adc_l = ADCL;
        unsigned int adc_h = ADCH;

        return (adc_h << 8) | adc_l;
}

bool read_pin(uint8_t pin) {
        return (PINB & (1 << pin)) != 0;
}

void write_fast_pwm_duty_cycle(uint8_t pin, uint8_t duty_cycle) {
        if (pin == 0) {
                OCR0A = duty_cycle;
        } else if (pin == 1) {
                OCR0B = duty_cycle;
        }

}

void write_pin(uint8_t pin, bool high) {
        /*
         * Port B Data Register (PORTB)
         * Bits 0 to 5 correspond to pins 0 to 5 and set the pin high or low if
         * the pin is configured as output.
         */
        if (high) {
                PORTB |= (1 << pin);
        } else {
                PORTB &= ~(1 << pin);
        }
}

int main(void) {
        bool led_state = false;

        init_output(0);
        init_adc(2);
        init_output(1);
        init_output(3);
        init_input(4, false);
        init_fast_pwm(true, false);

        write_fast_pwm_duty_cycle(0, 64);

        while (1) {
                if (!read_pin(4)) {
                        led_state = !led_state;
                        write_pin(1, led_state);
                }
                write_pin(3, read_adc() > 512);
                _delay_ms(50);
        }

        return 0;
}
