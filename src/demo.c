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

#include <assert.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

/**
 * @brief Used by `init_timer1()`, do not set this directly!
 *
 * This variable is used by `init_timer1()` to store the pointer to the function
 * which will be executed periodically.
 */
void (*timer1_callback)(void) = NULL;

/**
 * @brief Used by `init_timer1()`, do not set this directly!
 *
 * The callback for timer/counter 1 may be executed with a frequency that is not
 * an integer divisor of `F_CPU`. In that case timer/counter 1 may count to two
 * different periods - a lower one and a higher one - to match the given
 * frequency over a longer time.
 *
 * A cycle is defined as the number of times from the timer starting to count
 * to the lower period until the timer starts the next time counting to the
 * lower period.
 *
 * For example if the timer first counts `10` times to the lower period and then
 * `5` times to the higher period, before it counts to the lower period again,
 * the cycle is `15`.
 *
 * This variable holds the index in the cycle.
 */
uint16_t timer1_cycle_counter = 0;

/**
 * @brief Used by `init_timer1()`, do not set this directly!
 *
 * This is the length of the cycle.
 */
uint16_t timer1_cycle_length = 0;

/**
 * @brief Used by `init_timer1()`, do not set this directly!
 *
 * This is the low period, that the counter counts to in its cycle.
 */
uint8_t timer1_period_low = 0;

/**
 * @brief Used by `init_timer1()`, do not set this directly!
 *
 * This is how often the counter will count to the low period in its cycle until
 * it starts counting to the high period.
 */
uint16_t timer1_ticks_low = 0;

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
         * Timer/Counter 0 Control Register A (TCCR0A)
         *
         * ┌────────┬────────┬────────┬────────┬─────┬─────┬───────┬───────┐
         * │ 7      │ 6      │ 5      │ 4      │ 3   │ 2   │ 1     │ 0     │
         * ├────────┼────────┼────────┼────────┼─────┼─────┼───────┼───────┤
         * │ COM0A1 │ COM0A0 │ COM0B1 │ COM0B0 │     │     │ WGM01 │ WGM00 │
         * └────────┴────────┴────────┴────────┴─────┴─────┴───────┴───────┘
         *
         * Timer/Counter 0 Control Register B (TCCR0B)
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

/**
 * @brief Set up periodic execution of a function using timer 1.
 *
 * Set up Timer/Counter 1 of the ATTiny85 to execute the callback at the given
 * frequency. The frequency is limited by the CPU clock speed. At least 255 CPU
 * cycles should pass between each call of the callback. Otherwise the execution
 * will be aborted.
 * The prescaler and output compare registers will be set to match the given
 * frequency as close as possible, sometimes even with different values for the
 * output compare registers between calls to the callback to match the given
 * frequency over a broader period.
 *
 * For example with a CPU clock speed of `1 MHz` and a desired frequency of
 * `2300 Hz`, the prescaler is set to `2` in order the increase the
 * timer/counter with a frequency of `500 KHz`. Furthermore the output compare
 * register is set to `217` for `1400` ticks and `218` for `900` ticks each,
 * because
 * \f[
 *     \frac{\nicefrac{1 \text{MHz}}{2}}{217} \cdot \frac{1400}{2300}
 *     + \frac{\nicefrac{1 \text{MHz}}{2}}{218} \cdot \frac{900}{2300}
 *     \approx 2300 \text{Hz}
 * \f]
 *
 * @param[in] f_hz     is the frequency of the periodic calls to the function in
 *                     Hertz.
 * @param[in] callback is the function to be called periodically.
 */
void init_timer1(uint16_t f_hz, void (*callback)(void)) {
        // Ensure atleast 255 CPU cycles between timer interrupts
        assert(F_CPU / f_hz > 0xff);

        // Disable interrupts
        cli();

        /*
         * Timer/Counter 1 Control Register (TCCR1)
         *
         * ┌──────┬───────┬────────┬────────┬──────┬──────┬──────┬──────┐
         * │ 7    │ 6     │ 5      │ 4      │ 3    │ 2    │ 1    │ 0    │
         * ├──────┼───────┼────────┼────────┼──────┼──────┼──────┼──────┤
         * │ CTC1 │ PWM1A │ COM1A1 │ COM1A0 │ CS13 │ CS12 │ CS11 │ CS10 │
         * └──────┴───────┴────────┴────────┴──────┴──────┴──────┴──────┘
         * CTC1: Clear Timer/Counter on Compare Match. If set, timer is reset to
         *       0 after a match against the OCR1C register. Otherwise timer just
         *       continues.
         * CS13, CS12, CS11, CS10: Clock Select Bits. Important values:
         *                         ┌──────┬──────┬──────┬──────┬───────────────┐
         *                         │ CS13 │ CS12 │ CS11 │ CS10 │               │
         *                         ├──────┼──────┼──────┼──────┼───────────────┤
         *                         │ 0    │ 0    │ 0    │ 0    │ Timer stopped │
         *                         ├──────┼──────┼──────┼──────┼───────────────┤
         *                         │ 0    │ 0    │ 0    │ 1    │ No prescaling │
         *                         ├──────┼──────┼──────┼──────┼───────────────┤
         *                         │ 0    │ 0    │ 1    │ 0    │ F_CPU/2       │
         *                         ├──────┼──────┼──────┼──────┼───────────────┤
         *                         │ 0    │ 0    │ 1    │ 1    │ F_CPU/4       │
         *                         ├──────┼──────┼──────┼──────┼───────────────┤
         *                         │ ⋮    │ ⋮    │ ⋮    │ ⋮    │ ⋮             │
         *                         ├──────┼──────┼──────┼──────┼───────────────┤
         *                         │ 1    │ 1    │ 1    │ 1    │ F_CPU/16384   │
         *                         └──────┴──────┴──────┴──────┴───────────────┘
         */
        TCCR1 = (1 << CTC1);

        /*
         * Find an optimal configuration for the prescaler and OCR1A/OCR1C
         * registers to run the timer with the frequency given in the `f_hz`
         * parameter.
         */

        uint8_t prescaler_mask = 0x01;
        uint16_t prescaler = 1;

        // Use 254 here. If 255 is used here, the high period would overflow.
        while (F_CPU / (prescaler * f_hz) > 254) {
                prescaler *= 2;
                prescaler_mask += 1;
        }

        timer1_callback = callback;
        timer1_cycle_counter = 0;
        timer1_cycle_length = f_hz;
        timer1_period_low = F_CPU / (prescaler * f_hz);
        timer1_ticks_low = (F_CPU / prescaler) % f_hz;

        // Initialize the Timer/Counter 1 Output compare register A (OCR1A) for
        // the compare match A interrupt to the low period.
        OCR1A = timer1_period_low;
        // Set the Timer/Counter 1 Output compare register C (OCR1C) to the same
        // value as OCR1A to reset the Timer/Counter 1 after a compare match.
        // Note, that this only works with the CTC1 bit set in TCCR1 above.
        OCR1C = timer1_period_low;

        // Clear CS1[3:0]
        TCCR1 &= 0xf0;
        TCCR1 |= prescaler_mask;

        /*
         * Timer/Counter Interrupt Mask Register (TIMSK)
         * ┌───┬────────┬────────┬────────┬────────┬───────┬───────┬───┐
         * │ 7 │ 6      │ 5      │ 4      │ 3      │ 2     │ 1     │ 0 │
         * ├───┼────────┼────────┼────────┼────────┼───────┼───────┼───┤
         * │   │ OCIE1A │ OCIE1B │ OCIE0A │ OCIE0B │ TOIE1 │ TOIE0 │   │
         * └───┴────────┴────────┴────────┴────────┴───────┴───────┴───┘
         *
         * OCIE1A: Timer/Counter 1 Output Compare Interrupt Enable A
         * OCIE1B: Timer/Counter 1 Output Compare Interrupt Enable B
         * TOIE1: Timer/Counter 1 Overflow Interrupt Enable
         */

        // Enable the interrupt on OCR1A compare match.
        TIMSK |= (1 << OCIE1A);

        // Re-enable interrupts
        sei();
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

void square_wave(void) {
        static bool wave_active = false;

        write_pin(1, wave_active = !wave_active);
}

int main(void) {
        uint16_t ticks_since_last_btn_press = 0;
        size_t pin1_f_index = 1;
        uint16_t pin1_f[] = { 2, 10, 20, 50, 100, 2000 };

        // Setup all pins
        init_output(0);
        init_output(1);
        init_adc(2);
        init_output(3);
        init_input(4, false);

        // Setup Fast PWM on pin 0
        init_fast_pwm(true, false);

        write_fast_pwm_duty_cycle(0, 64);

        // Setup timer/counter 1 for a 1 Hz square wave on pin 1.
        init_timer1(2, &square_wave);

        while (true) {
                // Poor mans debouncing. Wait at least 500ms before registering
                // the next button press.
                if (!read_pin(4) && ticks_since_last_btn_press > 10) {
                        ticks_since_last_btn_press = 0;
                        if (pin1_f_index >= 5) {
                                pin1_f_index = 0;
                        } else {
                                pin1_f_index++;
                        }
                        init_timer1(pin1_f[pin1_f_index], &square_wave);
                }
                write_pin(3, read_adc() > 512);
                _delay_ms(50);
                ticks_since_last_btn_press++;
        }

        return 0;
}

/**
 * @brief Service routine for Timer/Counter 1 Compare Match A interrupt.
 *
 * Changes the output compare registers between the low and high periods to
 * match the desired frequency as close as possible.
 * Also calls @ref timer1_callback if not `NULL`.
 */
ISR(TIMER1_COMPA_vect) {
        if (NULL == timer1_callback) {
                // Without a callback, the whole logic to match the frequency is
                // not needed.
                return;
        }

        if (timer1_cycle_counter == timer1_ticks_low) {
                // This is the high period -> one higher than the low period.
                OCR1A = timer1_period_low + 1;
                OCR1C = timer1_period_low + 1;
        }

        timer1_cycle_counter++;

        if (timer1_cycle_counter >= timer1_cycle_length) {
                // Reset again to the low period.
                OCR1A = timer1_period_low;
                OCR1C = timer1_period_low;
                timer1_cycle_counter = 0;
        }

        (*timer1_callback)();
}
