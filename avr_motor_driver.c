/**
 * ATtiny85 Motor Driver with Potentiometer Control
 *
 * This program controls a DC motor using PWM on an ATtiny85 microcontroller.
 * Motor speed is controlled by a potentiometer connected to ADC input.
 *
 * Hardware connections:
 * - PB1 (pin 6): PWM output (connect to enable/PWM input of motor driver)
 * - PB2 (pin 7): Potentiometer input (connect to wiper of potentiometer)
 *   (Potentiometer outer pins should connect to VCC and GND)
 *
 * Motor driver chip (e.g., L293D, TB6612FNG, etc.) should be connected between
 * the ATtiny85 and the motor for proper current handling.
 */

#include <avr/io.h>
#include <util/delay.h>

// Define pins
#define PWM_PIN PB1
#define POT_PIN PB2       // ADC1 (physical pin 7)

/**
 * Initialize the PWM output on Timer0
 * ATtiny85 Timer0 can generate PWM on PB1 (OC0B)
 */
void pwm_init(void) {
    // Set PWM pin as output
    DDRB |= (1 << PWM_PIN);

    // Set up Timer0 for PWM operation:
    // - Fast PWM mode
    // - Clear OC0B on compare match, set at BOTTOM (non-inverting mode)
    // - No prescaler (maximum PWM frequency)
    TCCR0A = (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
    TCCR0B = (1 << CS00);  // No prescaler

    // Set initial PWM duty cycle to 0 (motor stopped)
    OCR0B = 0;
}

/**
 * Initialize the ADC for potentiometer reading
 */
void adc_init(void) {
    // Set reference voltage to VCC, select ADC1 (PB2)
    ADMUX = (0 << REFS0) | (1 << MUX0);

    // Enable ADC, set prescaler to 64 for 125kHz ADC clock
    // (ATtiny85 typically runs at 8MHz internal, so 8MHz/64 = 125kHz)
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);
}

/**
 * Read the ADC value from the potentiometer
 */
uint16_t read_adc(void) {
    // Start conversion
    ADCSRA |= (1 << ADSC);

    // Wait for conversion to complete
    while (ADCSRA & (1 << ADSC));

    // Return ADC value
    return ADC;
}

/**
 * Set motor speed (0-255)
 */
void motor_control(uint8_t speed) {
    // Set speed (PWM duty cycle)
    OCR0B = speed;
}

int main(void) {
    uint16_t pot_value;
    uint8_t motor_speed;

    // Initialize PWM and ADC
    pwm_init();
    adc_init();

    while (1) {
        // Read potentiometer value (0-1023)
        pot_value = read_adc();

        // Convert to motor speed (0-255)
        motor_speed = pot_value >> 2;

        // Set motor speed
        motor_control(motor_speed);

        // Small delay to prevent too frequent updates
        _delay_ms(10);
    }

    return 0;  // Never reached
}
