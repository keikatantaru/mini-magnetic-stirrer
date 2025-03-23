/**
 * ATtiny85 Motor Driver with Potentiometer Control
 * 
 * This program controls a DC motor using PWM on an ATtiny85 microcontroller.
 * Motor speed is controlled by a potentiometer connected to ADC input.
 * 
 * Hardware connections:
 * - PB0 (pin 5): Direction control (connect to direction input of motor driver)
 * - PB1 (pin 6): PWM output (connect to enable/PWM input of motor driver)
 * - PB2 (pin 7): Potentiometer input (connect to wiper of potentiometer)
 *   (Potentiometer outer pins should connect to VCC and GND)
 * - PB3 (pin 2): Direction toggle switch (with pull-up resistor)
 * 
 * Motor driver chip (e.g., L293D, TB6612FNG, etc.) should be connected between
 * the ATtiny85 and the motor for proper current handling.
 */

#include <avr/io.h>
#include <util/delay.h>

// Define pins
#define DIRECTION_PIN PB0
#define PWM_PIN PB1
#define POT_PIN PB2       // ADC1 (physical pin 7)
#define DIR_SWITCH_PIN PB3 // Direction toggle switch

// Define motor directions
#define FORWARD 1
#define REVERSE 0

/**
 * Initialize the PWM output on Timer0
 * ATtiny85 Timer0 can generate PWM on PB1 (OC0B)
 */
void pwm_init(void) {
    // Set PWM and direction pins as output
    DDRB |= (1 << PWM_PIN) | (1 << DIRECTION_PIN);
    
    // Set direction switch pin as input with pull-up
    DDRB &= ~(1 << DIR_SWITCH_PIN);
    PORTB |= (1 << DIR_SWITCH_PIN);
    
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
 * Set motor speed (0-255) and direction
 */
void motor_control(uint8_t speed, uint8_t direction) {
    // Set direction pin
    if (direction == FORWARD) {
        PORTB |= (1 << DIRECTION_PIN);  // Set high for forward
    } else {
        PORTB &= ~(1 << DIRECTION_PIN); // Set low for reverse
    }
    
    // Set speed (PWM duty cycle)
    OCR0B = speed;
}

/**
 * Read direction switch status
 */
uint8_t read_direction(void) {
    // Return FORWARD if switch is pressed (low), REVERSE if not
    return (PINB & (1 << DIR_SWITCH_PIN)) ? REVERSE : FORWARD;
}

int main(void) {
    uint16_t pot_value;
    uint8_t motor_speed;
    uint8_t direction;
    
    // Initialize PWM and ADC
    pwm_init();
    adc_init();
    
    while (1) {
        // Read potentiometer value (0-1023)
        pot_value = read_adc();
        
        // Convert to motor speed (0-255)
        motor_speed = pot_value >> 2;
        
        // Read direction switch
        direction = read_direction();
        
        // Set motor speed and direction
        motor_control(motor_speed, direction);
        
        // Small delay to prevent too frequent updates
        _delay_ms(10);
    }
    
    return 0;  // Never reached
}
