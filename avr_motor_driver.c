/**
 * ATtiny85 Motor Driver with Potentiometer Control - BEGINNERS TUTORIAL
 * 
 * This program demonstrates how to control a DC motor's speed using 
 * a potentiometer (variable resistor) with an ATtiny85 microcontroller.
 * 
 * ===== CONCEPTS COVERED =====
 * 1. Digital outputs (PWM for motor control)
 * 2. Analog inputs (reading potentiometer position)
 * 3. PWM (Pulse Width Modulation) for variable motor speed
 * 4. ADC (Analog to Digital Conversion)
 * 
 * ===== HARDWARE CONNECTIONS =====
 * - PB1 (physical pin 6): PWM output - Connect to motor driver's enable/PWM input
 *   This pin will output a variable signal to control motor speed
 * 
 * - PB2 (physical pin 7): Analog input - Connect to potentiometer's middle pin (wiper)
 *   The potentiometer's outer pins connect to VCC (+5V) and GND
 *   As you turn the potentiometer, the voltage on this pin changes
 * 
 * - You MUST use a motor driver chip (L293D, TB6612FNG, etc.) between the ATtiny85
 *   and the motor. The ATtiny85 cannot provide enough current to drive a motor directly!
 */

#include <avr/io.h>       // This header includes register definitions for ATtiny85
#include <util/delay.h>   // This header provides the _delay_ms() function

// Define meaningful names for the pins we'll use
// This makes the code more readable than using raw pin numbers
#define PWM_PIN PB1       // PB1 is the pin we'll use for PWM output
#define POT_PIN PB2       // PB2 is connected to ADC1 (analog input 1)

/**
 * Initialize the PWM output on Timer0
 * 
 * WHAT IS PWM?
 * PWM (Pulse Width Modulation) is a technique to create a variable analog-like output
 * using digital signals. It rapidly switches a pin on and off, and by varying the
 * ratio of on-time to off-time (duty cycle), we can control the average power
 * delivered to the motor, thus controlling its speed.
 */
void pwm_init(void) {
    // Set PWM_PIN as an output
    // DDRB is the Data Direction Register for Port B
    // Setting a bit to 1 configures that pin as an output
    DDRB |= (1 << PWM_PIN);   // The '|=' operator means "set this bit, leave others unchanged"
    
    // Configure Timer0 for PWM operation
    // TCCR0A and TCCR0B are Timer/Counter Control Registers for Timer0
    
    // TCCR0A configuration:
    // - COM0B1 bit set: Clear OC0B (PB1) on compare match, set at BOTTOM (non-inverting mode)
    //   This determines how the PWM signal behaves
    // - WGM01 and WGM00 bits set: Fast PWM mode (mode 3)
    //   This determines the PWM counting pattern
    TCCR0A = (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
    
    // TCCR0B configuration:
    // - CS00 bit set: No prescaler (run at full system clock)
    //   This determines the PWM frequency
    TCCR0B = (1 << CS00);
    
    // Set initial PWM duty cycle to 0 (motor stopped)
    // OCR0B is the Output Compare Register for Timer0 Channel B
    // This register holds the value that determines the duty cycle
    OCR0B = 0;  // 0 = 0% duty cycle (motor off), 255 = 100% duty cycle (max speed)
}

/**
 * Initialize the ADC (Analog-to-Digital Converter)
 * 
 * WHAT IS ADC?
 * ADC converts the analog voltage from the potentiometer (which can be any value
 * between 0V and 5V) into a digital number the microcontroller can understand.
 * On the ATtiny85, this digital value ranges from 0 (for 0V) to 1023 (for 5V).
 */
void adc_init(void) {
    // ADMUX (ADC Multiplexer Selection Register) configuration:
    // - REFS0 not set: Use VCC as voltage reference
    // - MUX0 set: Select ADC1 channel (which is connected to PB2)
    ADMUX = (0 << REFS0) | (1 << MUX0);
    
    // ADCSRA (ADC Control and Status Register A) configuration:
    // - ADEN bit set: Enable the ADC
    // - ADPS2 and ADPS1 bits set: Set prescaler to 64
    //   This sets the ADC clock to system clock / 64
    //   ATtiny85 typically runs at 8MHz, so ADC clock = 8MHz/64 = 125kHz
    //   ADC needs a clock between 50kHz and 200kHz for best accuracy
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);
}

/**
 * Read the ADC value from the potentiometer
 * 
 * This function starts an ADC conversion, waits for it to finish,
 * and returns the result (a value between 0 and 1023).
 */
uint16_t read_adc(void) {
    // Start an ADC conversion by setting ADSC (ADC Start Conversion) bit
    ADCSRA |= (1 << ADSC);
    
    // Wait for conversion to complete
    // ADSC bit is automatically cleared when conversion is done
    // The while loop continues as long as ADSC is set (conversion in progress)
    while (ADCSRA & (1 << ADSC));
    
    // Return the ADC value
    // ADC is a 16-bit register that holds the conversion result
    return ADC;
}

/**
 * Set motor speed (0-255)
 * 
 * This function adjusts the PWM duty cycle to control motor speed.
 * - 0 = Motor stopped (0% duty cycle)
 * - 255 = Maximum speed (100% duty cycle)
 */
void motor_control(uint8_t speed) {
    // OCR0B (Output Compare Register 0B) determines PWM duty cycle
    // Setting this register changes the PWM signal's on-time to off-time ratio
    OCR0B = speed;
}

/**
 * Main program
 * 
 * This is where execution begins when the microcontroller powers up.
 * It runs an infinite loop that continuously checks the potentiometer
 * position and adjusts the motor speed accordingly.
 */
int main(void) {
    // Variables to store our readings and calculated values
    uint16_t pot_value;   // Potentiometer reading (0-1023)
    uint8_t motor_speed;  // Motor speed (0-255)
    
    // Initialize the hardware peripherals
    pwm_init();  // Set up PWM for motor control
    adc_init();  // Set up ADC for reading the potentiometer
    
    // Main program loop - runs forever
    while (1) {
        // Step 1: Read potentiometer position (returns 0-1023)
        pot_value = read_adc();
        
        // Step 2: Convert potentiometer value (0-1023) to motor speed (0-255)
        // The right shift operator (>>) divides by 4
        // (1023 ÷ 4 ≈ 255)
        motor_speed = pot_value >> 2;
        
        // Step 3: Update the motor speed
        motor_control(motor_speed);
        
        // Step 4: Small delay to prevent too frequent updates
        // This makes the system more stable and reduces electrical noise
        _delay_ms(10);
        
        // Loop back to step 1 and repeat forever
    }
    
    // This return statement is never reached because of the infinite loop
    // It's included to satisfy the C compiler that expects a return value
    return 0;
}
