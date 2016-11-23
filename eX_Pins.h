#ifndef EX_PINS_H
#define EX_PINS_H

// USED PINS Pro Micro:
// смотри "Arduino Pro Micro - Pinout - Sparkfun.jpg"

// D0/RX     ->  PD2 (RXD1/INT2)                : Serial1
// D1/TX     ->  PD3 (TXD1/INT3)                : Serial1
// D2/SDA    ->  PD1 (SDA/INT1)                 : I2C 
// D3/SCL    ->  PD0 (OC0B/SCL/INT0)            : I2C
// D4        ->  PD4 (ICP1/ADC8)                : SONAR Echo
// D5        ->  PC6 (OC3A/!OC4A)               : DIR  Motor1 & Motor2
// D6        ->  PD7 (T0/OC4D/ADC10)            : ENABLE Motors
// D7        ->  PE6 (INT.6/AIN0)               : STEP Motor1
// D8        ->  PB4 (PCINT4/ADC11)             : STEP Motor2
// D9        ->  PB5 (PCINT5/OC1A/!OC4B/ADC12)  : SONAR Trigger
// D10       ->  PB6 (PCINT6/OC1B/OC4B/ADC13)   : SERVO
// D14/MISO  ->  PB3 (PDO/PCINT3/MISO)          : NO. (SPI)
// D15/SCLK  ->  PB1 (PCINT1/SCLK)              : NO. (SPI)
// D16/MOSI  ->  PB2 (PDI/PCINT2/MOSI)          : NO. (SPI)
// A0        ->  PF7 (ADC7/TDI)                 : Battery monitor
// A1        ->  PF6 (ADC6/TDO)                 : LED RED
// A2        ->  PF5 (ADC5/TMS)                 : LED GREEN
// A3        ->  PF4 (ADC4/TCK)                 : LED BLUE

#define SONAR_ECHO_PIN       4     // D4  -> PD4 (ICP1/ADC8)
#define MOTORS_DIR_PIN       5     // D5  -> PC6 (OC3A/!OC4A)
#define MOTORS_ENABLE_PIN    6     // D6  -> PD7 (T0/OC4D/ADC10)
#define MOTOR1_STEP_PIN      7     // D7  -> PE6 (INT.6/AIN0)
#define MOTOR2_STEP_PIN      8     // D8  -> PB4 (PCINT4/ADC11)
#define SONAR_TRIG_PIN       9     // D9  -> PB5 (PCINT5/OC1A/!OC4B/ADC12)
#define SERVO_PIN            10    // D10 -> PB6 (PCINT6/OC1B/OC4B/ADC13)
#define BATTERY_PIN          A0    // A0  -> PF7 (ADC7/TDI)
#define RED_LED_PIN          A1    // A1  -> PF6 (ADC6/TDO)                
#define GREEN_LED_PIN        A2    // A2  -> PF5 (ADC5/TMS)
#define BLUE_LED_PIN         A3    // A3  -> PF4 (ADC4/TCK) 

#endif //EX_PINS_H
