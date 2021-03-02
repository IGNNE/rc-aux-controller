#include <Arduino.h>
#include <Servo.h>
#define F_CPU 20000000
#include <util/delay.h>
#include <i2cslave.h>

/**
 * @file ESC and servo controller based on pololu BabyOrangutan 328
 *
 * Hardware setup: TODO
 *
 * TODO:
 * - servos
 * - RasPi power control
 * - some sort of ui?
 * - check if the inerrupts / cycles are not too much
 */

#define MOTOR_1_A PD5
#define MOTOR_1_B PD6
#define MOTOR_2_A PD3
#define MOTOR_2_B PB3

#define ULED PD1

#define NUM_OF_SERVOS 1
// servo pins need arduino numbers - finish this 
#define SERVO_1_PIN PD7
#define SERVO_2_PIN 4 //PD4
#define SERVO_3_PIN PD2
Servo servo1;

#define I2C_ADDRESS 0x04

// return values
#define I2C_OK 1
#define I2C_NOK 0
// commands/register addresses
#define I2C_CHIP_ID 0     //< chip id (i2c address), gets will be ignored
#define I2C_CMD_BATTERY 1 //< battery voltage: 255=5V, 0=0V, gets will be ignored
#define I2C_CMD_SERVO1 2  //< set/get servo
#define I2C_CMD_SERVO2 4  //< set/get servo
#define I2C_CMD_SERVO3 6  //< set/get servo
#define I2C_CMD_ESC 8     //< set/get motor controller


/**
 * Sleep in increments of 10 ms, because _delay_ms can sleep at max 13 ms
 */
void long_delay(uint8_t ten_ms) {
    for(uint8_t i = 0; i <= ten_ms; i++) {
        _delay_ms(10);
    }
}

/**
 * Battery voltage in <value>/256*5 V
 */
uint8_t get_battery_voltage() {
    return uint8_t(ADCH);
}

void M1_forward(unsigned char pwm)
{
    OCR0A = 0;
    OCR0B = pwm;
}

void M1_reverse(unsigned char pwm)
{
    OCR0B = 0;
    OCR0A = pwm;
}

void M2_forward(unsigned char pwm)
{
    OCR2A = 0;
    OCR2B = pwm;
}

void M2_reverse(unsigned char pwm)
{
    OCR2B = 0;
    OCR2A = pwm;
}


// Motor Initialization routine -- this function must be called
// before you use any of the functions above
void motors_init()
{
    // configure for inverted PWM output on motor control pins:
    //  set OCxx on compare match, clear on timer overflow
    //  Timer0 and Timer2 count up from 0 to 255
    TCCR0A = TCCR2A = 0xF3;

    // use the system clock/8 (=2.5 MHz) as the timer clock
    TCCR0B = TCCR2B = 0x02;

    // initialize all PWMs to 0% duty cycle (braking)
    OCR0A = OCR0B = OCR2A = OCR2B = 0;

    // set PWM pins as digital outputs (the PWM signals will not
    // appear on the lines if they are digital inputs)
    DDRD |= (1 << PORTD3) | (1 << PORTD5) | (1 << PORTD6);
    DDRB |= (1 << PORTB3);
}

void setup() {

    // setup motors
    motors_init();

    // setup servos
    // TODO: servos, in general
    servo1.attach(SERVO_1_PIN);


    // just to be safe, make sure we don't output 5V to the RasPi
    digitalWrite(SDA, 0);
    digitalWrite(SCL, 0);

    // setup i2c slave
    init_i2c_slave(I2C_ADDRESS);
    force_set_i2cdata(I2C_CHIP_ID, I2C_ADDRESS);


    // setup battery adc
    // free running, no ints, max. prescaler
    ADCSRA = 0b10100111;
    // channel 1, external reference, 8 bit only
    ADMUX = 0b00100001;
    // start ad
    ADCSRA |= _BV(ADSC);

    // setup led and blink to tell that we are ready
    DDRD |= _BV(ULED);
    for(uint8_t i = 0; i < 5; i++) {
        PORTD ^= _BV(ULED);
        long_delay(100);
    }

    // test-run motor
    M1_forward(50);
    long_delay(1);
    M1_forward(0);
    long_delay(20);
    M1_forward(50);
    long_delay(1);
    M1_forward(0);
}



void loop() {
    // refresh adc value from time to time
    force_set_i2cdata(I2C_CMD_BATTERY, get_battery_voltage());
    PORTD ^= _BV(ULED);
    long_delay(50);


}
