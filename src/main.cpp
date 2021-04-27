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
 * - RasPi power control
 * - fix servo stutter by blocking reads while setting servos
 */

// If defined, use internal PWM for motor control
// #define MOTOR_PWM



#define ULED PD1

// servo pins need arduino numbers - finish this
#define SERVO_1_PIN 7 // PD7
#define SERVO_2_PIN 4 // PD4
#define SERVO_3_PIN 2 // PD2
#define SERVO_4_PIN 0 // PD0
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;


#ifdef MOTOR_PWM
#define MOTOR_1_A PD5
#define MOTOR_1_B PD6
#define MOTOR_2_A PD3
#define MOTOR_2_B PB3
#endif // MOTOR_PWM

#define I2C_ADDRESS 0x04

// return values
#define I2C_OK 1
#define I2C_NOK 0
// commands/register addresses
#define I2C_CHIP_ID 0     //< chip id (i2c address), sets will be ignored
#define I2C_CMD_BATTERY 1 //< battery voltage: 255=5V, 0=0V, sets will be ignored
#define I2C_CMD_SERVO1 2  //< set/get servo
#define I2C_CMD_SERVO2 4  //< set/get servo
#define I2C_CMD_SERVO3 6  //< set/get servo
#define I2C_CMD_ESC 8     //< set/get motor controller


// About 3.2V
#define BATTERY_MIN_VALUE 165

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


#ifdef MOTOR_PWM

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

// don't use any more, both motor channels are shorted together in hardware
// and driven by M1 only
// void M2_forward(unsigned char pwm)
// {
//     OCR2A = 0;
//     OCR2B = pwm;
// }

// void M2_reverse(unsigned char pwm)
// {
//     OCR2B = 0;
//     OCR2A = pwm;
// }

// Motor Initialization routine -- this function must be called
// before you use any of the functions above
void motors_init()
{
    // configure for inverted PWM output on motor control pins:
    //  set OCxx on compare match, clear on timer overflow
    //  Timer0 and Timer2 count up from 0 to 255
    TCCR0A = TCCR2A = 0xF3;

    // use the system clock/64 (=312,5 kHz) as the timer clock
    TCCR0B = TCCR2B = 0x03;

    // initialize all PWMs to 0% duty cycle (braking)
    OCR0A = OCR0B = OCR2A = OCR2B = 0;

    // set PWM pins as digital outputs (the PWM signals will not
    // appear on the lines if they are digital inputs)
    // keep in mind that only M1 is used, M2 (B3/D3) has to stay input / high-Z
    DDRD = (1 << PORTD5) | (1 << PORTD6);
}

#endif // MOTOR_PWM


void setup() {

#ifdef MOTOR_PWM
    // setup motors
    motors_init();
#endif // MOTOR_PWM

    // just to be safe, make sure we don't output 5V to the RasPi
    digitalWrite(SDA, 0);
    digitalWrite(SCL, 0);

    // setup i2c slave
    init_i2c_slave(I2C_ADDRESS);
    force_set_i2cdata(I2C_CHIP_ID, I2C_ADDRESS);

    // setup servos
    servo1.attach(SERVO_1_PIN);
    servo2.attach(SERVO_2_PIN);
    servo3.attach(SERVO_3_PIN);
    servo4.attach(SERVO_4_PIN);

    // set all servo values to the middle position of 1500 (=0x05DC)
    try_set_i2cdata(I2C_CMD_SERVO1, 0x05);
    try_set_i2cdata(I2C_CMD_SERVO1+1, 0xDC);
    try_set_i2cdata(I2C_CMD_SERVO2, 0x05);
    try_set_i2cdata(I2C_CMD_SERVO2+1, 0xDC);
    try_set_i2cdata(I2C_CMD_SERVO3, 0x05);
    try_set_i2cdata(I2C_CMD_SERVO3+1, 0xDC);
#ifndef MOTOR_PWM
    // initialize the motor as fast as possible to arm the esc
    servo4.writeMicroseconds(1000);
    // if the motor is driven by an external speed controller, initialize like a servo
    // but set starting throttle to 1000 / 0%
    try_set_i2cdata(I2C_CMD_ESC, 0x03);
    try_set_i2cdata(I2C_CMD_ESC+1, 0xE8);
#endif // MOTOR_PWM


    // setup battery adc
    // free running, no ints, max. prescaler
    ADCSRA = 0b10100111;
    // channel 1, external reference, 8 bit only
    ADMUX = 0b00100001;
    // start adc
    ADCSRA |= _BV(ADSC);

    // setup led and blink to tell that we are ready
    DDRD |= _BV(ULED);
    for(uint8_t i = 0; i < 5; i++) {
        PORTD ^= _BV(ULED);
        long_delay(100);
    }

#ifdef MOTOR_PWM
    // test-run motor
    M1_forward(10);
    long_delay(1);
    M1_forward(0);
    long_delay(50);
    M1_forward(10);
    long_delay(1);
    M1_forward(0);
#endif // MOTOR_PWM

}

void loop() {

    // "tick counter" for doing tasks every few hundred ms, roughly estimated
    // the servo functions are the largest part in this loop, so without them this runs roughly 10x faster
    // TODO: runs a bit slow now with all the servo stuff, maybe use a timer instead?
    static uint16_t loops_since_last_tick = 0;
    const uint32_t MAX_SINCE_LAST_TICK = F_CPU / 500;

    // only trip the battery warning after two low readings in a row
    static bool battery_warning_armed = false;
    static bool battery_warning_tripped = false;

    if(loops_since_last_tick > MAX_SINCE_LAST_TICK) {
        loops_since_last_tick = 0;

        // vvv tick counter has run out, run tasks here vvv

        // refresh adc value from time to time
        uint8_t battery_voltage = get_battery_voltage();
        force_set_i2cdata(I2C_CMD_BATTERY, battery_voltage);


        // battery warning logic
        if(battery_voltage < BATTERY_MIN_VALUE) {
            if(!battery_warning_armed) {
                // first low battery reading, arm warning
                battery_warning_armed = true;
            } else {
                // battery warning armed, now run battery alert
                battery_warning_tripped = true;
            }
        } else {
            // battery is above threshold, dis-arm warning
            battery_warning_armed = false;
        }
        // if battery warning is tripped, blink led
        if(battery_warning_tripped) {
            PORTD ^= _BV(ULED);
#ifdef MOTOR_PWM
            // stop motors to prevent brownout
            M1_forward(0);
#endif // MOTOR_PWM
            // TODO: do something else here, maybe add a buzzer?
        }
    }

    // increase tick counter
    loops_since_last_tick++;

    // refresh motor/servo values on every loop for maximum responsiveness
    // TODO: lock i2c during these calls
    servo1.writeMicroseconds((get_i2cdata(I2C_CMD_SERVO1)<<8) + get_i2cdata(I2C_CMD_SERVO1+1));
    servo2.writeMicroseconds((get_i2cdata(I2C_CMD_SERVO2)<<8) + get_i2cdata(I2C_CMD_SERVO2+1));
    servo3.writeMicroseconds((get_i2cdata(I2C_CMD_SERVO3)<<8) + get_i2cdata(I2C_CMD_SERVO3+1));

    // only uptdate motor if there is enough battery voltage left
    if(!battery_warning_tripped) {
#ifdef MOTOR_PWM
        M1_forward(get_i2cdata(I2C_CMD_ESC));
#else
        servo4.writeMicroseconds((get_i2cdata(I2C_CMD_ESC)<<8) + get_i2cdata(I2C_CMD_ESC+1));
#endif // MOTOR_PWM
    }
}
