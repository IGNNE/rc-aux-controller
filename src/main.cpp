#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#define F_CPU 20000000
#include <util/delay.h>

/**
 * @file ESC and servo controller based on pololu BabyOrangutan 328
 * 
 * Hardware setup: TODO
 * 
 * TODO:
 * - servos
 * - i2c slave
 * - analog battery voltage
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
// servo pins need arduino numbers
#define SERVO_1_PIN PD7
#define SERVO_2_PIN 4 //PD4
#define SERVO_3_PIN PD2
Servo servo1;

#define I2C_ADDRESS 4
#define I2C_NO_REQUEST 0xFF
// return values
#define I2C_OK 1
#define I2C_NOK 0
// commands
#define I2C_CMD_BATTERY 1
#define I2C_CMD_SERVO1 2
#define I2C_CMD_SERVO2 3
#define I2C_CMD_SERVO3 4
#define I2C_CMD_ESC 5
/**
 * I2C should communicate as follows:
 * 1. Master sends one byte command
 * 2.a Slave responds with a '1' for OK, '0' for not OK
 * 2.b Slave sends the requested info byte
 */
volatile uint8_t request_byte = I2C_NO_REQUEST;

/**
 * Sleep in increments of 10 ms, because _delay_ms can sleep at max 13 ms
 */
void long_delay(uint8_t ten_ms) {
  for(uint8_t i = 0; i <= ten_ms; i++) {
    _delay_ms(10);
  }
}

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

void i2c_on_request() {
  switch (request_byte)
  {
    case I2C_CMD_BATTERY:
      Wire.write(get_battery_voltage());
      break;

    case I2C_NO_REQUEST:
      // something went wrong, answer nok
      Wire.write(I2C_NOK);
      break;
    default:
      // command already done, just answer ok
      Wire.write(I2C_OK);
      break;
  } 
  // toggle led to show that something is working
  PORTD ^= _BV(ULED);
}

void i2c_on_receive(int num_of_bytes) {
  // TODO: check 3-byte servo commands
  if(num_of_bytes == 3) {

    // 3 byte set command, for servo timings
    request_byte = Wire.read();
    uint16_t payload = Wire.read() + (Wire.read() << 16);
    switch (request_byte)
    {
      case I2C_CMD_SERVO1:
        servo1.writeMicroseconds(payload);
        break;
      case I2C_CMD_ESC:
        M1_forward(payload/4096);
        break;
      default:
        // command not understood
        request_byte = I2C_NO_REQUEST;
        break;
    }

  } else if(num_of_bytes == 1) {

    // 1 byte get command, pass on to request callback
    request_byte = Wire.read();

  } else {
    // something went wrong
    // empty buffer
    while(Wire.available()) { Wire.read(); }
    request_byte = I2C_NO_REQUEST;
  }
  // toggle led to show that something is working
  PORTD ^= _BV(ULED);
}


// Motor Initialization routine -- this function must be called
//  before you use any of the above functions
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
  servo1.attach(SERVO_1_PIN);

  // setup i2c slave
  Wire.begin(I2C_ADDRESS);
  // just to be safe 
  digitalWrite(SDA, 0);
  digitalWrite(SCL, 0);
  Wire.onRequest(i2c_on_request);
  Wire.onReceive(i2c_on_receive);

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
    long_delay(50);
  }

  // test-run motor
  M1_forward(64);
  _delay_ms(10);
  M1_forward(0);
}

void loop() {
  

}