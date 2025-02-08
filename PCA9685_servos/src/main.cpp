// 2022 Paulo Costa, based on the servo example from the Adafruit_PWMServoDriver library
// PCA9685 Servo driver and serial commands example

#include <Arduino.h>

#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_TCS34725.h>
#include <SPI.h>
#include <Wire.h>
#include <Arduino.h>


unsigned long interval, last_cycle;
unsigned long loop_micros;

#include "commands.h"

commands_t serial_commands;
int show_lux;

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver ServoDriver = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver ServoDriver = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver ServoDriver = Adafruit_PWMServoDriver(0x40, Wire);

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_1X);

#define NUM_SERVOS 4

int servo_pos[NUM_SERVOS];
int servo_idx;
float servo_angle[NUM_SERVOS];

int inicial_0 = 300;
int inicial_1 = 400;
int inicial_2 = 150;
int inicial_3 = 250;

int object_0 = 400;
int object_1 = 500;

int sensor_0 = 250;
int sensor_1 = 325;
int sensor_2 = 500;

int blue_0 = 400;
int blue_1 = 400;

int red_0 = 350;
int red_1 = 400;

int green_0 = 300;
int green_1 = 350;

int noColor_0 = 100;
int noColor_1 = 250;

int open = 350;
int closed = 400;

int top = 150;
int down = 500;


// Function to set initial servo positions
void setInitialServoPositions() {
  Serial.println("Setting initial servo positions");
  ServoDriver.setPWM(0, 0, inicial_0);
  delay(1000);
  ServoDriver.setPWM(1, 0, inicial_1);
  delay(1000);
  ServoDriver.setPWM(2, 0, top);
  delay(1000);
  ServoDriver.setPWM(3, 0, open);
  delay(1000);
}

void moveSuave(int num, int initial, int final, int cadence) {
  int i;
  if (initial > final) {
    for (i = initial/5; i > final/5 - 1; i--) {
      ServoDriver.setPWM(num, 0, i * cadence);
      delay(100);
    }
  } else {
    for (i = initial/5; i < final/5 + 1; i++) {
      ServoDriver.setPWM(num, 0, i * cadence);
      delay(100);
    }
  }
}

void moveToObject(){
  Serial.print("Moving to Object");
  moveSuave(0, inicial_0, object_0, 5);
  moveSuave(1, inicial_1, object_1, 5);
}

void pickUp(){
  Serial.print("Picking up");
  for (int i = 30; i < 101; i++) {
    ServoDriver.setPWM(2,0,i*5);
    delay(100);
  }
  ServoDriver.setPWM(3,0,450);
  delay(100); // NÃO TIRAR POR NADA
  for (int i = 100; i > 29; i--) {
    ServoDriver.setPWM(2,0,i*5);
    delay(100);
  }
}

void moveToSensor(){
  Serial.print("Moving to sensor");
  moveSuave(0, object_0, sensor_0, 5);
  delay(1000);
  moveSuave(1, object_1, sensor_1, 5);
  moveSuave(2, top, down, 10);
}

void dropObject(){
  Serial.print("Dropping Object");
  moveSuave(2, top, down, 10);
  ServoDriver.setPWM(3,0,open);  
  moveSuave(2, down, top, 10);
}

void blueObject(){
  Serial.print("Blue Object");
  ServoDriver.setPWM(2,0,top);
  delay(1000);
  moveSuave(0, sensor_0, blue_0, 5);
  moveSuave(1, sensor_1, blue_1, 5);
  dropObject();
}

void redObject(){
  Serial.print("Red Object");
  ServoDriver.setPWM(2,0,top);
  delay(1000);
  moveSuave(0, sensor_0, red_0, 5);
  moveSuave(1, sensor_1, red_1, 5);
  dropObject();
}

void greenObject(){
  Serial.print("Green Object");
  ServoDriver.setPWM(2,0,top);
  delay(1000);
  moveSuave(0, sensor_0, green_0, 5);
  moveSuave(1, sensor_1, green_1, 5);
  dropObject();
}

void noColorObject(){
  Serial.print("No Color Object");
  ServoDriver.setPWM(2,0,top);
  delay(1000);
  moveSuave(0, sensor_0, noColor_0, 5);
  moveSuave(1, sensor_1, noColor_1, 5);
  dropObject();
}

void getRawData_noDelay(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
  *c = tcs.read16(TCS34725_CDATAL);
  *r = tcs.read16(TCS34725_RDATAL);
  *g = tcs.read16(TCS34725_GDATAL);
  *b = tcs.read16(TCS34725_BDATAL);
}

void readColor() {

  ServoDriver.setPWM(2, 0, 500);
    Serial.println("Reading color...");
    
    uint16_t r, g, b, c;
    float redNorm, greenNorm, blueNorm; // Normalized RGB values

    tcs.getRawData(&r, &g, &b, &c);

    // Normalize the RGB values
    redNorm = (float)r / c * 255.0;
    greenNorm = (float)g / c * 255.0;
    blueNorm = (float)b / c * 255.0;

    // Print the normalized RGB values
    Serial.print("Normalized R: "); Serial.print(redNorm);
    Serial.print(", G: "); Serial.print(greenNorm);
    Serial.print(", B: "); Serial.println(blueNorm);

    // Determine the dominant color
    if (redNorm > greenNorm && redNorm > blueNorm) {
        Serial.println("Detected color: Red");
        redObject();
    } else if (greenNorm > redNorm && greenNorm > blueNorm) {
        Serial.println("Detected color: Green");
        greenObject();
    } else if (blueNorm > redNorm && blueNorm > greenNorm) {
        Serial.println("Detected color: Blue");
        blueObject();
    } else {
        Serial.println("Detected color: Unknown");
        noColorObject();
    }
}


void process_command(char command, float value)
{
  if (command == 's') {  // The 's' command Selects the current Servo
    
    servo_idx = value;
    if (servo_idx < 0) servo_idx = 0;
    if (servo_idx > NUM_SERVOS - 1) servo_idx = NUM_SERVOS - 1;

  } else if (command == 'p') { // The 'p' command sets current Servo pulse
                               // Ton = value/4096 * 20 ms
    if (value < 0) value = 0;
    if (value > 4095) value = 4095;  
    servo_pos[servo_idx] = value;  

  } else if (command == 'm') {  // 'm' command triggers moveServo()
    moveToObject();
    Serial.println("Executed moveServo()");

  } else if (command == 'u') {  // 'u' command triggers pickUp()
    pickUp();
    Serial.println("Executed pickUp()");

  } else if (command == 'r') {
    readColor();
    Serial.println("Executed readColor()"); 

  }
}

int find_ServoDriver(int addr)
{

  // test PCA9685 presence
  // write8(PCA9685_MODE1, MODE1_RESTART);

  Wire.beginTransmission(addr);
  Wire.write(PCA9685_MODE1);
  Wire.write(MODE1_RESTART);

  int err = Wire.endTransmission();

  return !err;
  
}

void setup() 
{
  serial_commands.init(process_command);

  // Start the serial port with 115200 baudrate
  Serial.begin(115200);
                    
                   // Connect PCA9685 Vcc to 3.3
  Wire.setSDA(8);  // Connect PCA9685 SDA to gpio 8
  Wire.setSCL(9);  // Connect PCA9685 SCL to gpio 9
  Wire.begin();
  
  /*Wire.begin();

  while (!find_ServoDriver(PCA9685_I2C_ADDRESS)) {
    Serial.println("No PCA9685 found ... check your connections");
    delay(200);
  }*/
  
  while (!ServoDriver.begin()) {
    Serial.println("No PCA9685 found ... check your connections");
    delay(200);
  }

  delay(10);

  Serial.println("Found PCA9685");

  while (!tcs.begin()) {
    Serial.println("No TCS34725 found ... check your connections");
    delay(500);
  }

  Serial.println("Found sensor");

  ServoDriver.begin();

  // In theory the internal oscillator (clock) is 25MHz but it really isn't
  // that precise. You can 'calibrate' this by tweaking this number until
  // you get the PWM update frequency you're expecting!
  // The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
  // is used for calculating things like writeMicroseconds()
  // Analog servos run at ~50 Hz updates, It is importaint to use an
  // oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
  // 1) Attach the oscilloscope to one of the PWM signal pins and ground on
  //    the I2C PCA9685 chip you are setting the value for.
  // 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
  //    expected value (50Hz for most ESCs)
  // Setting the value here is specific to each individual I2C PCA9685 chip and
  // affects the calculations for the PWM update frequency. 
  // Failure to correctly set the int.osc value will cause unexpected PWM results
     
  ServoDriver.setOscillatorFrequency(28000000);
  ServoDriver.setPWMFreq(50);  // Analog servos run at ~50 Hz updates (20 ms period)
  delay(10);

  interval = 50;

  setInitialServoPositions();
}

void loop() {
    uint8_t b;
    if (Serial.available()) {  // Only do this if there is serial data to be read
   
      b = Serial.read();    
      serial_commands.process_char(b);
    }  


    // To measure the time between loop() calls
    //unsigned long last_loop_micros = loop_micros; 
    
    // Do this only every "interval" miliseconds 
    unsigned long now = millis();
    if (now - last_cycle > interval) {
      loop_micros = micros();
      last_cycle = now;
      
      int i = 0;
      for(i = 0; i < NUM_SERVOS; i++) {
        ServoDriver.setPWM(i, 0, servo_pos[i]);
      }
      
      // Debug using the serial port
      //Serial.print(" state: ");
      //Serial.print(serial_commands.state);

      Serial.print(" S[0]: ");
      Serial.print(servo_pos[0]);

      Serial.print(" pwm[0]: ");
      Serial.print(ServoDriver.getPWMOff(0));

      Serial.print(" Command: ");
      Serial.print(serial_commands.command);
      Serial.print(serial_commands.data);

      Serial.print(" loop: ");
      Serial.println(micros() - loop_micros);

      setInitialServoPositions();

      moveToObject();

      pickUp();

      moveToSensor();

      readColor();
    }   
}

