#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Adafruit_BMP085.h> // barometer
MPU6050 mpu;
Adafruit_BMP085 bmp;

#define DEBUG_SERIAL //endable this for debugging

#define OUTPUT_READABLE_REALACCEL


#define OUTPUT_READABLE_WORLDACCEL

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

int incomingByte = 0; 


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


void setup() {

  Serial1.begin(9600); //this is ONLY needed for Leonardo! to connect to the Bluetooth module 
  #ifdef DEBUG_SERIAL
  Serial.begin(9600);
  
  while (!Serial) {}; //uncomment this line for serial debugging.. if you dont care leave it commented out
  delay(1000);
  Serial.println(F("Serial data coming;;"));
  #endif
  
  
  tests();
}

void loop() {

}

void serialRead() {
  //#ifdef DEBUG_SERIAL
  while (Serial1.available()) {
    // read the incoming byte:
    incomingByte = Serial.read();
  
    // say what you got:
    Serial.print("received: ");
    Serial.println(incomingByte, DEC);
  }
  //#endif
}
void tests() {
  #ifdef DEBUG_SERIAL
  Serial.println(F("\tTESTS\t"));  
  Serial.print(F("Gyroscope: \t"));
  #endif
  gyroscope();
  #ifdef DEBUG_SERIAL
  Serial.print(F("Testing Barometer: \t"));
  #endif
  barometer();
  #ifdef DEBUG_SERIAL
  Serial.print(F("bluetooth: \t"));
  #endif
  bluetooth();
  delay(3000);
  #ifdef DEBUG_SERIAL
  Serial.print(F("motor 1: \t"));
  #endif
  motor1();
  
  /* this motor 2 is broken/not working - try replacing the mosfet */
  #ifdef DEBUG_SERIAL
  Serial.print(F("motor 2: \t"));
  #endif
  motor2(); 
  #ifdef DEBUG_SERIAL
  Serial.print(F("motor 3: \t"));
  #endif
  motor3();
  #ifdef DEBUG_SERIAL
  Serial.print(F("motor 4: \t"));
  #endif
  motor4();
}

void gyroscope() {
  /* mpu6050 */  
   Wire.begin();
   mpu.initialize();
   #ifdef DEBUG_SERIAL
   Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
   #endif
}

void rename_bluetooth() {
  //rename bluetooth and set password to 6969

  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);
  delay(100);
  serialRead();
  #ifdef DEBUG_SERIAL
  Serial.print(F("Setting Name: "));
  #endif
  Serial1.println("AT+NAME=Quadrino");
  serialRead();
  #ifdef DEBUG_SERIAL
  Serial.println("Quadrino");
  #endif
  delay(1000);
  digitalWrite(12,LOW);
  delay(100);
  digitalWrite(12,HIGH);
  delay(100);
  #ifdef DEBUG_SERIAL
  Serial.print(F("Setting password: "));
  #endif
  Serial1.println("AT+PSWD=6969");
  #ifdef DEBUG_SERIAL
  serialRead();
  Serial.println("6969");
  #endif
  delay(1000);
  pinMode(12, INPUT);
  digitalWrite(12, LOW);
  
}

void barometer() {
  /*BMP085*/
 if (!bmp.begin()) {
   #ifdef DEBUG_SERIAL
    Serial.println(F("Could not find a valid BMP085 sensor, check wiring!"));
    #endif
  } else {
    #ifdef DEBUG_SERIAL
    Serial.print("Altitude = ");
    Serial.print(bmp.readAltitude());
    Serial.println(" meters");
    #endif
  }
}
void bluetooth() {
  /* comms */
  
  if (Serial1) { 
    serialRead();
    #ifdef DEBUG_SERIAL
    Serial.println(F("Works")) ;
    #endif
  }else {
    #ifdef DEBUG_SERIAL
    Serial.println(F("Failed"));
    #endif
  }
  rename_bluetooth();
}

void motor1() {
  int led = 9;
  analogWrite(led, 50);
  delay(2000);
  #ifdef DEBUG_SERIAL
  Serial.println(F("Works"));
  #endif
  analogWrite(led, 0);
}
void motor2() {
  /* broken */
  int led = 10;
  analogWrite(led, 50);
  delay(2000);
  #ifdef DEBUG_SERIAL
  Serial.println(F("Works"));
  #endif
  analogWrite(led, 0);
}

void motor3() {
  int led = 11;
  analogWrite(led, 50);
  delay(2000);
  #ifdef DEBUG_SERIAL
  Serial.println(F("Works"));
  #endif
  analogWrite(led, 0);
}

void motor4() {
  int led = 6;
  analogWrite(led, 50);
  delay(2000);
  #ifdef DEBUG_SERIAL
  Serial.println(F("Works"));
  #endif
  analogWrite(led, 0);
}


