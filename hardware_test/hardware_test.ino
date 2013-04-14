#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Adafruit_BMP085.h> // barometer
MPU6050 mpu;
Adafruit_BMP085 bmp;



// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT



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

// packet structure for InvenSense teapot demo
//uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


void setup() {
  Serial.begin(9600);
  Serial1.begin(9600); //this is ONLY needed for Leonardo! to connect to the Bluetooth module 
  while (!Serial) {}; //uncomment this line for serial debugging.. if you dont care leave it commented out
  
  tests();
}

void loop() {

}

void tests() {
  Serial.println(F("\tTESTS\t"));  
  Serial.print(F("Gyroscope: \t"));
  gyroscope();
  Serial.print(F("Testing Barometer: \t"));
  barometer();
  Serial.print(F("bluetooth: \t"));
  bluetooth();
  delay(3000);
  Serial.print(F("motor 1: \t"));
  motor1();
  
  /* this motor 2 is broken/not working - try replacing the mosfet */
  Serial.print(F("motor 2: \t"));
  motor2(); 
  Serial.print(F("motor 3: \t"));
  motor3();
  Serial.print(F("motor 4: \t"));
  motor4();
}

void gyroscope() {
  /* mpu6050 */  
   Wire.begin();
   mpu.initialize();
   Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
}

void rename_bluetooth() {
  //rename bluetooth and set password to 6969

  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);
  delay(100);
  Serial.print(F("Setting Name: "));
  Serial1.println("AT+NAME=Quadrino");
  Serial.println("Quadrino");
  delay(1000);
  digitalWrite(12,LOW);
  delay(100);
  digitalWrite(12,HIGH);
  delay(100);
  Serial.print(F("Setting password: "));
  Serial1.println("AT+PSWD=6969");
  Serial.println("6969");
  delay(1000);
  pinMode(12, INPUT);
  digitalWrite(12, LOW);
  
}

void barometer() {
  /*BMP085*/
 if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP085 sensor, check wiring!"));
  } else {
    Serial.print("Altitude = ");
    Serial.print(bmp.readAltitude());
    Serial.println(" meters");
  }
}
void bluetooth() {
  /* comms */
  
  (Serial1) ? Serial.println(F("Works")) : Serial.println(F("Failed"));
  rename_bluetooth();
}

void motor1() {
  int led = 9;
  analogWrite(led, 50);
  delay(2000);
  Serial.println(F("Works"));
  analogWrite(led, 0);
}
void motor2() {
  /* broken */
  int led = 10;
  analogWrite(led, 50);
  delay(2000);
  Serial.println(F("Works"));
  analogWrite(led, 0);
}

void motor3() {
  int led = 11;
  analogWrite(led, 50);
  delay(2000);
  Serial.println(F("Works"));
  analogWrite(led, 0);
}

void motor4() {
  int led = 6;
  analogWrite(led, 50);
  delay(2000);
  Serial.println(F("Works"));
  analogWrite(led, 0);
}


