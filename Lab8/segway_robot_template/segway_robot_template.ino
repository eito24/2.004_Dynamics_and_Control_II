/**********************************************************************************

  Arduino code to control a two-wheeled self-balancing robot (a.k.a. Segway robot)
  MIT Department of Mechanical Engineering, 2.004
  Rev. 1.0 -- 10/20/2018 H.C.
  Rev. 1.2 -- 10/26/2018 H.C.
  Rev. 2.0 -- 10/08/2020 H.C.
  Rev. 3.0 -- 11/13/2021 Q.B. & H.C.

***********************************************************************************/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "EnableInterrupt.h"      // Use this library to enable more interrupt pins

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif


// ===                         OPTIONS                          ===

// CONTROL SCHEME
#define PID
//#define FULL_STATE

// SERIAL OUTPUT CONTROL
#define arduinoSerialPrint
// #define MATLABSerialPrint

// SETPOINT SELECTION
#define setPointFn setpoint_constant
// #define setPointFn setpoint_square_wave
// #define setPointFn setpoint_sin_wave
// #define setPointFn setpoint_circle
// #define setPointFn setpoint_yaw_turn
// #define setPointFn setpoint_figure_eight
// #define setPointFn setpoint_sonar_turn
// #define setPointFn setpoint_MY_TRACK

// Variables for setpoints:
// change tilt_offset to make the robot move forward or backward
// change lr_offset to make the robot turn right or turn left
float tilt_offset = 1.5;   // tilt angle in degrees, nominal = 0 deg
float lr_offset = 0.8;   // adjust left/right motor pwm ratio, nominal = 1

// ENABLE OR DISABLE THE TWO KNOBS FOR REAL-TIME ADJUSTMENTS
#define TRIM_POT
#define BALANCE_POT
// Adjusment values if trim and/or balance knob is disabled
float trim_val = 0.0;   // adjust this value to center the robot (in degrees) (DEFAULT = 0)
float bal_val = 1.0;    // adjust this value to make the robot move straight (DEFAULT = 1)

// ===                   CONTROL PARAMETERS                     ===
#ifdef PID
// Provide PID controller gains here
float Kp = 0.0;     // P control gain
float Kd = 0.0;     // D control gain
float Ki = 0.0;     // I control gain
#endif

#ifdef FULL_STATE
// Provide FULL STATE controller gains here
float K1 = 0.0;     // theta
float K2 = 0.0;     // theta dot
float K3 = 0.0;     // x
float K4 = 0.0;     // x dot
#endif

#define delay_start_ms 1000 // delay 1 second before tracking setpoint, change if needed

// ===                   END OF OPTIONS                         ===


// Pots connected to A1 and A2 pins 
int TRIM_PIN = A2;
int BALANCE_PIN = A1;

// ===                 IMU VARIABLE DEFINITION                  ===
MPU6050 mpu;
#define IMU_pin 2   // MPU6050 interrupt pin to pin 2 of Arduino UNO (INT 0)

int16_t ax, ay, az;
int16_t gx, gy, gz;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// ===               DFROBOT MOTOR SHIELD DEFINITION            ===

const int
// Left motor connected to M1
PWM_A   = 10,
DIR_A   = 12,

// Right motor connected to M2
PWM_B   = 11,
DIR_B   = 13;

void motorPWM();  // function to control motors

// ===               ENCODER DEFINITION                         ===

// Encoder connections for wheel measurement
// This is an encoder with 360 CRP on motor shaft and 360*34 = 12,240 CPR on the output shaft

#define C2D 68    // encoder counts to degrees factor with 2x resolution implementation
#define LF 0    // LEFT side, opposite to IMU
#define RT 1    // RIGHT side, 

//volatile long RotationCoder[2];    // for debugging, not needed

// M1 (LF) encoder with channel A to D3 and B to D4
const byte encoder0pinA = 3;
const byte encoder0pinB = 4;
boolean encoder0PinALast;
volatile int Lduration = 0;
boolean LcoderDir = true;
void LwheelEncoder(void);

// M2 (RT) encoder with channel A to D8 and B to D9
const byte encoder1pinA = 8;
const byte encoder1pinB = 9;
boolean encoder1PinALast;
volatile int Rduration = 0;
boolean RcoderDir = true;
void RwheelEncoder(void);

float wheel_pos = 0.0, wheel_speed = 0.0, pre_wheel_pos = 0.0, filt_speed = 0.0;

// ===               VARIABLE DEFINITION                          ===

float yaw, pitch, roll, roll_rate, pitch_rate;
float error, sum_error = 0.0, d_error = 0.0, filt_d_error = 0.0, error_pre;
float alpha = 0.25;    // low pass filter constant
unsigned long timer;
double loop_time = 0.015;
float Pcontrol, Icontrol, Dcontrol;
int deadband = 0;
float pwm = 0.0, pwm_L, pwm_R;
float set_point = 0.0;   // tilt angle set point

//int PULSE_pin = 7;  // ultrasonic range finder pin pulse input
//unsigned long pw;
int SONAR = A3;     // Ultra sonic range finder analog output connected to A3
float dist = 0.0, filt_dist = 0.0;


//======== Main Setup Function ============================

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);

  // initialize IMU
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  // ===               ACCEL AND GYRO OFFSETS                     ===

  // Supply your own gyro offsets here, scaled for min sensitivity
  // Refer to your robot # and calibration spreadsheet
  // Run MPU6050_cal.ino to re-calibrate your IMU if needed

  // =================================================================
  // TO DO: put in your MPU6050 IMU offsets for the robot you have
  mpu.setXAccelOffset(14);
  mpu.setYAccelOffset(45);
  mpu.setZAccelOffset(698);
  mpu.setXGyroOffset(94);
  mpu.setYGyroOffset(-33);
  mpu.setZGyroOffset(-49);
  // =================================================================

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);

    pinMode(IMU_pin, INPUT);
    // enable Arduino interrupt detection for the IMU pin
    enableInterrupt((IMU_pin), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    //     ERROR!
    //     1 = initial memory load failed
    //     2 = DMP configuration updates failed
    //     (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // configure motor shield A & B motor outputs
  pinMode(PWM_A, OUTPUT);
  pinMode(PWM_B, OUTPUT);
  pinMode(DIR_A, OUTPUT);
  pinMode(DIR_B, OUTPUT);

#ifdef FULL_STATE
  // initialize encoder
  // 2x resolution of encoder, that is, 24,480 counts per revolution
  LcoderDir = true;     // LF
  pinMode(encoder0pinB, INPUT_PULLUP);
  pinMode(encoder0pinA, INPUT_PULLUP);
  enableInterrupt((encoder0pinA), LwheelEncoder, CHANGE);

  RcoderDir = true;       // RT, comment out this part if only one encoder is used
  pinMode(encoder1pinB, INPUT_PULLUP);
  pinMode(encoder1pinA, INPUT_PULLUP);
  enableInterrupt((encoder1pinA), RwheelEncoder, CHANGE);
#endif

  delay(1000);    // delay one second
}

//======== Main Loop Function ============================

void loop() {

  timer = micros();

  // ===                    READ DATA FROM POTS                   ===
#ifdef TRIM_POT
  trim_val = (analogRead(TRIM_PIN) - 512.0) / 256.0; // +/- 2 degrees
#endif

#ifdef BALANCE_POT
  bal_val = map(analogRead(BALANCE_PIN), 0, 1023, 75, 125) / 100.0; // constrain to +/- 25%
#endif

  // ===                    GET DATA FROM MPU6050                 ===

  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    ;
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
  }
  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // display Yaw, Pitch, and Roll angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // based on IMU orientation, pitch angle and pitch rate are the two measurements needed
    yaw = ypr[0] * 180 / M_PI;
    pitch = ypr[1] * 180 / M_PI;
    //    roll =  ypr[2] * 180 / M_PI;

    // get pitch rate from the respective gyro axes
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    pitch_rate = 1 * gy / 16.4; // full scale +/- 2000 deg/s for 16 bit AD
    //    z_accel = az / 16384.0;


    // ===                    GET SONAR DISTANCE                    ===

    dist = (analogRead(SONAR) / 2.0) * 2.54; // cm
    filt_dist = alpha * dist + (1 - alpha) * filt_dist;

    // ===                    GET SETPOINT                          ===

    setPointFn();

    // ===                    CONTROLLER CODE                       ===

    pitch = pitch - trim_val;
    error = set_point - pitch;
    //    d_error = (error - error_pre) / loop_time;
    //    filt_d_error = alpha * d_error + (1 - alpha) * filt_d_error;
    //    error_pre = error;
    sum_error += error * loop_time;

#ifdef PID
    // PID controller code
    Pcontrol = error * Kp;
    Icontrol = sum_error * Ki;
    Dcontrol = pitch_rate * Kd;   // use pitch_rate for better performance

    Icontrol = constrain(Icontrol, -255, 255);    // anti-windup for integrator
    pwm = Pcontrol + Icontrol + Dcontrol;
#endif

#ifdef FULL_STATE
    // wheel position and speed from each motor encoder
    wheel_pos += (Lduration - Rduration) * 0.5; // if both encoders are used
    wheel_pos = -1 * wheel_pos / C2D;  // convert to degrees and change direction
    wheel_speed = (wheel_pos - pre_wheel_pos);  // instantaneous speed
    //    filt_speed = alpha * wheel_speed + (1 - alpha) * filt_speed;
    pre_wheel_pos = wheel_pos;
    pwm = error * K1 + pitch_rate * K2 + wheel_pos * K3 + wheel_speed * K4;
#endif

    // ===                PASS THE PWM TO MOTORS                      ===

    motorPWM();

    // ===                    SERIAL OUTPUT SECTION                    ===

#ifdef arduinoSerialPrint
    Serial.print(set_point, 4);  Serial.print("\t");
    Serial.print(pitch, 4);  Serial.print("\t");
    Serial.print(error, 4);  Serial.print("\t");
    //    Serial.print(yaw); Serial.print("\t");
    //    Serial.print(pwm);  Serial.print("\t");
    //    Serial.print(loop_time, 6); Serial.print("\t");
    //    Serial.print(trim_val); Serial.print("\t");
    //    Serial.print(bal_val); Serial.print("\t");
    //    Serial.print(wheel_pos); Serial.print("\t");
    //    Serial.print(wheel_speed); Serial.print("\t");
    //    Serial.print(Lduration); Serial.print("\t");
    //    Serial.print(Rduration); Serial.print("\t");
    //    Serial.print(pwm_L); Serial.print("\t");
    //    Serial.print(pwm_R); Serial.print("\t");
    //    Serial.print(dist); Serial.print("\t");
    //    Serial.print(filt_dist); Serial.print("\t");
    Serial.print("\n");
#endif

#ifdef MATLABSerialPrint
    Serial.print(timer / 1000000.0, 4);   Serial.print("\t");
    Serial.print(set_point, 4);    Serial.print("\t");
    Serial.print(pitch, 4);    Serial.print("\t");
    Serial.print(error, 4);
    Serial.print("\n");
#endif

  }
  //  delay(2);

  Lduration = 0;
  Rduration = 0;

  loop_time = (micros() - timer) / 1000000.0;  //compute actual sample time
}

// ======== End of Main Loop Function ========================

// ======== Left and right encoder functions =================

void LwheelEncoder()
{
  boolean Lstate = digitalRead(encoder0pinA);
  if (encoder0PinALast == LOW && Lstate == HIGH)
  {
    boolean val = digitalRead(encoder0pinB);
    if (val == LOW && LcoderDir)  LcoderDir = false; //Reverse
    else if (val == HIGH && !LcoderDir)  LcoderDir = true; //Forward
  }
  encoder0PinALast = Lstate;

  if (!LcoderDir)  Lduration--;
  else  Lduration++;

  //  if(true)  RotationCoder[LF] ++;
  //  else RotationCoder[LF] = 0;
}

void RwheelEncoder()
{
  boolean Rstate = digitalRead(encoder1pinA);
  if (encoder1PinALast == LOW && Rstate == HIGH)
  {
    boolean val = digitalRead(encoder1pinB);
    if (val == LOW && RcoderDir)  RcoderDir = false; //Reverse
    else if (val == HIGH && !RcoderDir)  RcoderDir = true; //Forward
  }
  encoder1PinALast = Rstate;

  if (!RcoderDir)  Rduration--;
  else  Rduration++;

  //  if(true)  RotationCoder[RT] ++;
  //  else RotationCoder[RT] = 0;
}

//========= Motor control function =====================

void motorPWM()
{
  // set the correct direction
  if (pwm > 0) {
    digitalWrite(DIR_A, LOW);
    digitalWrite(DIR_B, HIGH);
  }
  else {
    digitalWrite(DIR_A, HIGH);
    digitalWrite(DIR_B, LOW);
  }

  // constrain pwm to +/- 255
  pwm = constrain(pwm, -255, 255);

  // minumum pwm to overcome deadband
  if (deadband > 0) {
    if (pwm < deadband && pwm >= 0)
      pwm = deadband;
    if (pwm > -deadband && pwm < 0)
      pwm = -deadband;
  }

  // set pwm to 0 to stop both motors if tilt angle exceeds 15 degrees
  if (abs(pitch) >= 15) pwm = 0.0;

  // Left-Right motor pwm adjustments
  pwm_L = pwm * bal_val;
  pwm_R = pwm * (2.0 - bal_val);

  pwm_L = constrain(pwm_L, -255, 255);
  pwm_R = constrain(pwm_R, -255, 255);

  analogWrite(PWM_A, (int) abs(pwm_L));     // Set the speed of the left motor
  analogWrite(PWM_B, (int) abs(pwm_R));     // Set the speed of the right motor
}

//============ SETPOINT FUNCTIONS ===============

void setpoint_MY_TRACK() {
// program your robot to navigate through the track

}

void setpoint_constant() {
  set_point = 0.0;
}

void setpoint_square_wave() {
  // define set point as square wave input here to give +/- tilt_offset deg tilt angles

  if (millis() >= delay_start_ms) {
    if (millis() % 3000 < 1500)
      set_point = tilt_offset;
    else
      set_point = -1 * tilt_offset;
  }
}

void setpoint_sin_wave() {
  // define set point as a sine wave with an amplitude of tilt_offset and frequency in rad/s
  float ang_freq = 17.9;
  if (millis() >= delay_start_ms) {
    set_point = tilt_offset * sin(ang_freq * millis() / 1000.0);
  }

}

void setpoint_circle() {
  // define set point as a circle with tilt_offset and lr_offset defined at the top of the file
  if (millis() >= delay_start_ms) {
    set_point = tilt_offset;
    bal_val = lr_offset;
  }
}

void setpoint_yaw_turn() {
  int t1 = 5000;

  if (millis() >= delay_start_ms) {
    if (millis() < t1) {
      set_point = 1.0;
      bal_val = 1.0;
    }
    else if (millis()  > t1 && abs(yaw) < 90) {
      set_point = 1.0;
      bal_val = 0.8;
    }
    else {
      set_point = 1.0;
      bal_val = 1.0;
    }
  }
}

void setpoint_figure_eight() {
  int t_period = 12000;
  int t1 = 5800;
  int t2 = 6000;
  int t3 = 11800;

  if (millis() >= delay_start_ms) {
    if (millis() % t_period < t1) {
      set_point = 1.0;
      bal_val = 0.8;
    }
    else if (millis() % t_period > t1 && millis() % t_period < t2) {
      set_point = 1.0;
      bal_val = 1.0;
    }
    else if (millis() % t_period > t2 && millis() % t_period < t3) {
      set_point = -0.6;
      bal_val = 1.2;
    }
    else {
      set_point = 1.0;
      bal_val = 1.0;
    }
  }
}

void setpoint_sonar_turn() {
  if (millis() >= delay_start_ms) {
    if (filt_dist >= 30)  // distance in cm
      set_point = tilt_offset;
    else
      bal_val -= lr_offset;
  }
}
