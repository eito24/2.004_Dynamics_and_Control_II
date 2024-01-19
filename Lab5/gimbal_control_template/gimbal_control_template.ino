/**********************************************************************************

  Arduino code to control a dual-axis gimbal for object/image stabilization
  MIT Department of Mechanical Engineering, 2.004
  Rev. 1.0 -- 11/01/2017 H.C.
  Rev. 1.1 -- 10/21/2018 H.C.
  Rev. 2.0 -- 10/20/2021 H.C.

***********************************************************************************/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// ================================================================
// ===               CONTROLLER GAINS                           ===
// ================================================================

// axis 1: pitch
float Kp_1 = 31.11;
float Kd_1 = 3.15;
float Ki_1 = 0.0;

// axis 2: roll
float Kp_2 = 40.21;
float Kd_2 = 4.12;
float Ki_2 = 0.0;

// ================================================================
// ===               SETPOINT TYPE                              ===
// ================================================================

// Define setpoint, if all three below are commented out then the setppoint is 0 degree
// Uncomment only one setpoint at a time

//#define SQUARE_WAVE
#define SINE_WAVE
//#define POT

// Specify setpoint separately for pitch and roll
// Set it to either 0 or set_point (one of the three above)
#define set_point_1 0   // pitch axis setpoint
#define set_point_2 0   // roll axis setpoint

#define desired_pos 15           // desired position in degrees

#define square_period 4000      // square wave period in milli-seconds
#define square_on 2000          // square wave ON time in milli-seconds

#define sine_amp 15             // sine wave amplitude in degrees
#define sine_freq_hz 1        // sine wave frequency in Hz

// ================================================================
// ===               SERIAL OUTPUT CONTROL                      ===
// ================================================================

// Switch between built-in Serial Plotter and Matlab streaming
// comment out both to disable serial output

//#define PRINT_DATA
//#define MATLAB_SERIAL_READ

// ================================================================
// ===               MPU6050 VARIABLE DEFINITION                ===
// ================================================================

MPU6050 mpu;

int16_t ax, ay, az;
int16_t gx, gy, gz;

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

// ================================================================
// ===               ANGLE AND PID RELATED VARIABLES            ===
// ================================================================

float yaw, pitch, roll, roll_rate, pitch_rate;
float set_point = 0.0;
float error_1, sum_error_1 = 0.0, d_error_1 = 0.0, filt_d_error_1 = 0.0, error_pre_1;   // pitch
float error_2, sum_error_2 = 0.0, d_error_2 = 0.0, filt_d_error_2 = 0.0, error_pre_2;   // roll
float alpha = 0.5;
unsigned long timer;
float loop_time = 0.015;
float Pcontrol, Icontrol, Dcontrol;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===               MOTOR SHIELD DEFINITION                    ===
// ================================================================

const int
// Pitch: motor A
PWM_A   = 3,
DIR_A   = 12,
BRAKE_A = 9,
SNS_A   = A0,
// Roll: motor B
PWM_B   = 11,
DIR_B   = 13,
BRAKE_B = 8,
SNS_B   = A1;
// Pot connected to A3
int Pot_Pin = A3;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

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

  // Supply your own gyro offsets here, scaled for min sensitivity
  // Refer to your gimbal # and calibration spreadsheet
  // Run MPU6050_cal.ino to re-calibrate your IMU if needed

    mpu.setXAccelOffset(-4127);
    mpu.setYAccelOffset(-2910);
    mpu.setZAccelOffset(1177);
    mpu.setXGyroOffset(10);
    mpu.setYGyroOffset(-54);
    mpu.setZGyroOffset(5);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // configure motor shield A & B outputs
  pinMode(PWM_A, OUTPUT);
  pinMode(PWM_B, OUTPUT);
  pinMode(BRAKE_A, OUTPUT);
  pinMode(BRAKE_B, OUTPUT);
  pinMode(DIR_A, OUTPUT);
  pinMode(DIR_B, OUTPUT);

  // set brake: HIGH - brake on, LOW - brake off
  digitalWrite(BRAKE_A, LOW);
  digitalWrite(BRAKE_B, LOW);

  // uncomment the line below to enable the highest PWM frequency to avoid audible noise
  // may need to lower controller gains to avoid oscillation
  TCCR2B = TCCR2B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

  timer = micros();

  // ================================================================
  // ===                    GET DATA FROM MPU6050                 ===
  // ================================================================

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
    yaw = ypr[0] * 180 / M_PI;
    pitch = ypr[1] * 180 / M_PI;
    roll =  ypr[2] * 180 / M_PI;

    // get pitch and roll rates from the respective gyro axes
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    roll_rate = -1 * gx / 16.4; // full scale +/- 2000 deg/s for 16 bit AD
    pitch_rate = 1 * gy / 16.4; // full scale +/- 2000 deg/s for 16 bit AD

    // ================================================================
    // ===                    CODE FOR VARIOUS SETPOINTS            ===
    // ================================================================

    // configure set point as square wave input to give +/- desired_pos deg angles
#ifdef SQUARE_WAVE
    if (millis() % square_period < square_on)
      set_point = desired_pos;
    else
      set_point = -desired_pos;
#endif

    // configure set point as a sine wave with an amplitude of sine_amp and frequency of sine_freq_hz Hz
#ifdef SINE_WAVE
    set_point = sine_amp * sin(sine_freq_hz * (2 * M_PI) * millis() / 1000.0);;
#endif

    // configure set point as pot input to give +/- desired_pos deg angles
#ifdef POT
    set_point = map(analogRead(Pot_Pin), 0, 1023, -512, 512) / 512.0 * desired_pos;
#endif

    // ================================================================
    // ===                    PID CODE                              ===
    // ================================================================

    // PID controller for pitch axis
    error_1 = set_point_1 - pitch;
    d_error_1 = (error_1 - error_pre_1) / loop_time;
    // 1st order filter to clean up noise
    filt_d_error_1 = alpha * d_error_1 + (1 - alpha) * filt_d_error_1;
    sum_error_1 += error_1 * loop_time;
    error_pre_1 = error_1;
    motorControl(DIR_A, PWM_A, error_1, pitch_rate, sum_error_1, Kp_1, Kd_1, Ki_1);

    // PID controller for roll axis
    error_2 = set_point_2 - roll;
    d_error_2 = (error_2 - error_pre_2) / loop_time;
    // 1st order filter to clean up noise
    filt_d_error_2 = alpha * d_error_2 + (1 - alpha) * filt_d_error_2;
    sum_error_2 += error_2 * loop_time;
    error_pre_2 = error_2;
    motorControl(DIR_B, PWM_B, error_2, roll_rate, sum_error_2, Kp_2, Kd_2, Ki_2);

#ifdef PRINT_DATA
    Serial.print(set_point);  Serial.print("\t");
    Serial.print(pitch);  Serial.print("\t");
    Serial.print(roll);  Serial.print("\t");
//    Serial.print(d_error_2);  Serial.print("\t");
//        Serial.print(filt_d_error_2); Serial.print("\t");
//        Serial.print(roll_rate); Serial.print("\t");
    //    Serial.print(loop_time,6); Serial.print("\t");
    Serial.print("\n");
#endif

#ifdef MATLAB_SERIAL_READ
    Serial.print(timer / 1000000.0 - loop_time);
    Serial.print("\t");
    Serial.print(set_point);
    Serial.print("\t");
    Serial.print(pitch);
    Serial.print("\t");
    Serial.println(roll);
#endif
  }
  //  delay(2);
  loop_time = (micros() - timer) / 1000000.0;
}

//-----------------------------------------------------
void motorControl(int DIR_x, int PWM_x, float error, float d_error, float sum_error, float kp_x, float kd_x, float ki_x)
{
  float pwm_command;

  Pcontrol = error * kp_x;
  Icontrol = sum_error * ki_x;
  Dcontrol = d_error * kd_x;

  Icontrol = constrain(Icontrol, -200, 200);  // I control saturation limits for anti-windup

  pwm_command = Pcontrol + Icontrol + Dcontrol;

  if (pwm_command > 0)
  { digitalWrite(DIR_x, HIGH);
    analogWrite(PWM_x, (int) constrain(pwm_command, 0, 255));
  }
  else
  {
    digitalWrite(DIR_x, LOW);
    analogWrite(PWM_x, (int) constrain(abs(pwm_command), 0, 255));
  }
}
//------------------------------------------------------
