/**********************************************************************************

  Arduino code to control a DC motor
  MIT Department of Mechanical Engineering, 2.004
  Rev. 1.0 -- 08/03/2020 H.C.
  Rev. 2.0 -- 08/25/2020 H.C.
  Rev. 3.0 -- 04/04/2021 H.C.
  Rev. 4.0 -- 09/29/2021 H.C.

***********************************************************************************/

#define vc2pwm 51.0    // pwm = 255 corresponds to 5v from pwm pin (255/5)
#define period_us 10000  // microseconds (1 sec = 1000000 us)

// ================================================================
// ===               PID FEEDBACK CONTROLLER                    ===
// ================================================================

// Provide PID controller gains here

float Kp = 11.63;
float Ki = 87.21;
float Kd = 0.0;

// ================================================================
// ===               SETPOINT TYPE                              ===
// ================================================================

// Define setpoint

#define desired_vel 15        // desired velocity in rad/s

#define square_period 4000      // square wave period in milli-seconds
#define square_on 2000          // square wave on time in milli-seconds

#define sine_amp 10         // sine wave amplitude
#define sine_freq_hz 1.0        // sine wave frequency in Hz

#define max_speed 25        // max velocity in rad/s

//#define SQUARE_WAVE
//#define SINE_WAVE
//#define POT
//#define OPEN_LOOP

// ================================================================
// ===               SERIAL OUTPUT CONTROL                      ===
// ================================================================

// Switch between built-in Serial Plotter and Matlab streaming
// comment out both to disable serial output

//#define PRINT_DATA
#define MATLAB_SERIAL_READ

// ================================================================
// ===               DFROBOT MOTOR SHIELD DEFINITION            ===
// ================================================================

// DRI0017 motor shield
const int
PWM_A   = 10,
DIR_A   = 12;

int Pot_Pin = A5;    // select the input pin for the potentiometer

// ================================================================
// ===               ENCODER DEFINITION                         ===
// ================================================================

// The motor has a dual channel encoder and each has 12 counts per revolution (CPR)
// The effective resolution is 48 CPR with quadrature decoding. With gear reduction
// of 34, the motor output shaft's encoder resolution becomes 34*48 = 1632 CPR
// Manually calibrate the conversion factor if necessary.

float C2Rad = 34 * 12 * 4 / (2 * PI); // nominal encoder counts to radians factor

// Encoder outputs to interrupt pins with channel A to D2 (INT0) and B to D3 (INT1)
const byte encoder0pinA = 2;
const byte encoder0pinB = 3;

long   count = 0;
volatile long counter = 0;    // Inrerrupt counter (position)

// ================================================================
// ===               VARIABLE DEFINITION                        ===
// ================================================================

float wheel_pos = 0.0, wheel_vel = 0.0, pre_wheel_pos = 0.0, filt_vel = 0.0;
float error, sum_error = 0.0, d_error = 0.0, filt_d_error = 0.0, error_pre;
float alpha = 0.5;    // low pass filter constant
unsigned long timer;
double loop_time = 0.01;
float Pcontrol, Icontrol, Dcontrol;
float pwm = 0;
float vc;
float set_point = desired_vel;   // initial set point

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

  Serial.begin(115200);

  // configure motor shield M1 outputs
  pinMode(PWM_A, OUTPUT);
  pinMode(DIR_A, OUTPUT);

  // initialize encoder
  pinMode(encoder0pinA, INPUT);
  pinMode(encoder0pinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder0pinA), ISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder0pinB), ISR_B, CHANGE);

  delay(10);    // delay 0.01 second
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  if (micros() - timer >= period_us) {
    timer = micros();
    count = counter;

    // wheel position and velocity from the encoder counts
    wheel_pos = count / C2Rad;    // convert to radians
    wheel_vel = (wheel_pos - pre_wheel_pos) / loop_time;
    filt_vel = alpha * wheel_vel + (1 - alpha) * filt_vel;
    pre_wheel_pos = wheel_pos;

    // ================================================================
    // ===                    GET SETPOINT                          ===
    // ================================================================

    // define set point as a square wave input
#ifdef SQUARE_WAVE
    if (millis() % square_period < square_on)
      set_point = desired_vel;
    else
      set_point = 0.0;
#endif

    // define set point as a sine wave with an amplitude and frequency
#ifdef SINE_WAVE
    set_point = sine_amp * sin(sine_freq_hz * (2 * M_PI) * millis() / 1000.0);
#endif

    // a variable set point cotrolled by the potentiometer
#ifdef POT
    set_point = map(analogRead(Pot_Pin), 0, 1023, -512, 512) / 512.0 * max_speed;
#endif

    // ================================================================
    // ===                    CONTROLLER CODE                       ===
    // ================================================================

    // TO DO: PID controller code
    error = set_point - filt_vel;
    d_error = (error-error_pre)/loop_time;
    error_pre = error;
    sum_error = sum_error + error*loop_time;

    Pcontrol = Kp*error;
    Icontrol = Ki*sum_error;
    Dcontrol = Kd*d_error;

    Icontrol = constrain(Icontrol, -255, 255);    // anti-windup for I control
    pwm =  Pcontrol + Icontrol + Dcontrol;

#ifdef OPEN_LOOP
    pwm = 150;
#endif

    // ================================================================
    // ===                    MOTOR PWM CODE                        ===
    // ================================================================

    vc = constrain(pwm / vc2pwm, -5, 5);

    // set the correct direction
    if (pwm > 0) {
      digitalWrite(DIR_A, LOW);
    }
    else {
      digitalWrite(DIR_A, HIGH);
    }

    // constrain pwm to +/- 230
    pwm = constrain(pwm, -255, 255);

    analogWrite(PWM_A, (int) abs(pwm));     // Set speed

#ifdef PRINT_DATA
    Serial.print(set_point);  Serial.print("\t");
    //    Serial.print(wheel_pos); Serial.print("\t");
    Serial.print(filt_vel); Serial.print("\t");
    //    Serial.print(error);  Serial.print("\t");
    //    Serial.print(d_error);  Serial.print("\t");
    Serial.print(vc);  Serial.print("\t");
    //    Serial.print(pwm);  Serial.print("\t");
    //    Serial.print(loop_time,6); Serial.print("\t");
    Serial.print("\n");
#endif

#ifdef MATLAB_SERIAL_READ
    Serial.print(timer / 1000000.0);   Serial.print("\t");
    Serial.print(set_point);    Serial.print("\t");
    Serial.print(filt_vel);    Serial.print("\t");
    Serial.print(vc);
    Serial.print("\n");
#endif

    //  delay(8);
  }
  loop_time = (micros() - timer) / 1000000.0;  //compute actual sample time
}

// ================================================================
// ===                   Interrupt Service Routines             ===
// ================================================================

// Interrupt Service Routine for encoder A (pin 2)
void ISR_A() {
  boolean stateA = digitalRead(encoder0pinA) == HIGH;
  boolean stateB = digitalRead(encoder0pinB) == HIGH;
  if ((!stateA && stateB) || (stateA && !stateB)) counter++;
  else counter--;
}

// Interrupt Service Routine for encoder B (pin 3)
void ISR_B() {
  boolean stateA = digitalRead(encoder0pinA) == HIGH;
  boolean stateB = digitalRead(encoder0pinB) == HIGH;
  if ((!stateA && !stateB) || (stateA && stateB)) counter++;
  else counter--;
}
