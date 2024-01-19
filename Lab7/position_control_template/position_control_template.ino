/**********************************************************************************

  Arduino code to control a DC motor
  MIT Department of Mechanical Engineering, 2.004
  Rev. 1.0 -- 08/03/2020 H.C.
  Rev. 2.0 -- 08/25/2020 H.C.
  Rev. 3.0 -- 04/04/2021 H.C.
  Rev. 4.0 -- 11/04/2021 H.C.

***********************************************************************************/

//#define OPEN_LOOP_PLANT
//#define OPEN_LOOP_LEAD
//#define PID           // closed-loop
#define LEAD_LAG      // closed-loop

#define vc2pwm 51.0    // pwm = 255 corresponds to 5v from pwm pin (255/5)
#define period_us 10000  // microseconds (1 sec = 1000000 us)

// ================================================================
// ===               PID FEEDBACK CONTROLLER                    ===
// ================================================================

// Provide PID controller gains here

float Kp = 100.0;
float Kd = 0.0;
float Ki = 0.0;

// ================================================================
// ===               LEAD-LAG CONTROLLER                        ===
// ================================================================

// Lead or Lag controller format:
//
//            T_1 * s + 1
//  Gc = Kc * -------------
//            T_2 * s + 1

float Kc = 106.01;     // gain
float T_1 = 0.14;    // time constant of the zero
float T_2 = 0.03;    // time constant of the pole
float x = 0, x_new = 0;      // intermediate variable

// ================================================================
// ===               SETPOINT TYPE                              ===
// ================================================================

// Define setpoint

#define SQUARE_WAVE
//#define SINE_WAVE
//#define POT
//#define CHIRP
//#define EXP_CHIRP

#define desired_pos PI/2        // desired position in rad

#define square_period 3000      // square wave period in milli-seconds
#define square_on 1500          // square wave on time in milli-seconds

#define sine_amp PI/3         // sine wave amplitude
#define sine_freq_hz 15.4/(2*PI)        // sine wave frequency in Hz

// chirp constants
#define phi_0 0   // initial phase angle 
#define f0_hz 0.05   // initial frequency in Hz
#define f1_hz 10.0     // final frequency in Hz
#define T 100.0 // totsal chirp duration in secs 
float chirp_amp = PI / 3; // chirp signal amplitude
float k = pow((f1_hz / f0_hz), 1 / T); // exponential chirp parameter

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

const int
// motor connected to M1 (DRI0017)
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
double loop_time = period_us / 1000000.0;
float Pcontrol, Icontrol, Dcontrol;
float pwm;
float vc;
float set_point = desired_pos;   // initial set point

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

  Serial.begin(115200);

  // configure motor shield M1 outputs
  pinMode(PWM_A, OUTPUT);
  pinMode(DIR_A, OUTPUT);

  // initialize encoder
  pinMode(encoder0pinA, INPUT_PULLUP);
  pinMode(encoder0pinB, INPUT_PULLUP);
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
      set_point = desired_pos;
    else
      set_point = 0.0;
#endif

    // define set point as a sine wave with an amplitude and frequency
#ifdef SINE_WAVE
    set_point = sine_amp * sin(sine_freq_hz * (2 * M_PI) * millis() / 1000.0);
#endif

    // use potentiometer to adjust setpoint
#ifdef POT
    set_point = map(analogRead(Pot_Pin), 0, 1023, -512, 512) / 512.0 * PI;
#endif

#ifdef CHIRP
    if ( millis() / 1000.0 <= T)
      set_point = chirp_amp * sin(phi_0 + 2 * M_PI * ((f1_hz - f0_hz) / (2 * T) * pow((millis() / 1000.0), 2) + f0_hz * millis() / 1000.0));
    else
      set_point = 0.0;
#endif

#ifdef EXP_CHIRP
    if ( millis() / 1000.0 <= T) {
#ifdef PID
      chirp_amp = PI / 3; // for P control
#endif
#ifdef LEAD_LAG
      //    chirp_amp = 2.0 - ((2.0 - 0.1) / T) * millis() / 1000.0; // for closed loop lead control (linear decay)
      chirp_amp = 4.0 * exp(-(1 / 40.0) * millis() / 1000.0); // for closed loop lead control (exponential decay)
#endif
      set_point = chirp_amp * sin(phi_0 + 2 * M_PI * f0_hz * ((pow(k, (millis() / 1000.0)) - 1) / log(k)));
    }
    else
      set_point = 0.0;
#endif

    // ================================================================
    // ===                    OPEN LOOP SETTINGS                    ===
    // ================================================================

#ifdef OPEN_LOOP_PLANT
    chirp_amp = 125;
    pwm = set_point;
#endif

#ifdef OPEN_LOOP_LEAD
    chirp_amp = 2.5 - ((2.5 - 0.7) / T) * millis() / 1000.0;
    x_new = (1 - loop_time / T_2) * x + (loop_time / T_2) * set_point * Kc;
    pwm = (1 - T_1 / T_2) * x + (T_1 / T_2) * set_point * Kc;
    x = x_new;
#endif

    // ================================================================
    // ===                    CONTROLLER CODE                       ===
    // ================================================================

    // PID controller
    error = set_point - wheel_pos;
    d_error = (error - error_pre) / loop_time;
    filt_d_error = alpha * d_error + (1 - alpha) * filt_d_error;
    error_pre = error;
    sum_error += error * loop_time;

#ifdef PID
    Pcontrol = error * Kp;
    Icontrol = sum_error * Ki;
    Dcontrol = d_error * Kd;    // use d_error to avoid delay

    Icontrol = constrain(Icontrol, -255, 255);    // anti-windup for integrator
    pwm =  Pcontrol + Icontrol + Dcontrol;
#endif

#ifdef LEAD_LAG
    x_new = (1 - loop_time / T_2) * x + (loop_time / T_2) * error * Kc;
    pwm = (1 - T_1 / T_2) * x + (T_1 / T_2) * error * Kc;
    x = x_new;
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
    Serial.print(wheel_pos); Serial.print("\t");
    //    Serial.print(filt_vel); Serial.print("\t");
    //    Serial.print(error);  Serial.print("\t");
    //    Serial.print(d_error);  Serial.print("\t");
    Serial.print(vc);  Serial.print("\t");
    //      Serial.print(pwm);  Serial.print("\t");
    //    Serial.print(loop_time,6); Serial.print("\t");
    Serial.print("\n");
#endif

#ifdef MATLAB_SERIAL_READ
    Serial.print(timer / 1000000.0, 4);   Serial.print("\t");
    Serial.print(set_point, 4);    Serial.print("\t");
    Serial.print(wheel_pos, 4);    Serial.print("\t");
    Serial.print(pwm / 100.0, 4);  // PWM/100 to scale the pwm signal
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
